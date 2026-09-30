// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// The interpreter's floating point against Unicorn's Cortex-M4 (QEMU), bit
// for bit: every FPv4-SP data-processing instruction on every combination of
// awkward operands (signed zeros, subnormals, infinities, halves, the limits
// of the integer conversions, quiet and signalling NaNs with payloads), the
// conversions in each rounding mode, and the flags compares leave. difftest
// compares results exactly, NaNs too, and firmware seldom makes a NaN; this
// is where the two are shown to agree on them.
//
//   make fpu

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unicorn/unicorn.h>

#include "vfc.h"
#include "vfc_internal.h"

static uint8_t image[4096];
#define CODE 0x100u

static void put16(uint32_t offset, uint16_t v)
{
    image[offset] = (uint8_t)v;
    image[offset + 1] = (uint8_t)(v >> 8);
}

static void put32(uint32_t offset, uint32_t v)
{
    put16(offset, (uint16_t)v);
    put16(offset + 2, (uint16_t)(v >> 16));
}

static void check(uc_err err, const char *what)
{
    if (err != UC_ERR_OK) {
        fprintf(stderr, "unicorn: %s: %s\n", what, uc_strerror(err));
        exit(2);
    }
}

// Each instruction takes Sd = s0, Sn = s1, Sm = s2, and varies the operands
// it reads: d, n and m; d and m; d alone; or m alone. Conversions to integer
// that follow FPSCR run in each of the four rounding modes.
enum { DNM, DM, D, M, M_ROUNDED };

typedef struct {
    const char *name;
    uint16_t hw1, hw2;
    int reads;
} op_t;

#define DATA(opc1, op) (uint16_t)(0xEE00 | (((opc1) >> 2) << 7) | (((opc1) & 3) << 4)), \
                       (uint16_t)(0x0A00 | (1u << 7) | ((op) << 6) | 1)
#define OTHER(opc2, bit7, bit6) (uint16_t)(0xEEB0 | (opc2)), (uint16_t)(0x0A00 | ((bit7) << 7) | ((bit6) << 6) | 1)
#define WITH_ZERO(e) (uint16_t)0xEEB5, (uint16_t)(0x0A40 | ((e) << 7))

static const op_t ops[] = {
    { "vmla", DATA(0, 0), DNM }, { "vmls", DATA(0, 1), DNM },
    { "vnmls", DATA(1, 0), DNM }, { "vnmla", DATA(1, 1), DNM },
    { "vmul", DATA(2, 0), DNM }, { "vnmul", DATA(2, 1), DNM },
    { "vadd", DATA(3, 0), DNM }, { "vsub", DATA(3, 1), DNM },
    { "vdiv", DATA(4, 0), DNM },
    { "vfnms", DATA(5, 0), DNM }, { "vfnma", DATA(5, 1), DNM },
    { "vfma", DATA(6, 0), DNM }, { "vfms", DATA(6, 1), DNM },
    { "vmov", OTHER(0x0, 0, 1), M }, { "vabs", OTHER(0x0, 1, 1), M },
    { "vneg", OTHER(0x1, 0, 1), M }, { "vsqrt", OTHER(0x1, 1, 1), M },
    { "vcmp", OTHER(0x4, 0, 1), DM }, { "vcmpe", OTHER(0x4, 1, 1), DM },
    { "vcmp #0", WITH_ZERO(0), D }, { "vcmpe #0", WITH_ZERO(1), D },
    { "vcvt.f32.u32", OTHER(0x8, 0, 1), M }, { "vcvt.f32.s32", OTHER(0x8, 1, 1), M },
    { "vcvtr.u32.f32", OTHER(0xC, 0, 1), M_ROUNDED }, { "vcvt.u32.f32", OTHER(0xC, 1, 1), M },
    { "vcvtr.s32.f32", OTHER(0xD, 0, 1), M_ROUNDED }, { "vcvt.s32.f32", OTHER(0xD, 1, 1), M },
};
#define NOPS (int)(sizeof(ops) / sizeof(ops[0]))

static const uint32_t values[] = {
    0x00000000, 0x80000000,                         // zeros
    0x3F800000, 0xBF800000, 0x40000000, 0x40400000, // 1, -1, 2, 3
    0x3F000000, 0xBF000000, 0x3FC00000, 0xC0200000, // 0.5, -0.5, 1.5, -2.5: ties
    0x7F7FFFFF, 0xFF7FFFFF, 0x7F000000,             // the largest
    0x00800000, 0x007FFFFF, 0x00000001, 0x807FFFFF, // smallest normal, subnormals
    0x4F000000, 0xCF000000, 0xCF000001, 0x4F7FFFFF, 0x4F800000, // 2^31, -2^31, the limits
    0x7F800000, 0xFF800000,                         // infinities
    0x7FC00000, 0xFFC00000, 0x7FC12345, 0xFFE54321, 0x7FFFFFFF, // quiet NaNs
    0x7F800001, 0x7FA12345, 0xFF812345, 0xFFBFFFFF, // signalling NaNs
};
#define NVALUES (int)(sizeof(values) / sizeof(values[0]))

static vfc_t *vfc;
static uc_engine *uc;
static long runs, differences;
static int differencesBy[NOPS];

static void run(int i, const uint32_t in[3], uint32_t fpscr)
{
    const op_t *o = &ops[i];
    const uint32_t pc = VFC_FLASH_BASE + CODE + 8u * (uint32_t)i;

    vfc_cpu_t *cpu = &vfc->cpu;
    cpu->r[15] = pc;
    cpu->itstate = 0;
    cpu->fpscr = fpscr;
    memset(cpu->s.u, 0, sizeof(cpu->s.u));
    memcpy(cpu->s.u, in, 3 * sizeof(uint32_t));
    vfc->stopRequested = false;
    vfc_run(vfc, 1);

    uint32_t s[32], ucFpscr, ucPc;
    check(uc_reg_write(uc, UC_ARM_REG_FPSCR, &fpscr), "write fpscr");
    for (int r = 0; r < 32; r++) {
        check(uc_reg_write(uc, UC_ARM_REG_S0 + r, r < 3 ? &in[r] : &(uint32_t){ 0 }), "write s");
    }
    check(uc_emu_start(uc, pc | 1, 0xFFFFFFFFu, 0, 1), o->name);
    for (int r = 0; r < 32; r++) {
        uc_reg_read(uc, UC_ARM_REG_S0 + r, &s[r]);
    }
    uc_reg_read(uc, UC_ARM_REG_FPSCR, &ucFpscr);
    uc_reg_read(uc, UC_ARM_REG_PC, &ucPc);
    runs++;

    if (cpu->r[15] != pc + 4 || ucPc != pc + 4) {
        fprintf(stderr, "%s did not run: pc %08x, unicorn %08x %s\n", o->name, cpu->r[15], ucPc, vfc->fault);
        exit(2);
    }
    // FPSCR's cumulative exception bits are not modelled; its flags are.
    const bool same = !memcmp(cpu->s.u, s, sizeof(s)) && (cpu->fpscr >> 28) == (ucFpscr >> 28);
    if (same) {
        return;
    }
    differences++;
    if (differencesBy[i]++ < 4) {
        printf("%-14s d %08x n %08x m %08x FPSCR %08x: s0 %08x, NZCV %x; Unicorn s0 %08x, NZCV %x\n",
               o->name, in[0], in[1], in[2], fpscr, cpu->s.u[0], cpu->fpscr >> 28, s[0], ucFpscr >> 28);
    }
}

int main(void)
{
    // A vector table and board info, then each instruction followed by WFI.
    put32(0x00, VFC_RAM_BASE + VFC_RAM_SIZE);
    put32(0x04, VFC_FLASH_BASE + 0x40 + 1);
    put32(0x1C, VFC_FLASH_BASE + 0x20);
    put32(0x20, 0x56464331u);
    put32(0x24, VFC_ABI);
    put32(0x28, VFC_MAILBOX_BASE);
    put32(0x2C, VFC_RAM_BASE + 0x70000);
    put32(0x30, 0x1000);
    put32(0x34, VFC_FLASH_BASE + 0x38);
    memcpy(image + 0x38, "fpu", 4);
    put16(0x40, 0xBF30);
    for (int i = 0; i < NOPS; i++) {
        put16(CODE + 8u * (uint32_t)i, ops[i].hw1);
        put16(CODE + 8u * (uint32_t)i + 2, ops[i].hw2);
        put16(CODE + 8u * (uint32_t)i + 4, 0xBF30);
    }
    const uint32_t length = CODE + 8u * NOPS;

    vfc = vfc_create();
    vfc_set_jit(vfc, false);
    if (vfc_load(vfc, image, length, VFC_FLASH_BASE) != VFC_OK) {
        fprintf(stderr, "load failed\n");
        return 2;
    }

    check(uc_open(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS, &uc), "open");
    check(uc_ctl_set_cpu_model(uc, UC_CPU_ARM_CORTEX_M4), "cpu model");
    check(uc_mem_map(uc, VFC_FLASH_BASE, 0x10000, UC_PROT_READ | UC_PROT_EXEC), "map flash");
    check(uc_mem_write(uc, VFC_FLASH_BASE, image, length), "write flash");
    check(uc_mem_map(uc, VFC_RAM_BASE, 0x10000, UC_PROT_ALL), "map RAM");
    // Open an FP context with one instruction (vmov.f32 s0, s0), as difftest
    // does, so that QEMU keeps the FPSCR written before each run.
    const uint8_t preamble[] = { 0xB0, 0xEE, 0x40, 0x0A };
    check(uc_mem_map(uc, 0x10000000, 0x1000, UC_PROT_READ | UC_PROT_EXEC), "map preamble");
    check(uc_mem_write(uc, 0x10000000, preamble, sizeof(preamble)), "write preamble");
    const uint32_t sp = VFC_RAM_BASE + 0x8000;
    check(uc_reg_write(uc, UC_ARM_REG_SP, &sp), "write sp");
    check(uc_emu_start(uc, 0x10000001, 0xFFFFFFFFu, 0, 1), "preamble");

    for (int i = 0; i < NOPS; i++) {
        const int reads = ops[i].reads;
        const int nd = reads == DNM || reads == DM || reads == D ? NVALUES : 1;
        const int nn = reads == DNM ? NVALUES : 1;
        const int nm = reads == D ? 1 : NVALUES;
        const int modes = reads == M_ROUNDED ? 4 : 1;
        for (int d = 0; d < nd; d++) {
            for (int n = 0; n < nn; n++) {
                for (int m = 0; m < nm; m++) {
                    for (int mode = 0; mode < modes; mode++) {
                        const uint32_t in[3] = { values[d], values[n], values[m] };
                        run(i, in, (uint32_t)mode << 22);
                    }
                }
            }
        }
    }
    printf("%ld runs of %d instructions against Unicorn: %ld differ\n", runs, NOPS, differences);
    return differences != 0;
}
