// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Differential test against Unicorn (QEMU's Cortex-M4), development only:
// Unicorn is GPL and is never linked into the library.
//
// Runs a firmware image in both emulators one instruction at a time. vfc
// drives the virtual board as usual; every mailbox access it makes is
// recorded, and Unicorn's mailbox replays them in order, so both see
// identical inputs. After every instruction the core and FPU registers are
// compared, and RAM every few thousand instructions.
//
//   cc -O2 -DVFC_TRACE -ffp-contract=off -ISources/VFC/include -ISources/VFC \
//      -I<unicorn>/include Sources/VFC/*.c tools/difftest.c <libunicorn.2.dylib> -o tools/difftest
//   tools/difftest firmware.bin [instructions] [--snapshot <file>]

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unicorn/unicorn.h>

#include "vfc.h"
#include "vfc_internal.h"

#define MAILBOX_LOG 4096

typedef struct {
    bool write;
    uint32_t offset, value;
} access_t;

static access_t accessLog[MAILBOX_LOG];
static int logCount, logNext;
static bool replayError;

void vfc_trace_mailbox(vfc_t *vfc, bool write, uint32_t offset, uint32_t value)
{
    (void)vfc;
    if (logCount < MAILBOX_LOG) {
        accessLog[logCount++] = (access_t){ write, offset, value };
    }
}

static uint64_t mailbox_read_cb(uc_engine *uc, uint64_t offset, unsigned size, void *user)
{
    (void)uc;
    (void)user;
    if (logNext >= logCount || accessLog[logNext].write || accessLog[logNext].offset != (offset & ~3u)) {
        fprintf(stderr, "replay: unicorn read mailbox 0x%03llx (size %u), vfc did %s 0x%03x\n",
                (unsigned long long)offset, size,
                logNext < logCount ? (accessLog[logNext].write ? "write" : "read") : "nothing",
                logNext < logCount ? accessLog[logNext].offset : 0);
        replayError = true;
        return 0;
    }
    const uint32_t value = accessLog[logNext++].value;
    return (value >> (8 * (offset & 3))) & (size == 4 ? 0xFFFFFFFFu : size == 2 ? 0xFFFF : 0xFF);
}

static void mailbox_write_cb(uc_engine *uc, uint64_t offset, unsigned size, uint64_t value, void *user)
{
    (void)uc;
    (void)user;
    (void)size;
    if (logNext >= logCount || !accessLog[logNext].write || accessLog[logNext].offset != (offset & ~3u)
        || accessLog[logNext].value != (uint32_t)value) {
        fprintf(stderr, "replay: unicorn wrote 0x%08x to mailbox 0x%03llx, vfc did %s 0x%03x = 0x%08x\n",
                (uint32_t)value, (unsigned long long)offset,
                logNext < logCount ? (accessLog[logNext].write ? "write" : "read") : "nothing",
                logNext < logCount ? accessLog[logNext].offset : 0,
                logNext < logCount ? accessLog[logNext].value : 0);
        replayError = true;
        return;
    }
    logNext++;
}

static void check(uc_err err, const char *what)
{
    if (err != UC_ERR_OK) {
        fprintf(stderr, "%s: %s\n", what, uc_strerror(err));
        exit(1);
    }
}

typedef struct {
    uint32_t r[16];
    uint32_t nzcvq, ge;
    uint32_t s[32];
    uint32_t fpscr;
    uint32_t basepri, primask;
} state_t;

static void vfc_state(const vfc_t *vfc, state_t *out)
{
    const vfc_cpu_t *cpu = &vfc->cpu;
    memcpy(out->r, cpu->r, sizeof(out->r));
    out->nzcvq = ((uint32_t)cpu->n << 31) | ((uint32_t)cpu->z << 30) | ((uint32_t)cpu->c << 29)
               | ((uint32_t)cpu->v << 28) | ((uint32_t)cpu->q << 27);
    out->ge = cpu->ge;
    memcpy(out->s, cpu->s.u, sizeof(out->s));
    out->fpscr = cpu->fpscr;
    out->basepri = cpu->basepri;
    out->primask = cpu->primask;
}

// Unicorn's IT state: ITSTATE[1:0] is XPSR[26:25], ITSTATE[7:2] XPSR[15:10].
static uint32_t uc_itstate(uc_engine *uc)
{
    uint32_t xpsr = 0;
    uc_reg_read(uc, UC_ARM_REG_XPSR, &xpsr);
    return ((xpsr >> 25) & 3) | ((xpsr >> 8) & 0xFC);
}

static void uc_state(uc_engine *uc, state_t *out)
{
    static int ids[16 + 32];
    static void *ptrs[16 + 32];
    static bool ready;
    if (!ready) {
        for (int i = 0; i < 13; i++) ids[i] = UC_ARM_REG_R0 + i;
        ids[13] = UC_ARM_REG_SP;
        ids[14] = UC_ARM_REG_LR;
        ids[15] = UC_ARM_REG_PC;
        for (int i = 0; i < 32; i++) ids[16 + i] = UC_ARM_REG_S0 + i;
        ready = true;
    }
    for (int i = 0; i < 16; i++) ptrs[i] = &out->r[i];
    for (int i = 0; i < 32; i++) ptrs[16 + i] = &out->s[i];
    uc_reg_read_batch(uc, ids, ptrs, 48);
    uint32_t xpsr = 0, fpscr = 0, basepri = 0, primask = 0;
    uc_reg_read(uc, UC_ARM_REG_XPSR, &xpsr);
    uc_reg_read(uc, UC_ARM_REG_FPSCR, &fpscr);
    uc_reg_read(uc, UC_ARM_REG_BASEPRI, &basepri);
    uc_reg_read(uc, UC_ARM_REG_PRIMASK, &primask);
    out->nzcvq = xpsr & 0xF8000000u;
    out->ge = (xpsr >> 16) & 0xF;
    out->fpscr = fpscr;
    out->basepri = basepri;
    out->primask = primask;
}

static int compare(const state_t *a, const state_t *b, char *out, size_t size)
{
    int differences = 0;
    size_t n = 0;
    for (int i = 0; i < 16; i++) {
        if (a->r[i] != b->r[i]) {
            n += (size_t)snprintf(out + n, size - n, "  r%d vfc %08x unicorn %08x\n", i, a->r[i], b->r[i]);
            differences++;
        }
    }
    if (a->nzcvq != b->nzcvq) {
        n += (size_t)snprintf(out + n, size - n, "  NZCVQ vfc %x unicorn %x\n", a->nzcvq >> 27, b->nzcvq >> 27);
        differences++;
    }
    if (a->ge != b->ge) {
        n += (size_t)snprintf(out + n, size - n, "  GE vfc %x unicorn %x\n", a->ge, b->ge);
        differences++;
    }
    for (int i = 0; i < 32; i++) {
        if (a->s[i] != b->s[i]) {                   // NaNs too: both follow Arm's rules
            float fa, fb;
            memcpy(&fa, &a->s[i], 4);
            memcpy(&fb, &b->s[i], 4);
            n += (size_t)snprintf(out + n, size - n, "  s%d vfc %08x (%g) unicorn %08x (%g)\n", i, a->s[i], fa, b->s[i], fb);
            differences++;
        }
    }
    if ((a->fpscr & 0xF0000000u) != (b->fpscr & 0xF0000000u)) {
        n += (size_t)snprintf(out + n, size - n, "  FPSCR.NZCV vfc %x unicorn %x\n", a->fpscr >> 28, b->fpscr >> 28);
        differences++;
    }
    if (a->basepri != b->basepri || a->primask != b->primask) {
        n += (size_t)snprintf(out + n, size - n, "  BASEPRI/PRIMASK vfc %x/%x unicorn %x/%x\n",
                              a->basepri, a->primask, b->basepri, b->primask);
        differences++;
    }
    return differences;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s firmware.bin [instructions]\n", argv[0]);
        return 2;
    }
    uint64_t limit = 5000000;
    const char *snapshotPath = NULL;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--snapshot") && i + 1 < argc) {
            snapshotPath = argv[++i];
        } else {
            limit = strtoull(argv[i], NULL, 10);
        }
    }

    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        perror(argv[1]);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    const long length = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *image = malloc((size_t)length);
    if (fread(image, 1, (size_t)length, f) != (size_t)length) {
        return 1;
    }
    fclose(f);

    vfc_t *vfc = vfc_create();
    vfc_set_jit(vfc, false);            // the interpreter is what's checked here; jitdiff checks the JIT against it
    if (vfc_load(vfc, image, (size_t)length, VFC_FLASH_BASE) != VFC_OK) {
        fprintf(stderr, "vfc load failed\n");
        return 1;
    }

    uc_engine *uc;
    check(uc_open(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS, &uc), "uc_open");
    check(uc_ctl_set_cpu_model(uc, UC_CPU_ARM_CORTEX_M4), "cpu model");
    check(uc_mem_map(uc, VFC_FLASH_BASE, VFC_FLASH_SIZE, UC_PROT_READ | UC_PROT_EXEC), "map flash");
    check(uc_mem_write(uc, VFC_FLASH_BASE, image, (size_t)length), "write flash");
    check(uc_mem_map(uc, VFC_RAM_BASE, VFC_RAM_SIZE, UC_PROT_ALL), "map ram");
    check(uc_mem_map(uc, VFC_SCS_BASE, VFC_SCS_SIZE, UC_PROT_READ | UC_PROT_WRITE), "map scs");
    check(uc_mmio_map(uc, VFC_MAILBOX_BASE, VFC_MAILBOX_SIZE, mailbox_read_cb, NULL, mailbox_write_cb, NULL), "map mailbox");

    uint32_t sp = vfc->cpu.r[13], pc = vfc->cpu.r[15] | 1;
    check(uc_reg_write(uc, UC_ARM_REG_SP, &sp), "sp");
    // Architectural reset values, which Unicorn leaves at zero.
    uint32_t lr = 0xFFFFFFFFu, xpsr = 0x01000000u;
    check(uc_reg_write(uc, UC_ARM_REG_LR, &lr), "lr");
    check(uc_reg_write(uc, UC_ARM_REG_XPSR, &xpsr), "xpsr");

    // Or start from a snapshot, such as an armed quad in flight
    // (setpoint-cli vfc <log> --firmware <bin> --snapshot <file>).
    bool armed = false;
    if (snapshotPath) {
        FILE *s = fopen(snapshotPath, "rb");
        if (!s) {
            perror(snapshotPath);
            return 1;
        }
        fseek(s, 0, SEEK_END);
        const long snapshotLength = ftell(s);
        fseek(s, 0, SEEK_SET);
        uint8_t *snapshot = malloc((size_t)snapshotLength);
        if (fread(snapshot, 1, (size_t)snapshotLength, s) != (size_t)snapshotLength
            || vfc_restore(vfc, snapshot, (size_t)snapshotLength) != VFC_OK) {
            fprintf(stderr, "snapshot does not fit this image and build\n");
            return 1;
        }
        fclose(s);
        free(snapshot);
        armed = true;

        // Let Unicorn open its FP context itself with one throwaway
        // instruction (vmov.f32 s0, s0), as the firmware did long ago. Setting
        // CONTROL.FPCA by register write alone leaves QEMU's cached state
        // saying a fresh context is due, which would reset FPSCR at the
        // firmware's next floating point instruction.
        const uint8_t preamble[] = { 0xB0, 0xEE, 0x40, 0x0A };
        check(uc_mem_map(uc, 0x10000000, 0x1000, UC_PROT_READ | UC_PROT_EXEC), "map preamble");
        check(uc_mem_write(uc, 0x10000000, preamble, sizeof(preamble)), "write preamble");
        check(uc_emu_start(uc, 0x10000001, 0xFFFFFFFFu, 0, 1), "run preamble");

        const vfc_cpu_t *cpu = &vfc->cpu;
        check(uc_mem_write(uc, VFC_RAM_BASE, vfc->ram, VFC_RAM_SIZE), "ram");
        check(uc_mem_write(uc, VFC_SCS_BASE, vfc->scs, VFC_SCS_SIZE), "scs");
        for (int i = 0; i < 13; i++) {
            check(uc_reg_write(uc, UC_ARM_REG_R0 + i, &cpu->r[i]), "r");
        }
        check(uc_reg_write(uc, UC_ARM_REG_SP, &cpu->r[13]), "sp");
        check(uc_reg_write(uc, UC_ARM_REG_LR, &cpu->r[14]), "lr");
        for (int i = 0; i < 32; i++) {
            check(uc_reg_write(uc, UC_ARM_REG_S0 + i, &cpu->s.u[i]), "s");
        }
        check(uc_reg_write(uc, UC_ARM_REG_FPSCR, &cpu->fpscr), "fpscr");
        check(uc_reg_write(uc, UC_ARM_REG_BASEPRI, &cpu->basepri), "basepri");
        check(uc_reg_write(uc, UC_ARM_REG_PRIMASK, &cpu->primask), "primask");
        xpsr = ((uint32_t)cpu->n << 31) | ((uint32_t)cpu->z << 30) | ((uint32_t)cpu->c << 29)
             | ((uint32_t)cpu->v << 28) | ((uint32_t)cpu->q << 27) | ((uint32_t)cpu->ge << 16)
             | 0x01000000u | ((uint32_t)(cpu->itstate & 3) << 25) | ((uint32_t)(cpu->itstate >> 2) << 10);
        check(uc_reg_write(uc, UC_ARM_REG_XPSR, &xpsr), "xpsr");
        pc = cpu->r[15] | 1;
        uint32_t readControl = 0, readFpscr = 0;
        uc_reg_read(uc, UC_ARM_REG_CONTROL, &readControl);
        uc_reg_read(uc, UC_ARM_REG_FPSCR, &readFpscr);
        printf("from snapshot: pc %08x, %.3f s virtual (unicorn CONTROL %x FPSCR %08x, vfc FPSCR %08x)\n",
               cpu->r[15], vfc_time_ns(vfc) * 1e-9, readControl, readFpscr, cpu->fpscr);
    }

    const int16_t acc[3] = { 0, 0, 2048 };
    state_t a, b;
    char report[8192];
    uint64_t steps = 0, wakes = 0;
    uint32_t lastPcs[16] = { 0 };

    while (steps < limit) {
        logCount = logNext = 0;
        const uint32_t instPc = vfc->cpu.r[15];
        const uint16_t hw1 = (uint16_t)(image[instPc - VFC_FLASH_BASE] | (image[instPc - VFC_FLASH_BASE + 1] << 8));
        const bool isWfi = hw1 == 0xBF30;
        vfc_stop_t stop = vfc_run(vfc, 1);
        // An IT block is one step, in both.
        while ((vfc->cpu.itstate & 0xF) && stop == VFC_STOP_BUDGET) {
            stop = vfc_run(vfc, 1);
        }
        if (stop == VFC_STOP_FAULT || stop == VFC_STOP_RESET) {
            printf("vfc stopped (%d) at step %llu: %s\n", stop, (unsigned long long)steps, vfc_fault(vfc) ? vfc_fault(vfc) : "");
            break;
        }

        uc_err err = uc_emu_start(uc, pc, 0xFFFFFFFFu, 0, 1);
        // Unicorn usually runs a whole IT block as one step, as vfc just did,
        // but it can stop inside one (before a store, say): finish it.
        for (int i = 0; i < 4 && err == UC_ERR_OK && uc_itstate(uc); i++) {
            uint32_t at;
            uc_reg_read(uc, UC_ARM_REG_PC, &at);
            err = uc_emu_start(uc, at | 1, 0xFFFFFFFFu, 0, 1);
        }
        if (err != UC_ERR_OK) {
            printf("unicorn: %s at step %llu, pc %08x\n", uc_strerror(err), (unsigned long long)steps, pc & ~1u);
            break;
        }
        uint32_t ucPc;
        uc_reg_read(uc, UC_ARM_REG_PC, &ucPc);
        if (isWfi && ucPc == instPc) {
            ucPc += 2;                      // Unicorn halts on WFI without advancing
            uc_reg_write(uc, UC_ARM_REG_PC, &ucPc);
        }
        pc = ucPc | 1;
        steps++;
        lastPcs[steps % 16] = instPc;

        vfc_state(vfc, &a);
        uc_state(uc, &b);
        const int differences = compare(&a, &b, report, sizeof(report));
        if (differences || replayError || logNext != logCount) {
            printf("DIVERGED at step %llu, instruction at %08x (%04x", (unsigned long long)steps, instPc, hw1);
            if ((hw1 >> 11) >= 0x1D) {
                printf(" %04x", image[instPc - VFC_FLASH_BASE + 2] | (image[instPc - VFC_FLASH_BASE + 3] << 8));
            }
            printf(")\n%s", report);
            if (logNext != logCount) {
                printf("  unicorn made %d of vfc's %d mailbox accesses\n", logNext, logCount);
            }
            printf("  previous pcs:");
            for (int i = 1; i <= 15; i++) {
                printf(" %08x", lastPcs[(steps + i) % 16]);
            }
            printf("\n");
            return 1;
        }

        if (steps % 4096 == 0) {
            static uint8_t ucRam[VFC_RAM_SIZE];
            uc_mem_read(uc, VFC_RAM_BASE, ucRam, VFC_RAM_SIZE);
            for (uint32_t i = 0; i < VFC_RAM_SIZE; i++) {
                if (ucRam[i] != vfc->ram[i]) {
                    printf("RAM DIVERGED at step %llu: 0x%08x vfc %02x unicorn %02x\n",
                           (unsigned long long)steps, VFC_RAM_BASE + i, vfc->ram[i], ucRam[i]);
                    return 1;
                }
            }
        }

        if (stop == VFC_STOP_IDLE) {
            // The host's side of lockstep: a moving gyro sample, then wake.
            const double t = vfc_time_ns(vfc) * 1e-9;
            const int16_t gyro[3] = {
                (int16_t)(1640.0 * sin(2 * M_PI * 3.0 * t)),
                (int16_t)(820.0 * sin(2 * M_PI * 7.0 * t + 1.0)),
                (int16_t)(410.0 * sin(2 * M_PI * 1.3 * t + 2.0)),
            };
            vfc_post_sensor(vfc, gyro, acc);
            if (wakes % 32 == 0) {
                uint16_t rc[8] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000 };
                rc[0] = (uint16_t)(1500 + 400 * sin(2 * M_PI * 0.7 * t));
                rc[1] = (uint16_t)(1500 + 300 * sin(2 * M_PI * 1.1 * t));
                if (armed) {
                    // Keep the arm switch on and fly: throttle moving through
                    // the middle, and motor speed for the RPM filter.
                    rc[2] = (uint16_t)(1450 + 250 * sin(2 * M_PI * 0.4 * t));
                    rc[4] = 2000;
                    for (int m = 0; m < 4; m++) {
                        vfc_set_erpm100(vfc, m, (uint32_t)(250 + 100 * sin(2 * M_PI * (0.4 * t + 0.1 * m))));
                    }
                }
                vfc_post_rc(vfc, rc, 8);
            }
            vfc_set_time_ns(vfc, vfc_wake_time_ns(vfc));
            wakes++;
            static bool wasLogging = true;
            if (armed && wasLogging && !vfc_blackbox_logging(vfc)) {
                printf("disarmed at %.3f s virtual, step %llu\n", t, (unsigned long long)steps);
                wasLogging = false;
            }
        }
    }
    printf("%llu instructions matched (%llu wakes, %.3f s virtual)\n",
           (unsigned long long)steps, (unsigned long long)wakes, vfc_time_ns(vfc) * 1e-9);
    if (armed) {
        printf("still armed and logging: %s; motors %.0f %.0f %.0f %.0f\n",
               vfc_blackbox_logging(vfc) ? "yes" : "NO",
               vfc_motor(vfc, 0), vfc_motor(vfc, 1), vfc_motor(vfc, 2), vfc_motor(vfc, 3));
    }
    uc_close(uc);
    return 0;
}
