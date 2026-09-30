// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Development only: runs the JIT's generated code in Unicorn's AArch64
// emulator, so that the JIT can be built, tested and fuzzed on hosts other
// than Apple silicon, such as Linux CI. Link it with the library built with
// -DVFC_JIT_EMULATED. Only the jump into generated code comes here; the rest,
// translation included, is the library's own code. Unicorn is GPL and is
// never linked into the library.
//
// Generated code sees host memory directly, at the same addresses, so the
// board, its buffers and the code cache need no copying. It is held to
// stricter rules than hardware holds it to:
//
// - every load and store it makes must fall inside memory it has a reason to
//   touch (the board, its RAM and flash, the translation table, its stack),
//   and anything else stops it with a fault naming the access;
// - a call into the C helpers leaves every register the AArch64 calling
//   convention lets a callee change, and the flags, holding junk.
//
//   cc -DVFC_JIT_EMULATED -ffp-contract=off -ISources/VFC/include -ISources/VFC
//      Sources/VFC/*.c tools/jitemu.c <tool>.c -lunicorn

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include <unicorn/unicorn.h>

#include "vfc_internal.h"

#define PAGE 4096u
#define STACK_BYTES (64u * 1024)
#define RETURN_ADDRESS 0x1000u          // enter() returns here; never mapped
#define JUNK 0x5A5A5A5A5A5A5A5Aull

static uc_engine *uc;
static uint8_t *stack;
static uint32_t *helperCode;            // one RET per helper, run after the hook
static void *helperFunction[2];

// Current run: what the generated code may touch.
typedef struct {
    uint64_t start, end;
    const char *what;
} range_t;
static range_t allowed[5];
static int nallowed;
static vfc_t *current;
static char violation[160];

// Page runs already mapped into Unicorn.
static range_t mapped[64];
static int nmapped;

static void fail(uc_err err, const char *what)
{
    if (err != UC_ERR_OK) {
        fprintf(stderr, "jitemu: %s: %s\n", what, uc_strerror(err));
        exit(1);
    }
}

// Maps host memory [start, end) into Unicorn at the same addresses, a page
// at a time, skipping pages already mapped.
static void map_host(uint64_t start, uint64_t end)
{
    start &= ~(uint64_t)(PAGE - 1);
    end = (end + PAGE - 1) & ~(uint64_t)(PAGE - 1);
    uint64_t page = start;
    while (page < end) {
        bool done = false;
        for (int i = 0; i < nmapped && !done; i++) {
            if (page >= mapped[i].start && page < mapped[i].end) {
                page = mapped[i].end;
                done = true;
            }
        }
        if (done) {
            continue;
        }
        uint64_t runEnd = page + PAGE;
        for (bool grew = true; grew && runEnd < end;) {
            grew = true;
            for (int i = 0; i < nmapped; i++) {
                if (runEnd >= mapped[i].start && runEnd < mapped[i].end) {
                    grew = false;
                }
            }
            if (grew) {
                runEnd += PAGE;
            }
        }
        if (nmapped == (int)(sizeof(mapped) / sizeof(mapped[0]))) {
            fprintf(stderr, "jitemu: too many mappings\n");
            exit(1);
        }
        fail(uc_mem_map_ptr(uc, page, runEnd - page, UC_PROT_ALL, (void *)(uintptr_t)page), "map");
        mapped[nmapped++] = (range_t){ page, runEnd, "" };
        page = runEnd;
    }
}

static void on_access(uc_engine *engine, uc_mem_type type, uint64_t address, int size, int64_t value, void *user)
{
    (void)value;
    (void)user;
    for (int i = 0; i < nallowed; i++) {
        if (address >= allowed[i].start && address + (uint64_t)size <= allowed[i].end) {
            return;
        }
    }
    if (!violation[0]) {
        uint64_t pc = 0;
        uc_reg_read(engine, UC_ARM64_REG_PC, &pc);
        snprintf(violation, sizeof(violation), "generated code %s %d bytes at %#llx, outside anything it may touch (host pc %#llx)",
                 type == UC_MEM_WRITE ? "wrote" : "read", size, (unsigned long long)address, (unsigned long long)pc);
    }
    uc_emu_stop(engine);
}

static bool on_unmapped(uc_engine *engine, uc_mem_type type, uint64_t address, int size, int64_t value, void *user)
{
    (void)value;
    (void)user;
    if (!violation[0]) {
        uint64_t pc = 0;
        uc_reg_read(engine, UC_ARM64_REG_PC, &pc);
        snprintf(violation, sizeof(violation), "generated code %s %d bytes at unmapped %#llx (host pc %#llx)",
                 type == UC_MEM_FETCH_UNMAPPED ? "jumped to" : type == UC_MEM_WRITE_UNMAPPED ? "wrote" : "read",
                 size, (unsigned long long)address, (unsigned long long)pc);
    }
    return false;
}

// A call to a C helper: made natively, with the arguments the generated code
// passed, before the RET at the helper's address returns to it.
static void on_helper(uc_engine *engine, uint64_t address, uint32_t size, void *user)
{
    (void)size;
    (void)user;
    uint64_t x[4];
    for (int i = 0; i < 4; i++) {
        uc_reg_read(engine, UC_ARM64_REG_X0 + i, &x[i]);
    }
    uint64_t result = JUNK;
    if (address == (uint64_t)(uintptr_t)&helperCode[0]) {
        uint32_t (*load)(vfc_t *, uint32_t, uint32_t) = (uint32_t (*)(vfc_t *, uint32_t, uint32_t))helperFunction[0];
        result = load((vfc_t *)(uintptr_t)x[0], (uint32_t)x[1], (uint32_t)x[2]);
    } else {
        void (*store)(vfc_t *, uint32_t, uint32_t, uint32_t) = (void (*)(vfc_t *, uint32_t, uint32_t, uint32_t))helperFunction[1];
        store((vfc_t *)(uintptr_t)x[0], (uint32_t)x[1], (uint32_t)x[2], (uint32_t)x[3]);
    }
    // What a real callee may leave behind: x0 is the result (or junk), and
    // x1-x17, the flags and the caller-saved vector registers are junk.
    uc_reg_write(engine, UC_ARM64_REG_X0, &result);
    const uint64_t junk = JUNK;
    for (int i = 1; i <= 17; i++) {
        uc_reg_write(engine, UC_ARM64_REG_X0 + i, &junk);
    }
    const uint64_t nzcv = 0xA0000000u;
    uc_reg_write(engine, UC_ARM64_REG_NZCV, &nzcv);
    const uint32_t junkBits = 0x7FC0DEADu;
    for (int i = 0; i < 32; i++) {
        if (i < 8 || i >= 16) {
            uc_reg_write(engine, UC_ARM64_REG_S0 + i, &junkBits);
        }
    }
}

static void init(void)
{
    if (uc) {
        return;
    }
    fail(uc_open(UC_ARCH_ARM64, UC_MODE_ARM, &uc), "open");
    stack = mmap(NULL, STACK_BYTES, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    helperCode = mmap(NULL, PAGE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (stack == MAP_FAILED || helperCode == MAP_FAILED) {
        fprintf(stderr, "jitemu: out of memory\n");
        exit(1);
    }
    helperCode[0] = helperCode[1] = 0xD65F03C0u;        // ret
    map_host((uintptr_t)stack, (uintptr_t)stack + STACK_BYTES);
    map_host((uintptr_t)helperCode, (uintptr_t)helperCode + PAGE);
    uc_hook hook;
    fail(uc_hook_add(uc, &hook, UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, (void *)on_access, NULL, 1, 0), "hook");
    fail(uc_hook_add(uc, &hook, UC_HOOK_MEM_UNMAPPED, (void *)on_unmapped, NULL, 1, 0), "hook");
    fail(uc_hook_add(uc, &hook, UC_HOOK_CODE, (void *)on_helper, NULL,
                     (uintptr_t)&helperCode[0], (uintptr_t)&helperCode[1]), "hook");
    // Floating point and SIMD usable at EL1 and EL0.
    const uint64_t cpacr = 3u << 20;
    fail(uc_reg_write(uc, UC_ARM64_REG_CPACR_EL1, &cpacr), "cpacr");
}

void *vfc_jit_emulated_helper(int which, void *function)
{
    init();
    helperFunction[which] = function;
    return &helperCode[which];
}

void vfc_jit_emulated_flush(void *code, size_t length)
{
    init();
    // The code cache was written behind Unicorn's back: drop what it translated.
    fail(uc_ctl_remove_cache(uc, (uintptr_t)code, (uintptr_t)code + length), "flush");
}

bool vfc_jit_emulated_enter(vfc_t *vfc, void *code, size_t codeBytes, void *enter, void *block)
{
    init();
    current = vfc;
    nallowed = 0;
    allowed[nallowed++] = (range_t){ (uintptr_t)vfc, (uintptr_t)vfc + sizeof(vfc_t), "board" };
    allowed[nallowed++] = (range_t){ (uintptr_t)vfc->ram, (uintptr_t)vfc->ram + VFC_RAM_SIZE, "RAM" };
    allowed[nallowed++] = (range_t){ (uintptr_t)vfc->flash, (uintptr_t)vfc->flash + VFC_FLASH_SIZE, "flash" };
    allowed[nallowed++] = (range_t){ (uintptr_t)vfc->jitTable, (uintptr_t)vfc->jitTable + VFC_FLASH_SIZE / 2 * sizeof(void *), "table" };
    allowed[nallowed++] = (range_t){ (uintptr_t)stack, (uintptr_t)stack + STACK_BYTES, "stack" };
    for (int i = 0; i < nallowed; i++) {
        map_host(allowed[i].start, allowed[i].end);
    }
    map_host((uintptr_t)code, (uintptr_t)code + codeBytes);

    const uint64_t x0 = (uintptr_t)vfc, x1 = (uintptr_t)block, sp = (uintptr_t)stack + STACK_BYTES, lr = RETURN_ADDRESS;
    uc_reg_write(uc, UC_ARM64_REG_X0, &x0);
    uc_reg_write(uc, UC_ARM64_REG_X1, &x1);
    uc_reg_write(uc, UC_ARM64_REG_SP, &sp);
    uc_reg_write(uc, UC_ARM64_REG_X30, &lr);
    violation[0] = 0;
    const uc_err err = uc_emu_start(uc, (uintptr_t)enter, RETURN_ADDRESS, 0, 0);
    uint64_t pc = 0;
    uc_reg_read(uc, UC_ARM64_REG_PC, &pc);
    if (!violation[0] && (err != UC_ERR_OK || pc != RETURN_ADDRESS)) {
        snprintf(violation, sizeof(violation), "generated code stopped at host pc %#llx: %s",
                 (unsigned long long)pc, err != UC_ERR_OK ? uc_strerror(err) : "did not return");
    }
    if (violation[0]) {
        vfc->stopRequested = false;
        vfc_raise_fault(vfc, "jitemu: %s", violation);
        return false;
    }
    return true;
}
