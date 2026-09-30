// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Regression tests for bugs found by verification, each on the interpreter
// and, where there is one (natively or emulated), the JIT. Meant to be built
// with -fsanitize=address,undefined, so that a bug that corrupts memory or
// relies on undefined behaviour fails here even when its results look right.
//
//   make test

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vfc.h"
#include "vfc_internal.h"

static int failures, checks;

static void check(bool ok, const char *format, ...)
{
    checks++;
    if (ok) {
        return;
    }
    failures++;
    va_list args;
    va_start(args, format);
    fputs("FAIL ", stdout);
    vprintf(format, args);
    fputc('\n', stdout);
    va_end(args);
}

// --- Images: vectors, board info, a version string, code at 0x40

#define CODE (VFC_FLASH_BASE + 0x40)

static void put32(uint8_t *p, uint32_t v)
{
    memcpy(p, &v, 4);
}

static size_t image(uint8_t *img, const uint16_t *code, int halfwords)
{
    memset(img, 0, 256);
    put32(img + 0x00, VFC_RAM_BASE + VFC_RAM_SIZE);
    put32(img + 0x04, CODE + 1);
    put32(img + 0x1C, VFC_FLASH_BASE + 0x20);
    put32(img + 0x20, 0x56464331u);
    put32(img + 0x24, VFC_ABI);
    put32(img + 0x28, VFC_MAILBOX_BASE);
    put32(img + 0x2C, VFC_RAM_BASE + 0x70000);
    put32(img + 0x30, 0x1000);
    put32(img + 0x34, VFC_FLASH_BASE + 0x38);
    memcpy(img + 0x38, "regress", 8);
    memcpy(img + 0x40, code, 2u * (unsigned)halfwords);
    return 256;
}

static vfc_t *board(const uint16_t *code, int halfwords, bool jit)
{
    static uint8_t img[256];
    const size_t length = image(img, code, halfwords);
    vfc_t *v = vfc_create();
    vfc_set_jit(v, jit);
    if (vfc_load(v, img, length, VFC_FLASH_BASE) != VFC_OK) {
        puts("FAIL could not load a test image");
        exit(1);
    }
    return v;
}

// Engines to run each program on: the interpreter, and the JIT if any.
static int engines(void)
{
    return vfc_jit_available() ? 2 : 1;
}

static const char *engine_name(int e)
{
    return e ? "JIT" : "interpreter";
}

// --- The loader

static void test_loader(void)
{
    static const uint16_t nop[] = { 0xBF00, 0xBF30 };
    uint8_t img[256];
    vfc_t *v = vfc_create();
    image(img, nop, 2);
    check(vfc_load(v, img, sizeof(img), VFC_FLASH_BASE) == VFC_OK, "loader: a good image loads");
    check(!strcmp(vfc_firmware_version(v), "regress"), "loader: version string");

    // A board-info pointer anywhere just below flash used to wrap the bounds
    // check and read ~4 GB past the image.
    for (uint32_t info = 0x07FFFFE0u; info < VFC_FLASH_BASE; info++) {
        image(img, nop, 2);
        put32(img + 0x1C, info);
        check(vfc_load(v, img, sizeof(img), VFC_FLASH_BASE) == VFC_ERR_IMAGE, "loader: info at %08x", info);
    }
    // Board info must fit in the image, with nothing past its end.
    image(img, nop, 2);
    put32(img + 0x1C, VFC_FLASH_BASE + sizeof(img) - 23);
    check(vfc_load(v, img, sizeof(img), VFC_FLASH_BASE) == VFC_ERR_IMAGE, "loader: info running off the end");

    // A config region whose size wrapped the check used to be accepted, and
    // vfc_reset then copied ~4 GB.
    static const uint32_t regions[][2] = {
        { VFC_RAM_BASE + 0x100, 0xFFFFFF00u }, { VFC_RAM_BASE, 0 }, { VFC_RAM_BASE + VFC_RAM_SIZE, 4 },
        { VFC_RAM_BASE + VFC_RAM_SIZE - 4, 5 }, { VFC_RAM_BASE - 4, 8 }, { 0xFFFFFFFCu, 8 },
    };
    const uint32_t pcBefore = vfc_pc(v), usedBefore = v->flashUsed;
    for (size_t i = 0; i < sizeof(regions) / sizeof(regions[0]); i++) {
        image(img, nop, 2);
        put32(img + 0x2C, regions[i][0]);
        put32(img + 0x30, regions[i][1]);
        check(vfc_load(v, img, 200, VFC_FLASH_BASE) == VFC_ERR_IMAGE, "loader: config region %08x+%x", regions[i][0], regions[i][1]);
    }
    check(vfc_pc(v) == pcBefore && v->flashUsed == usedBefore, "loader: a refused image leaves the board as it was");
    image(img, nop, 2);
    put32(img + 0x2C, VFC_RAM_BASE + VFC_RAM_SIZE - 4);
    put32(img + 0x30, 4);
    check(vfc_load(v, img, sizeof(img), VFC_FLASH_BASE) == VFC_OK, "loader: config region at the very end of RAM");

    // A version string with no NUL before the end of the image stops there.
    image(img, nop, 2);
    put32(img + 0x34, VFC_FLASH_BASE + sizeof(img) - 3);
    memcpy(img + sizeof(img) - 3, "xyz", 3);
    check(vfc_load(v, img, sizeof(img), VFC_FLASH_BASE) == VFC_OK && !strcmp(vfc_firmware_version(v), "xyz"),
          "loader: version string cut at the end of the image");
    vfc_destroy(v);
}

// --- Snapshots

static void test_restore(void)
{
    static const uint16_t nop[] = { 0xBF00, 0xBF30 };
    vfc_t *v = board(nop, 2, false);
    const size_t n = vfc_snapshot_size(v);
    uint8_t *good = malloc(n), *bad = malloc(n);
    vfc_snapshot(v, good);
    check(vfc_restore(v, good, n) == VFC_OK, "restore: a good snapshot");

    const size_t state = 16;                     // the header, then the board
    const struct { const char *what; size_t offset; uint32_t value; int bytes; } cases[] = {
        { "toGuest.head", offsetof(vfc_t, toGuest.head), VFC_SERIAL_CAPACITY, 4 },
        { "toGuest.tail", offsetof(vfc_t, toGuest.tail), 0x80000000u, 4 },
        { "fromGuest.head", offsetof(vfc_t, fromGuest.head), VFC_SERIAL_CAPACITY + 1, 4 },
        { "fromGuest.tail", offsetof(vfc_t, fromGuest.tail), 0xFFFFFFFFu, 4 },
        { "consoleLength", offsetof(vfc_t, consoleLength), VFC_CONSOLE_CAPACITY + 1, 4 },
        { "stopReason", offsetof(vfc_t, stopReason), 7, 4 },
        { "cpu.n", offsetof(vfc_t, cpu.n), 2, 1 },
        { "stopRequested", offsetof(vfc_t, stopRequested), 0xFF, 1 },
        { "blackboxOpen", offsetof(vfc_t, blackboxOpen), 3, 1 },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        memcpy(bad, good, n);
        memcpy(bad + state + cases[i].offset, &cases[i].value, (size_t)cases[i].bytes);
        check(vfc_restore(v, bad, n) == VFC_ERR_IMAGE, "restore: refuses %s = %#x", cases[i].what, cases[i].value);
    }
    memcpy(bad, good, n);
    memset(bad + state + offsetof(vfc_t, fault), 'x', sizeof(v->fault));
    check(vfc_restore(v, bad, n) == VFC_ERR_IMAGE, "restore: refuses an unterminated fault string");

    // What the board read from its image stays its own.
    memcpy(bad, good, n);
    const uint32_t hugeUsed = 0xFFFFFFFFu, zeroClock = 0;
    memcpy(bad + state + offsetof(vfc_t, flashUsed), &hugeUsed, 4);
    memcpy(bad + state + offsetof(vfc_t, clockHz), &zeroClock, 4);
    check(vfc_restore(v, bad, n) == VFC_OK && v->flashUsed != hugeUsed && vfc_clock_hz(v) != 0,
          "restore: keeps its own flashUsed and clock");
    vfc_serial_write(v, (const uint8_t *)"ok", 2);
    char out[8];
    check(vfc_console_read(v, out, sizeof(out)) == 0, "restore: console empty after restore");
    free(good);
    free(bad);
    vfc_destroy(v);
}

// --- Instructions

typedef struct {
    uint32_t r[16];
    uint32_t s[32];
} regs_t;

// Runs `code` (then WFI) from `in`, on one engine. Returns the stop reason.
static vfc_stop_t run(const uint16_t *code, int halfwords, const regs_t *in, bool jit, vfc_t **out)
{
    uint16_t program[16];
    memcpy(program, code, 2u * (unsigned)halfwords);
    program[halfwords] = 0xBF30;
    vfc_t *v = board(program, halfwords + 1, jit);
    memcpy(v->cpu.r, in->r, 15 * 4);
    memcpy(v->cpu.s.u, in->s, sizeof(in->s));
    const vfc_stop_t stop = vfc_run(v, 64);
    *out = v;
    return stop;
}

// The most significant word multiplies, against 128-bit arithmetic.
static void test_smmla(void)
{
    static const uint32_t values[] = { 0, 1, 0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, 0x12345678, 0x80000001, 0xDEADBEEF };
    static const struct { uint16_t hw1, hw2; bool subtract, round, accumulate; const char *name; } forms[] = {
        { 0xFB51, 0x3002, false, false, true, "SMMLA" },  { 0xFB51, 0x3012, false, true, true, "SMMLAR" },
        { 0xFB61, 0x3002, true, false, true, "SMMLS" },   { 0xFB61, 0x3012, true, true, true, "SMMLSR" },
        { 0xFB51, 0xF002, false, false, false, "SMMUL" }, { 0xFB51, 0xF012, false, true, false, "SMMULR" },
    };
    for (int e = 0; e < engines(); e++) {
        for (size_t f = 0; f < sizeof(forms) / sizeof(forms[0]); f++) {
            for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
                for (size_t j = 0; j < sizeof(values) / sizeof(values[0]); j++) {
                    regs_t in = { 0 };
                    in.r[1] = values[i];
                    in.r[2] = values[j];
                    in.r[3] = values[(i + j) % (sizeof(values) / sizeof(values[0]))];
                    const uint16_t code[] = { forms[f].hw1, forms[f].hw2 };
                    vfc_t *v;
                    run(code, 2, &in, e, &v);
                    __int128 product = (__int128)(int32_t)in.r[1] * (int32_t)in.r[2];
                    __int128 result = forms[f].accumulate ? (__int128)(int32_t)in.r[3] * ((__int128)1 << 32) : 0;
                    result = forms[f].subtract ? result - product : result + product;
                    if (forms[f].round) result += 0x80000000;
                    const uint32_t expected = (uint32_t)((unsigned __int128)result >> 32);
                    check(v->cpu.r[0] == expected, "%s %s %08x %08x %08x: %08x, expected %08x", engine_name(e),
                          forms[f].name, in.r[1], in.r[2], in.r[3], v->cpu.r[0], expected);
                    vfc_destroy(v);
                }
            }
        }
    }
}

// VLDR and VSTR of D16-D31 used to run off the register file into the
// board's pointers.
static void test_vldr_d16(void)
{
    static const uint16_t forms[][2] = { { 0xEDD0, 0x0B00 }, { 0xEDC0, 0x0B00 }, { 0xED55, 0xDB9D } };
    for (int e = 0; e < engines(); e++) {
        for (int f = 0; f < 3; f++) {
            regs_t in = { 0 };
            in.r[0] = in.r[5] = VFC_RAM_BASE + 0x1000;
            vfc_t *v;
            const vfc_stop_t stop = run(forms[f], 2, &in, e, &v);
            const uint8_t *ram = v->ram, *scs = v->scs;
            check(stop == VFC_STOP_FAULT && vfc_fault(v) && strstr(vfc_fault(v), "UNDEFINSTR"),
                  "%s: %04x %04x (D16-D31) is UNDEFINED: %s", engine_name(e), forms[f][0], forms[f][1], vfc_fault(v));
            vfc_reset(v);                                   // touches RAM and SCS through the pointers
            check(v->ram == ram && v->scs == scs, "%s: %04x %04x left the board intact", engine_name(e), forms[f][0], forms[f][1]);
            vfc_destroy(v);
        }
    }
}

// Accesses that straddle the end of RAM fault; ones inside it don't. Fault
// reports name the faulting instruction, or the address a fetch faulted at.
static void test_ram_edge_and_fault_pc(void)
{
    static const struct { uint16_t hw; uint32_t address; bool faults; const char *name; } cases[] = {
        { 0x6008, 0x2007FFFD, true, "str 4 bytes, 3 in RAM" },
        { 0x6008, 0x2007FFFF, true, "str 4 bytes, 1 in RAM" },
        { 0x8008, 0x2007FFFF, true, "strh 2 bytes, 1 in RAM" },
        { 0x6808, 0x2007FFFE, true, "ldr 4 bytes, 2 in RAM" },
        { 0x8808, 0x2007FFFF, true, "ldrh 2 bytes, 1 in RAM" },
        { 0x6008, 0x2007FFFC, false, "str, the last word" },
        { 0x7008, 0x2007FFFF, false, "strb, the last byte" },
        { 0x6808, 0x1FFFFFFE, true, "ldr from just below RAM" },
        { 0x6808, 0x081FFFFE, true, "ldr straddling the end of flash" },
        { 0x6808, 0x081FFFFC, false, "ldr, the last word of flash" },
    };
    for (int e = 0; e < engines(); e++) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            regs_t in = { 0 };
            in.r[1] = cases[i].address;
            vfc_t *v;
            const vfc_stop_t stop = run(&cases[i].hw, 1, &in, e, &v);
            char expected[96];
            snprintf(expected, sizeof(expected), "at 0x%08x (pc 0x%08x)", cases[i].address, CODE);
            if (cases[i].faults) {
                check(stop == VFC_STOP_FAULT && vfc_fault(v) && strstr(vfc_fault(v), expected) && vfc_pc(v) == CODE,
                      "%s: %s: expected a fault %s, got %d: %s", engine_name(e), cases[i].name, expected, stop, vfc_fault(v));
            } else {
                check(stop == VFC_STOP_IDLE, "%s: %s: expected no fault, got %s", engine_name(e), cases[i].name, vfc_fault(v));
            }
            vfc_destroy(v);
        }
        // bx r0 to unmapped memory: the fetch there faults, and names it.
        static const uint16_t bx[] = { 0x4700 };
        regs_t in = { 0 };
        in.r[0] = 0x10000001;
        vfc_t *v;
        run(bx, 1, &in, e, &v);
        check(vfc_fault(v) && strstr(vfc_fault(v), "(pc 0x10000000)"), "%s: fetch fault names the fetch: %s", engine_name(e), vfc_fault(v));
        vfc_destroy(v);
    }
}

// --- Floating point NaNs, as Arm's pseudocode gives them

#define ONE 0x3F800000u
#define TWO 0x40000000u
#define INF 0x7F800000u
#define DEFAULT_NAN 0x7FC00000u

static void test_nans(void)
{
    // s0 = d, s1 = n, s2 = m: <op> s0, s1, s2 (VSQRT: s0, s2).
    static const struct { const char *name; uint16_t hw1, hw2; uint32_t d, n, m, expected; } cases[] = {
        { "VMUL 0 x inf: the default NaN", 0xEE20, 0x0A81, 0, 0, INF, DEFAULT_NAN },
        { "VADD with a signalling NaN: quieted", 0xEE30, 0x0A81, 0, ONE, 0x7F800001, 0x7FC00001 },
        { "VADD quiet, signalling: the signalling one", 0xEE30, 0x0A81, 0, 0x7FC00123, 0xFF800456, 0xFFC00456 },
        { "VADD quiet, quiet: the first", 0xEE30, 0x0A81, 0, 0x7FC00001, 0xFFC00002, 0x7FC00001 },
        { "VSUB 1 - NaN: the NaN, not negated", 0xEE30, 0x0AC1, 0, ONE, 0x7FC0000D, 0x7FC0000D },
        { "VMLS, NaN product: negated", 0xEE00, 0x0AC1, ONE, 0, INF, 0xFFC00000 },
        { "VMLA, NaN product", 0xEE00, 0x0A81, ONE, 0, INF, DEFAULT_NAN },
        { "VNMLS, NaN accumulator: negated", 0xEE10, 0x0A81, 0x7FC00005, ONE, ONE, 0xFFC00005 },
        { "VNMLA, NaN product: negated", 0xEE10, 0x0AC1, TWO, 0x7FC00007, ONE, 0xFFC00007 },
        { "VNMUL, signalling NaN: quieted, negated", 0xEE20, 0x0AC1, 0, 0x7F800009, ONE, 0xFFC00009 },
        { "VFMA, quiet NaN addend, 0 x inf: the default NaN", 0xEEA0, 0x0A81, 0x7FC0000A, 0, INF, DEFAULT_NAN },
        { "VFMA, signalling NaN addend, 0 x inf: quieted addend", 0xEEA0, 0x0A81, 0x7F80000B, 0, INF, 0x7FC0000B },
        { "VFMS, NaN n: negated", 0xEEA0, 0x0AC1, ONE, 0x7FC0000C, ONE, 0xFFC0000C },
        { "VFNMA, NaN d: negated", 0xEE90, 0x0AC1, 0x7FC0000E, ONE, ONE, 0xFFC0000E },
        { "VFNMS, NaN m", 0xEE90, 0x0A81, ONE, ONE, 0xFFC0000F, 0xFFC0000F },
        { "VDIV 0 / 0: the default NaN", 0xEE80, 0x0A81, 0, 0, 0, DEFAULT_NAN },
        { "VDIV 1 / 0: infinity", 0xEE80, 0x0A81, 0, ONE, 0, INF },
        { "VSQRT -1: the default NaN", 0xEEB1, 0x0AC1, 0, 0, 0xBF800000, DEFAULT_NAN },
        { "VSQRT -0: -0", 0xEEB1, 0x0AC1, 0, 0, 0x80000000, 0x80000000 },
        { "VSQRT, signalling NaN: quieted", 0xEEB1, 0x0AC1, 0, 0, 0xFF800010, 0xFFC00010 },
    };
    for (int e = 0; e < engines(); e++) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            regs_t in = { 0 };
            in.s[0] = cases[i].d;
            in.s[1] = cases[i].n;
            in.s[2] = cases[i].m;
            const uint16_t code[] = { cases[i].hw1, cases[i].hw2 };
            vfc_t *v;
            run(code, 2, &in, e, &v);
            check(v->cpu.s.u[0] == cases[i].expected, "%s: %s: %08x, expected %08x",
                  engine_name(e), cases[i].name, v->cpu.s.u[0], cases[i].expected);
            vfc_destroy(v);
        }
    }
}

int main(int argc, char **argv)
{
    static const struct { const char *name; void (*run)(void); } tests[] = {
        { "loader", test_loader },
        { "restore", test_restore },
        { "smmla", test_smmla },
        { "vldr_d16", test_vldr_d16 },
        { "ram_edge", test_ram_edge_and_fault_pc },
        { "nans", test_nans },
    };
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        bool chosen = argc < 2;                     // all, or those named
        for (int a = 1; a < argc; a++) {
            chosen |= !strcmp(argv[a], tests[i].name);
        }
        if (chosen) {
            tests[i].run();
        }
    }
    printf("%d checks on the interpreter%s: %d failed\n", checks, vfc_jit_available() ? " and the JIT" : "", failures);
    return failures != 0;
}
