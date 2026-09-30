// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Development check: the JIT against the interpreter one instruction at a
// time, over every 16-bit Thumb encoding, random 32-bit encodings from each
// encoding class, and random IT blocks, each from several random machine
// states chosen to find edges: addresses at the ends of RAM and flash, in
// the mailbox and the system control space; the awkward integers; floats that
// are zeros, infinities, subnormals and NaNs. Unlike jitdiff it needs no
// firmware, and it reaches instructions and states that firmware never does.
//
// Every test is an instruction (or IT block) followed by WFI, in its own slot
// of one image, so that each is translated once, as a block of its own. Both
// boards run it from the same state, and everything is compared: core and
// FPU registers, flags, IT state, instruction counts, stop reasons and fault
// messages, the mailbox, RAM and the system control space.
//
// On Apple silicon the JIT runs natively:
//   cc -O2 -ffp-contract=off -ISources/VFC/include -ISources/VFC Sources/VFC/*.c tools/jitfuzz.c -o tools/jitfuzz
// Elsewhere it runs in Unicorn's AArch64 emulator (see tools/jitemu.c):
//   cc -O2 -ffp-contract=off -DVFC_JIT_EMULATED -ISources/VFC/include -ISources/VFC
//      Sources/VFC/*.c tools/jitemu.c tools/jitfuzz.c -lunicorn -o tools/jitfuzz
//
//   tools/jitfuzz [--seed N] [--states N] [--wide N] [--it N] [--show N]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vfc.h"
#include "vfc_internal.h"

#define CODE 0x100u                     // first test slot, as an offset into flash
#define SLOT 16u                        // bytes per single-instruction test
#define IT_SLOT 32u                     // bytes per IT block test

// --- Random numbers: splitmix64, so a seed reproduces a run exactly.

static uint64_t seed = 1;

static uint64_t rnd(void)
{
    uint64_t z = (seed += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

static uint32_t below(uint32_t n)
{
    return (uint32_t)(rnd() % n);
}

// --- Test programs

typedef struct {
    uint32_t pc;                        // where the test starts
    uint32_t count;                     // instructions to run, WFI included
    uint16_t code[12];                  // for the report
    int halfwords;
} test_t;

static uint8_t image[VFC_FLASH_SIZE];
static uint32_t imageLength;
static test_t *tests;
static int ntests;

static void put16(uint32_t offset, uint16_t v)
{
    memcpy(image + offset, &v, 2);
}

static void put32(uint32_t offset, uint32_t v)
{
    memcpy(image + offset, &v, 4);
}

static bool is_wide(uint32_t hw)
{
    return (hw >> 11) >= 0x1D;
}

static bool is_it(uint32_t hw)
{
    return (hw & 0xFF00) == 0xBF00 && (hw & 0xF);
}

// A random 32-bit encoding, usually from one of the classes vfc decodes.
static void random_wide(uint16_t *hw1, uint16_t *hw2)
{
    static const struct { uint16_t mask1, value1, mask2, value2; } classes[] = {
        { 0xFE00, 0xEA00, 0x8000, 0x0000 },     // data processing, shifted register
        { 0xFA00, 0xF000, 0x8000, 0x0000 },     // data processing, modified immediate
        { 0xFA00, 0xF200, 0x8000, 0x0000 },     // data processing, plain binary immediate
        { 0xF800, 0xF000, 0x8000, 0x8000 },     // branches and miscellaneous control
        { 0xFE00, 0xF800, 0x0000, 0x0000 },     // load and store single
        { 0xFE40, 0xE800, 0x0000, 0x0000 },     // load and store multiple
        { 0xFE40, 0xE840, 0x0000, 0x0000 },     // load and store dual, exclusive, table branch
        { 0xFF00, 0xFA00, 0xF000, 0xF000 },     // data processing, register
        { 0xFF80, 0xFB00, 0x0000, 0x0000 },     // multiply
        { 0xFF80, 0xFB80, 0x0000, 0x0000 },     // long multiply, divide
        { 0xFF00, 0xEE00, 0x0E10, 0x0A00 },     // floating point data processing
        { 0xFF00, 0xEE00, 0x0E10, 0x0A10 },     // floating point register transfers
        { 0xFE00, 0xEC00, 0x0E00, 0x0A00 },     // floating point loads, stores, 64-bit transfers
    };
    const uint32_t n = sizeof(classes) / sizeof(classes[0]);
    const uint32_t pick = below(n + 1);
    const uint16_t r1 = (uint16_t)rnd(), r2 = (uint16_t)rnd();
    if (pick == n) {                        // anything at all with a 32-bit prefix
        *hw1 = (uint16_t)(0xE800 | (r1 & 0x17FF));
        *hw2 = r2;
        return;
    }
    *hw1 = (uint16_t)((r1 & ~classes[pick].mask1) | classes[pick].value1);
    *hw2 = (uint16_t)((r2 & ~classes[pick].mask2) | classes[pick].value2);
}

static test_t *new_test(uint32_t slotBytes)
{
    static uint32_t next = CODE;
    test_t *t = &tests[ntests++];
    memset(t, 0, sizeof(*t));
    t->pc = VFC_FLASH_BASE + next;
    next += slotBytes;
    imageLength = next;
    return t;
}

static void emit(test_t *t, uint16_t hw)
{
    put16(t->pc - VFC_FLASH_BASE + 2u * (uint32_t)t->halfwords, hw);
    t->code[t->halfwords++] = hw;
}

static void build(int wide, int itBlocks)
{
    tests = calloc(0x10000u + (size_t)wide + (size_t)itBlocks, sizeof(test_t));
    // Vector table: stack at the top of RAM, reset at a WFI; board info at 0x20.
    put32(0x00, VFC_RAM_BASE + VFC_RAM_SIZE);
    put32(0x04, VFC_FLASH_BASE + 0x40 + 1);
    put32(0x1C, VFC_FLASH_BASE + 0x20);
    put32(0x20, 0x56464331u);
    put32(0x24, VFC_ABI);
    put32(0x28, VFC_MAILBOX_BASE);
    put32(0x2C, VFC_RAM_BASE + 0x70000);
    put32(0x30, 0x1000);
    put32(0x34, VFC_FLASH_BASE + 0x38);
    memcpy(image + 0x38, "jitfuzz", 8);
    put16(0x40, 0xBF30);

    // Every 16-bit encoding but IT, which comes in blocks below.
    for (uint32_t hw = 0; hw < 0xE800; hw++) {
        if (is_it(hw)) {
            continue;
        }
        test_t *t = new_test(SLOT);
        emit(t, (uint16_t)hw);
        emit(t, 0xBF30);
        t->count = 2;
    }
    for (int i = 0; i < wide; i++) {
        test_t *t = new_test(SLOT);
        uint16_t hw1, hw2;
        random_wide(&hw1, &hw2);
        emit(t, hw1);
        emit(t, hw2);
        emit(t, 0xBF30);
        t->count = 2;
    }
    // IT blocks of random instructions, of either width. Conditions include
    // AL, with its UNPREDICTABLE else-slots, to hold the two to one answer.
    for (int i = 0; i < itBlocks; i++) {
        test_t *t = new_test(IT_SLOT);
        const uint32_t firstcond = below(15), mask = 1 + below(15);
        emit(t, (uint16_t)(0xBF00 | (firstcond << 4) | mask));
        const int length = 4 - __builtin_ctz(mask);
        for (int k = 0; k < length; k++) {
            if (below(3) == 0) {
                uint16_t hw1, hw2;
                random_wide(&hw1, &hw2);
                emit(t, hw1);
                emit(t, hw2);
            } else {
                uint32_t hw;
                do {
                    hw = below(0xE800);
                } while (is_it(hw));
                emit(t, (uint16_t)hw);
            }
        }
        emit(t, 0xBF30);
        t->count = 1u + (uint32_t)length + 1u;
    }
}

// --- Machine states

static uint32_t pick_word(void)
{
    switch (below(16)) {
    case 0: return 0;
    case 1: return 0xFFFFFFFFu;
    case 2: return 0x80000000u;
    case 3: return 0x7FFFFFFFu;
    case 4: return below(70);                                           // shift amounts, small numbers
    case 5: case 6: return VFC_RAM_BASE + below(VFC_RAM_SIZE);          // anywhere in RAM
    case 7: return VFC_RAM_BASE + VFC_RAM_SIZE - 1 - below(40);         // the end of RAM
    case 8: return VFC_RAM_BASE - 8 + below(16);                        // the start of RAM
    case 9: return VFC_FLASH_BASE + below(imageLength);                 // flash, code and all
    case 10: return VFC_FLASH_BASE + VFC_FLASH_SIZE - 8 + below(16);    // the end of flash
    case 11: return VFC_MAILBOX_BASE + below(0x160);                    // the mailbox
    case 12: return 0xE000ED00u + below(64);                            // system control space
    default: return (uint32_t)rnd();
    }
}

static uint32_t pick_float(void)
{
    static const uint32_t special[] = {
        0x00000000, 0x80000000, 0x3F800000, 0xBF800000, 0x7F800000, 0xFF800000,         // zeros, ones, infinities
        0x7FC00000, 0xFFC00000, 0x7FC01234, 0xFFFFFFFF,                                 // quiet NaNs
        0x7F800001, 0xFF812345, 0x7FBFFFFF,                                             // signalling NaNs
        0x00000001, 0x807FFFFF, 0x00800000, 0x7F7FFFFF,                                 // subnormals, extremes
        0x4F000000, 0xCF000000, 0x4F800000, 0xCF000001, 0x4EFFFFFF,                     // around 2^31, 2^32
        0x3F000000, 0x3FC00000, 0x40200000, 0xBFC00000, 0xBF000000,                     // halves, for rounding
    };
    switch (below(4)) {
    case 0: case 1: return special[below(sizeof(special) / sizeof(special[0]))];
    case 2: {
        const float f = (float)((int32_t)below(2001) - 1000);
        uint32_t u;
        memcpy(&u, &f, 4);
        return u;
    }
    default: return (uint32_t)rnd();
    }
}

typedef struct {
    uint32_t r[15];
    uint32_t nzcvq, ge, s[32], fpscr, basepri, primask;
} state_t;

static void pick_state(state_t *s)
{
    for (int i = 0; i < 15; i++) {
        s->r[i] = pick_word();
    }
    if (below(4)) {
        s->r[13] = (VFC_RAM_BASE + below(VFC_RAM_SIZE)) & ~3u;
    }
    s->nzcvq = below(32);
    s->ge = below(16);
    for (int i = 0; i < 32; i++) {
        s->s[i] = pick_float();
    }
    s->fpscr = (uint32_t)below(16) << 28;
    s->basepri = below(4) ? 0 : below(256);
    s->primask = below(2);
}

static void apply(vfc_t *v, const test_t *t, const state_t *s)
{
    vfc_cpu_t *cpu = &v->cpu;
    memcpy(cpu->r, s->r, sizeof(s->r));
    cpu->r[15] = t->pc;
    cpu->n = (s->nzcvq >> 4) & 1;
    cpu->z = (s->nzcvq >> 3) & 1;
    cpu->c = (s->nzcvq >> 2) & 1;
    cpu->v = (s->nzcvq >> 1) & 1;
    cpu->q = s->nzcvq & 1;
    cpu->ge = (uint8_t)s->ge;
    cpu->itstate = 0;
    memcpy(cpu->s.u, s->s, sizeof(s->s));
    cpu->fpscr = s->fpscr;
    cpu->basepri = s->basepri;
    cpu->primask = s->primask;
    v->stopRequested = false;
    v->fault[0] = 0;
}

// --- Comparing

static char report[16384];
static size_t reportLength;

#define DIFF(...) (reportLength += (size_t)snprintf(report + reportLength, sizeof(report) - reportLength, __VA_ARGS__), differences++)

static int compare(const vfc_t *a, const vfc_t *b, bool scs)
{
    int differences = 0;
    reportLength = 0;
    report[0] = 0;
    const vfc_cpu_t *x = &a->cpu, *y = &b->cpu;
    for (int i = 0; i < 16; i++) {
        if (x->r[i] != y->r[i]) DIFF("    r%-2d interp %08x  jit %08x\n", i, x->r[i], y->r[i]);
    }
    if (x->n != y->n || x->z != y->z || x->c != y->c || x->v != y->v || x->q != y->q) {
        DIFF("    NZCVQ interp %d%d%d%d%d  jit %d%d%d%d%d\n", x->n, x->z, x->c, x->v, x->q, y->n, y->z, y->c, y->v, y->q);
    }
    if (x->ge != y->ge) DIFF("    GE interp %x  jit %x\n", x->ge, y->ge);
    if (x->itstate != y->itstate) DIFF("    IT interp %02x  jit %02x\n", x->itstate, y->itstate);
    for (int i = 0; i < 32; i++) {
        if (x->s.u[i] != y->s.u[i]) DIFF("    s%-2d interp %08x (%g)  jit %08x (%g)\n", i, x->s.u[i], x->s.f[i], y->s.u[i], y->s.f[i]);
    }
    if (x->fpscr != y->fpscr) DIFF("    FPSCR interp %08x  jit %08x\n", x->fpscr, y->fpscr);
    if (x->primask != y->primask || x->basepri != y->basepri || x->faultmask != y->faultmask || x->control != y->control) {
        DIFF("    PRIMASK/BASEPRI/FAULTMASK/CONTROL interp %x/%x/%x/%x  jit %x/%x/%x/%x\n",
             x->primask, x->basepri, x->faultmask, x->control, y->primask, y->basepri, y->faultmask, y->control);
    }
    if (a->instructions != b->instructions) {
        DIFF("    instructions interp %llu  jit %llu\n", (unsigned long long)a->instructions, (unsigned long long)b->instructions);
    }
    if (a->stopRequested != b->stopRequested || (a->stopRequested && a->stopReason != b->stopReason)) {
        DIFF("    stop interp %d/%d  jit %d/%d\n", a->stopRequested, a->stopReason, b->stopRequested, b->stopReason);
    }
    if (strcmp(a->fault, b->fault)) DIFF("    fault interp \"%s\"\n          jit    \"%s\"\n", a->fault, b->fault);
    if (memcmp(a->timeRegs, b->timeRegs, sizeof(a->timeRegs)) || a->guestAbi != b->guestAbi || a->stage != b->stage) {
        DIFF("    mailbox time/ABI/stage registers differ\n");
    }
    if (memcmp(a->motor, b->motor, sizeof(a->motor)) || a->motorSeq != b->motorSeq || a->motorCount != b->motorCount) {
        DIFF("    motors differ\n");
    }
    if (a->toGuest.head != b->toGuest.head || a->toGuest.tail != b->toGuest.tail
        || a->fromGuest.head != b->fromGuest.head || a->fromGuest.tail != b->fromGuest.tail
        || memcmp(a->fromGuest.data, b->fromGuest.data, sizeof(a->fromGuest.data))) {
        DIFF("    serial FIFOs differ\n");
    }
    if (a->consoleLength != b->consoleLength || memcmp(a->console, b->console, a->consoleLength)) {
        DIFF("    console differs\n");
    }
    if (a->blackboxLength != b->blackboxLength || a->blackboxOpen != b->blackboxOpen || a->blackboxLogs != b->blackboxLogs
        || (a->blackboxLength && memcmp(a->blackbox, b->blackbox, a->blackboxLength))) {
        DIFF("    blackbox differs\n");
    }
    for (uint32_t i = 0; i < VFC_RAM_SIZE; i++) {
        if (a->ram[i] != b->ram[i]) {
            DIFF("    RAM %08x interp %02x  jit %02x\n", VFC_RAM_BASE + i, a->ram[i], b->ram[i]);
            break;
        }
    }
    if (scs && memcmp(a->scs, b->scs, VFC_SCS_SIZE)) DIFF("    system control space differs\n");
    return differences;
}

// --- Running

static vfc_t *interp, *jit;
static uint8_t *snapshot;

static void resync(void)
{
    vfc_snapshot(interp, snapshot);
    if (vfc_restore(interp, snapshot, vfc_snapshot_size(interp)) != VFC_OK
        || vfc_restore(jit, snapshot, vfc_snapshot_size(jit)) != VFC_OK) {
        fprintf(stderr, "resync failed\n");
        exit(1);
    }
}

static const char *kind(const test_t *t)
{
    if (is_it(t->code[0])) return "IT block";
    if (is_wide(t->code[0])) return "32-bit";
    return "16-bit";
}

int main(int argc, char **argv)
{
    int states = 4, wide = 32768, itBlocks = 4096, show = 12;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--states") && i + 1 < argc) states = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--wide") && i + 1 < argc) wide = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--it") && i + 1 < argc) itBlocks = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--show") && i + 1 < argc) show = atoi(argv[++i]);
        else {
            fprintf(stderr, "usage: %s [--seed N] [--states N] [--wide N] [--it N] [--show N]\n", argv[0]);
            return 2;
        }
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    const uint64_t firstSeed = seed;
    build(wide, itBlocks);
    if (imageLength > VFC_FLASH_SIZE) {
        fprintf(stderr, "too many tests for flash\n");
        return 2;
    }

    interp = vfc_create();
    jit = vfc_create();
    vfc_set_jit(interp, false);
    vfc_set_jit(jit, true);
    if (!vfc_jit_enabled(jit)) {
        fprintf(stderr, "no JIT on this host: build with -DVFC_JIT_EMULATED and tools/jitemu.c\n");
        return 2;
    }
    if (vfc_load(interp, image, imageLength, VFC_FLASH_BASE) != VFC_OK
        || vfc_load(jit, image, imageLength, VFC_FLASH_BASE) != VFC_OK) {
        fprintf(stderr, "load failed\n");
        return 2;
    }
    // The same random RAM in both, and some serial input for the mailbox.
    for (uint32_t i = 0; i < VFC_RAM_SIZE; i += 8) {
        const uint64_t r = rnd();
        memcpy(interp->ram + i, &r, 8);
    }
    for (int i = 0; i < 4096; i++) {
        const uint8_t byte = (uint8_t)rnd();
        vfc_serial_write(interp, &byte, 1);
    }
    snapshot = malloc(vfc_snapshot_size(interp));
    resync();

    uint64_t trials = 0, failures = 0;
    static uint32_t failedBy[3];
    for (int i = 0; i < ntests; i++) {
        const test_t *t = &tests[i];
        for (int k = 0; k < states; k++) {
            state_t s;
            pick_state(&s);
            apply(interp, t, &s);
            apply(jit, t, &s);
            const uint64_t before = interp->instructions;
            jit->instructions = before;
            vfc_run(interp, t->count);
            vfc_run(jit, t->count);
            trials++;
            if (!compare(interp, jit, (trials & 255) == 0)) {
                continue;
            }
            failures++;
            failedBy[is_it(t->code[0]) ? 2 : is_wide(t->code[0]) ? 1 : 0]++;
            if (failures <= (uint64_t)show) {
                printf("MISMATCH %s test at %08x:", kind(t), t->pc);
                for (int h = 0; h < t->halfwords; h++) printf(" %04x", t->code[h]);
                printf("\n  from r0-r14:");
                for (int r = 0; r < 15; r++) printf(" %08x", s.r[r]);
                printf("\n  NZCVQ %x GE %x FPSCR %08x\n  s0-s31:", s.nzcvq, s.ge, s.fpscr);
                for (int r = 0; r < 32; r++) printf(" %08x", s.s[r]);
                printf("\n%s", report);
            }
            resync();
        }
    }
    printf("%llu trials of %d tests (seed %#llx): %llu mismatches (16-bit %u, 32-bit %u, IT blocks %u)\n",
           (unsigned long long)trials, ntests, (unsigned long long)firstSeed, (unsigned long long)failures,
           failedBy[0], failedBy[1], failedBy[2]);
    return failures != 0;
}
