// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Development check: the JIT against the interpreter, in lockstep. One board
// runs a translated block at a time; the other interprets the same number of
// instructions; their whole state is compared after every block. With
// --chain, the JIT runs as in use, blocks linked, and the comparison is at
// each sleep.
//
//   cc -O2 -ffp-contract=off -ISources/VFC/include -ISources/VFC Sources/VFC/*.c tools/jitdiff.c -o tools/jitdiff
//   tools/jitdiff firmware.bin [blocks] [--snapshot file] [--chain]

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vfc.h"
#include "vfc_internal.h"

static int compare(const vfc_t *a, const vfc_t *b, bool ram)
{
    int bad = 0;
    for (int i = 0; i < 16; i++) {
        if (a->cpu.r[i] != b->cpu.r[i]) { printf("  r%d interp %08x jit %08x\n", i, a->cpu.r[i], b->cpu.r[i]); bad++; }
    }
    if (a->cpu.n != b->cpu.n || a->cpu.z != b->cpu.z || a->cpu.c != b->cpu.c || a->cpu.v != b->cpu.v) {
        printf("  NZCV interp %d%d%d%d jit %d%d%d%d\n", a->cpu.n, a->cpu.z, a->cpu.c, a->cpu.v, b->cpu.n, b->cpu.z, b->cpu.c, b->cpu.v);
        bad++;
    }
    if (a->cpu.itstate != b->cpu.itstate) { printf("  IT interp %02x jit %02x\n", a->cpu.itstate, b->cpu.itstate); bad++; }
    for (int i = 0; i < 32; i++) {
        if (a->cpu.s.u[i] != b->cpu.s.u[i]) { printf("  s%d interp %08x (%g) jit %08x (%g)\n", i, a->cpu.s.u[i], a->cpu.s.f[i], b->cpu.s.u[i], b->cpu.s.f[i]); bad++; }
    }
    if (a->cpu.fpscr != b->cpu.fpscr) { printf("  FPSCR interp %08x jit %08x\n", a->cpu.fpscr, b->cpu.fpscr); bad++; }
    if (a->cpu.basepri != b->cpu.basepri || a->cpu.primask != b->cpu.primask) { printf("  BASEPRI/PRIMASK differ\n"); bad++; }
    if (a->instructions != b->instructions) { printf("  instructions interp %llu jit %llu\n", (unsigned long long)a->instructions, (unsigned long long)b->instructions); bad++; }
    if (a->stopRequested != b->stopRequested || (a->stopRequested && a->stopReason != b->stopReason)) { printf("  stop interp %d/%d jit %d/%d\n", a->stopRequested, a->stopReason, b->stopRequested, b->stopReason); bad++; }
    if (memcmp(a->motor, b->motor, sizeof(a->motor)) || a->motorSeq != b->motorSeq) { printf("  motors differ\n"); bad++; }
    if (a->blackboxLength != b->blackboxLength || memcmp(a->blackbox, b->blackbox, a->blackboxLength)) { printf("  blackbox differs\n"); bad++; }
    if (ram) {
        for (uint32_t i = 0; i < VFC_RAM_SIZE; i++) {
            if (a->ram[i] != b->ram[i]) { printf("  RAM 0x%08x interp %02x jit %02x\n", VFC_RAM_BASE + i, a->ram[i], b->ram[i]); bad++; break; }
        }
    }
    return bad;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s firmware.bin [blocks] [--snapshot file]\n", argv[0]);
        return 2;
    }
    uint64_t limit = 2000000;
    const char *snapshotPath = NULL;
    bool chain = false;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--snapshot") && i + 1 < argc) snapshotPath = argv[++i];
        else if (!strcmp(argv[i], "--chain")) chain = true;
        else limit = strtoull(argv[i], NULL, 10);
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 1; }
    fseek(f, 0, SEEK_END);
    const long length = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *image = malloc((size_t)length);
    if (fread(image, 1, (size_t)length, f) != (size_t)length) return 1;
    fclose(f);

    vfc_t *interp = vfc_create(), *jit = vfc_create();
    vfc_set_jit(interp, false);
    vfc_set_jit(jit, true);
    if (vfc_load(interp, image, (size_t)length, VFC_FLASH_BASE) != VFC_OK
        || vfc_load(jit, image, (size_t)length, VFC_FLASH_BASE) != VFC_OK) {
        fprintf(stderr, "load failed\n");
        return 1;
    }
    bool armed = false;
    if (snapshotPath) {
        FILE *s = fopen(snapshotPath, "rb");
        fseek(s, 0, SEEK_END);
        const long n = ftell(s);
        fseek(s, 0, SEEK_SET);
        uint8_t *snapshot = malloc((size_t)n);
        if (fread(snapshot, 1, (size_t)n, s) != (size_t)n
            || vfc_restore(interp, snapshot, (size_t)n) != VFC_OK || vfc_restore(jit, snapshot, (size_t)n) != VFC_OK) {
            fprintf(stderr, "snapshot does not fit\n");
            return 1;
        }
        fclose(s);
        armed = true;
    }

    const int16_t acc[3] = { 0, 0, 2048 };
    uint64_t blocks = 0, wakes = 0, jitInstructions = 0;
    uint32_t lastPcs[8] = { 0 };
    while (blocks < limit) {
        const uint32_t pc = jit->cpu.r[15];
        uint64_t n;
        if (chain) {
            const uint64_t before = jit->instructions;
            vfc_run(jit, UINT64_C(1) << 40);
            n = jit->instructions - before;
        } else {
            n = vfc_jit_step(jit);
        }
        jitInstructions += n;
        interp->stopRequested = false;
        vfc_cpu_run(interp, n);
        lastPcs[blocks % 8] = pc;
        blocks++;
        if (compare(interp, jit, blocks % (chain ? 16 : 1024) == 0)) {
            printf("DIVERGED after block %llu at %08x (%llu instructions); recent block starts:",
                   (unsigned long long)blocks, pc, (unsigned long long)n);
            for (int i = 1; i <= 8; i++) printf(" %08x", lastPcs[(blocks + (uint64_t)i) % 8]);
            printf("\n");
            return 1;
        }
        if (jit->stopRequested && jit->stopReason == VFC_STOP_IDLE) {
            const double t = vfc_time_ns(jit) * 1e-9;
            const int16_t gyro[3] = {
                (int16_t)(1640.0 * sin(2 * M_PI * 3.0 * t)),
                (int16_t)(820.0 * sin(2 * M_PI * 7.0 * t + 1.0)),
                (int16_t)(410.0 * sin(2 * M_PI * 1.3 * t + 2.0)),
            };
            vfc_t *both[2] = { interp, jit };
            for (int k = 0; k < 2; k++) {
                vfc_post_sensor(both[k], gyro, acc);
                if (wakes % 32 == 0) {
                    uint16_t rc[8] = { 1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000 };
                    rc[0] = (uint16_t)(1500 + 400 * sin(2 * M_PI * 0.7 * t));
                    rc[1] = (uint16_t)(1500 + 300 * sin(2 * M_PI * 1.1 * t));
                    if (armed) {
                        rc[2] = (uint16_t)(1450 + 250 * sin(2 * M_PI * 0.4 * t));
                        rc[4] = 2000;
                        for (int m = 0; m < 4; m++) vfc_set_erpm100(both[k], m, (uint32_t)(250 + 100 * sin(2 * M_PI * (0.4 * t + 0.1 * m))));
                    }
                    vfc_post_rc(both[k], rc, 8);
                }
                vfc_set_time_ns(both[k], vfc_wake_time_ns(both[k]));
            }
            wakes++;
        } else if (jit->stopRequested) {
            printf("stopped (%d) after block %llu: %s\n", jit->stopReason, (unsigned long long)blocks, vfc_fault(jit) ? vfc_fault(jit) : "");
            break;
        }
    }
    if (!vfc_jit_enabled(jit) || vfc_jit_enabled(interp)) {
        printf("JIT setting lost\n");
        return 1;
    }
    printf("%llu blocks matched (%llu instructions, %.1f per block; %llu wakes, %.3f s virtual)\n",
           (unsigned long long)blocks, (unsigned long long)jitInstructions,
           (double)jitInstructions / (double)(blocks ? blocks : 1), (unsigned long long)wakes, vfc_time_ns(jit) * 1e-9);
    return 0;
}
