// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Development runner: boots a virtual board image, then runs the gyro loop at
// a fixed rate with a still quad, reporting what the firmware did and how fast.
//
//   cc -O3 -ffp-contract=off -ISources/VFC/include Sources/VFC/*.c tools/run.c -o tools/run
//   tools/run firmware.bin [loops] [--cli "command"]

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "vfc.h"

static double now_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void drain(vfc_t *vfc)
{
    char buffer[4096];
    size_t n;
    while ((n = vfc_console_read(vfc, buffer, sizeof(buffer))) > 0) {
        fwrite(buffer, 1, n, stdout);
    }
    uint8_t bytes[4096];
    while ((n = vfc_serial_read(vfc, bytes, sizeof(bytes))) > 0) {
        fwrite(bytes, 1, n, stdout);
    }
}

static int report(vfc_t *vfc, vfc_stop_t stop)
{
    if (stop == VFC_STOP_FAULT) {
        printf("FAULT: %s\n", vfc_fault(vfc));
        for (int i = 0; i < 16; i++) {
            printf("  r%-2d %08x%s", i, vfc_reg(vfc, i), (i % 4 == 3) ? "\n" : "");
        }
        return 1;
    }
    if (stop == VFC_STOP_RESET) {
        printf("firmware asked for a reset\n");
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s firmware.bin [loops] [--cli \"command\"]\n", argv[0]);
        return 2;
    }
    const long loops = argc > 2 && argv[2][0] != '-' ? atol(argv[2]) : 8000;
    const char *cli = NULL;
    for (int i = 2; i < argc - 1; i++) {
        if (!strcmp(argv[i], "--cli")) {
            cli = argv[i + 1];
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
        perror("read");
        return 1;
    }
    fclose(f);

    vfc_t *vfc = vfc_create();
    const vfc_error_t error = vfc_load(vfc, image, (size_t)length, 0x08000000);
    if (error != VFC_OK) {
        fprintf(stderr, "load failed: %d\n", error);
        return 1;
    }
    printf("firmware %s, config %zu bytes\n", vfc_firmware_version(vfc), vfc_config_size(vfc));

    const int16_t gyro[3] = { 0, 0, 0 };
    const int16_t acc[3] = { 0, 0, 2048 };
    const uint64_t budget = 50000000;

    // Boot: let time pass until the scheduler is running and sleeping between loops.
    double start = now_seconds();
    uint64_t t = 0;
    vfc_stop_t stop = VFC_STOP_IDLE;
    while (vfc_stage(vfc) < 3 && t < 10000000000ull) {
        vfc_post_sensor(vfc, gyro, acc);
        t += 125000;
        stop = vfc_advance_to(vfc, t, budget);
        if (stop != VFC_STOP_IDLE) {
            break;
        }
    }
    drain(vfc);
    printf("boot: stage %u at %.3f s virtual, %llu instructions, %.3f s wall, pc %08x\n",
           vfc_stage(vfc), t * 1e-9, (unsigned long long)vfc_instructions(vfc), now_seconds() - start, vfc_pc(vfc));
    if (report(vfc, stop)) {
        return 1;
    }

    // Steady state: a still quad at 8 kHz.
    const uint64_t before = vfc_instructions(vfc);
    const uint32_t motorBefore = vfc_motor_seq(vfc);
    start = now_seconds();
    for (long i = 0; i < loops; i++) {
        vfc_post_sensor(vfc, gyro, acc);
        t += 125000;
        stop = vfc_advance_to(vfc, t, budget);
        if (stop != VFC_STOP_IDLE) {
            break;
        }
    }
    const double wall = now_seconds() - start;
    const uint64_t executed = vfc_instructions(vfc) - before;
    drain(vfc);
    printf("%ld loops: %.0f instructions per loop, %u motor updates, %.1f MIPS, %.1fx real time\n",
           loops, (double)executed / (double)loops, vfc_motor_seq(vfc) - motorBefore,
           executed / wall / 1e6, loops * 125e-6 / wall);
    if (report(vfc, stop)) {
        return 1;
    }

    if (cli) {
        // Enter the CLI with '#', run one command, and show what came back.
        vfc_serial_write(vfc, (const uint8_t *)"#", 1);
        for (int i = 0; i < 1600; i++) {
            vfc_post_sensor(vfc, gyro, acc);
            t += 125000;
            vfc_advance_to(vfc, t, budget);
        }
        drain(vfc);
        vfc_serial_write(vfc, (const uint8_t *)cli, strlen(cli));
        vfc_serial_write(vfc, (const uint8_t *)"\r\n", 2);
        for (int i = 0; i < 8000; i++) {
            vfc_post_sensor(vfc, gyro, acc);
            t += 125000;
            stop = vfc_advance_to(vfc, t, budget);
            if (stop != VFC_STOP_IDLE) {
                break;
            }
        }
        drain(vfc);
        printf("\n");
        return report(vfc, stop);
    }
    vfc_destroy(vfc);
    return 0;
}
