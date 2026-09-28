// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// Internal state shared by the core (cpu.c) and the board (board.c).

#ifndef VFC_INTERNAL_H
#define VFC_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "vfc.h"

#define VFC_FLASH_BASE      0x08000000u
#define VFC_FLASH_SIZE      (2u * 1024 * 1024)
#define VFC_RAM_BASE        0x20000000u
#define VFC_RAM_SIZE        (512u * 1024)
#define VFC_MAILBOX_BASE    0x40000000u
#define VFC_MAILBOX_SIZE    0x1000u
#define VFC_SCS_BASE        0xE0000000u
#define VFC_SCS_SIZE        0x00100000u

#define VFC_SERIAL_CAPACITY 65536u
#define VFC_CONSOLE_CAPACITY 4096u

typedef struct {
    uint8_t data[VFC_SERIAL_CAPACITY];
    uint32_t head, tail;        // head == tail: empty
} vfc_fifo_t;

typedef struct {
    // Core registers. r[13] is the active stack pointer (MSP only).
    uint32_t r[16];
    bool n, z, c, v, q;
    uint8_t ge;                 // APSR.GE[3:0]
    uint8_t itstate;            // EPSR.IT
    uint32_t primask, basepri, faultmask, control;
    uint32_t psp;

    // FPv4-SP.
    union {
        float f[32];
        uint32_t u[32];
    } s;
    uint32_t fpscr;
} vfc_cpu_t;

struct vfc {
    vfc_cpu_t cpu;

    uint8_t *flash;
    uint32_t flashUsed;
    uint8_t *ram;
    uint8_t *scs;

    uint64_t instructions;
    bool stopRequested;
    vfc_stop_t stopReason;
    char fault[160];

    // Mailbox state.
    uint64_t timeNs;
    uint32_t clockHz;
    uint32_t idleUntilCycles;
    uint32_t guestAbi;
    uint32_t stage;
    int16_t gyro[3];
    int16_t acc[3];
    uint32_t sensorSeq;
    uint16_t rc[16];
    uint32_t rcCount;
    uint32_t rcSeq;
    uint32_t vbatMv, currentMa;
    uint32_t erpm[8];
    uint32_t motorCount;
    uint32_t motorSeq;
    uint32_t motor[8];
    vfc_fifo_t toGuest, fromGuest;
    // Blackbox log bytes, until the host takes them.
    uint8_t *blackbox;
    size_t blackboxLength, blackboxCapacity;
    uint32_t blackboxLogs;
    bool blackboxOpen;
    char console[VFC_CONSOLE_CAPACITY];
    uint32_t consoleLength;

    // From the image's board info.
    uint32_t eepromAddress, eepromSize;
    char firmwareVersion[64];
};

// Bus, implemented by the board.
uint32_t vfc_bus_read(vfc_t *vfc, uint32_t address, int size);
void vfc_bus_write(vfc_t *vfc, uint32_t address, uint32_t value, int size);

// Core.
void vfc_cpu_reset(vfc_t *vfc);
void vfc_cpu_run(vfc_t *vfc, uint64_t budget);
void vfc_raise_fault(vfc_t *vfc, const char *format, ...);

#endif
