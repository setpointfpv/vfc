// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// The virtual board: memory map, the mailbox peripheral, loading firmware and
// moving time. The mailbox register map is the ABI shared with firmware; it is
// documented in docs/abi.md and must only change with VFC_ABI.

#include <stdlib.h>
#include <string.h>

#include "vfc_internal.h"

// Mailbox registers (byte offsets). See docs/abi.md.
enum {
    MBX_MAGIC = 0x000,
    MBX_HOST_ABI = 0x004,
    MBX_GUEST_ABI = 0x008,
    MBX_STAGE = 0x00C,
    MBX_TIME_US_LO = 0x010,
    MBX_TIME_US_HI = 0x014,
    MBX_CYCLES = 0x018,
    MBX_IDLE_UNTIL = 0x01C,
    MBX_CLOCK_HZ = 0x020,
    MBX_PUTC = 0x024,
    MBX_RESET = 0x028,
    MBX_FAULT = 0x02C,
    MBX_GYRO_X = 0x040,
    MBX_ACC_X = 0x04C,
    MBX_SENSOR_SEQ = 0x058,
    MBX_RC_COUNT = 0x060,
    MBX_RC_SEQ = 0x064,
    MBX_VBAT_MV = 0x068,
    MBX_CURRENT_MA = 0x06C,
    MBX_RC_BASE = 0x080,
    MBX_MOTOR_COUNT = 0x0C0,
    MBX_MOTOR_SEQ = 0x0C4,
    MBX_MOTOR_BASE = 0x0D0,
    MBX_ERPM_BASE = 0x100,
    MBX_SERIAL_RX_COUNT = 0x140,
    MBX_SERIAL_RX_DATA = 0x144,
    MBX_SERIAL_TX_DATA = 0x148,
};

#define BOARD_INFO_MAGIC 0x56464331u    // "VFC1"
#define DEFAULT_CLOCK_HZ 100000000u

// --- FIFOs

static uint32_t fifo_count(const vfc_fifo_t *f)
{
    return (f->head - f->tail) & (VFC_SERIAL_CAPACITY - 1);
}

static void fifo_push(vfc_fifo_t *f, uint8_t byte)
{
    const uint32_t next = (f->head + 1) & (VFC_SERIAL_CAPACITY - 1);
    if (next == f->tail) {
        return;                             // full: drop, as a UART overrun would
    }
    f->data[f->head] = byte;
    f->head = next;
}

static int fifo_pop(vfc_fifo_t *f)
{
    if (f->head == f->tail) {
        return -1;
    }
    const uint8_t byte = f->data[f->tail];
    f->tail = (f->tail + 1) & (VFC_SERIAL_CAPACITY - 1);
    return byte;
}

// --- Time

static uint32_t cycles_at(const vfc_t *vfc, uint64_t ns)
{
    return (uint32_t)(((unsigned __int128)ns * vfc->clockHz) / 1000000000u);
}

// --- Mailbox

static uint32_t mailbox_read(vfc_t *vfc, uint32_t offset)
{
    const uint64_t us = vfc->timeNs / 1000;
    switch (offset) {
    case MBX_MAGIC: return BOARD_INFO_MAGIC;
    case MBX_HOST_ABI: return VFC_ABI;
    case MBX_GUEST_ABI: return vfc->guestAbi;
    case MBX_STAGE: return vfc->stage;
    case MBX_TIME_US_LO: return (uint32_t)us;
    case MBX_TIME_US_HI: return (uint32_t)(us >> 32);
    case MBX_CYCLES: return cycles_at(vfc, vfc->timeNs);
    case MBX_IDLE_UNTIL: return vfc->idleUntilCycles;
    case MBX_CLOCK_HZ: return vfc->clockHz;
    case MBX_SENSOR_SEQ: return vfc->sensorSeq;
    case MBX_RC_COUNT: return vfc->rcCount;
    case MBX_RC_SEQ: return vfc->rcSeq;
    case MBX_VBAT_MV: return vfc->vbatMv;
    case MBX_CURRENT_MA: return vfc->currentMa;
    case MBX_MOTOR_COUNT: return vfc->motorCount;
    case MBX_MOTOR_SEQ: return vfc->motorSeq;
    case MBX_SERIAL_RX_COUNT: return fifo_count(&vfc->toGuest);
    case MBX_SERIAL_RX_DATA: {
        const int byte = fifo_pop(&vfc->toGuest);
        return byte < 0 ? 0 : (uint32_t)byte;
    }
    default:
        break;
    }
    if (offset >= MBX_GYRO_X && offset < MBX_GYRO_X + 12) {
        return (uint32_t)(int32_t)vfc->gyro[(offset - MBX_GYRO_X) / 4];
    }
    if (offset >= MBX_ACC_X && offset < MBX_ACC_X + 12) {
        return (uint32_t)(int32_t)vfc->acc[(offset - MBX_ACC_X) / 4];
    }
    if (offset >= MBX_RC_BASE && offset < MBX_RC_BASE + 64) {
        return vfc->rc[(offset - MBX_RC_BASE) / 4];
    }
    if (offset >= MBX_MOTOR_BASE && offset < MBX_MOTOR_BASE + 32) {
        return vfc->motor[(offset - MBX_MOTOR_BASE) / 4];
    }
    if (offset >= MBX_ERPM_BASE && offset < MBX_ERPM_BASE + 32) {
        return vfc->erpm[(offset - MBX_ERPM_BASE) / 4];
    }
    return 0;
}

static void mailbox_write(vfc_t *vfc, uint32_t offset, uint32_t value)
{
    switch (offset) {
    case MBX_GUEST_ABI: vfc->guestAbi = value; return;
    case MBX_STAGE: vfc->stage = value; return;
    case MBX_IDLE_UNTIL: vfc->idleUntilCycles = value; return;
    case MBX_PUTC:
        if (vfc->consoleLength < VFC_CONSOLE_CAPACITY) {
            vfc->console[vfc->consoleLength++] = (char)value;
        }
        return;
    case MBX_RESET:
        vfc->stopRequested = true;
        vfc->stopReason = VFC_STOP_RESET;
        return;
    case MBX_FAULT:
        vfc_raise_fault(vfc, "firmware reported fault 0x%x at pc 0x%08x (lr 0x%08x)",
                        value, vfc->cpu.r[15], vfc->cpu.r[14]);
        return;
    case MBX_MOTOR_COUNT: vfc->motorCount = value; return;
    case MBX_MOTOR_SEQ: vfc->motorSeq = value; return;
    case MBX_SERIAL_TX_DATA: fifo_push(&vfc->fromGuest, (uint8_t)value); return;
    default:
        break;
    }
    if (offset >= MBX_MOTOR_BASE && offset < MBX_MOTOR_BASE + 32) {
        vfc->motor[(offset - MBX_MOTOR_BASE) / 4] = value;
    }
}

// --- Bus

uint32_t vfc_bus_read(vfc_t *vfc, uint32_t address, int size)
{
    uint32_t value = 0;
    if (address - VFC_FLASH_BASE < VFC_FLASH_SIZE - (uint32_t)size + 1) {
        memcpy(&value, vfc->flash + (address - VFC_FLASH_BASE), (size_t)size);
        return value;
    }
    if (address < VFC_FLASH_SIZE - (uint32_t)size + 1) {      // boot alias of flash at 0
        memcpy(&value, vfc->flash + address, (size_t)size);
        return value;
    }
    if (address - VFC_RAM_BASE < VFC_RAM_SIZE - (uint32_t)size + 1) {
        memcpy(&value, vfc->ram + (address - VFC_RAM_BASE), (size_t)size);
        return value;
    }
    if (address - VFC_MAILBOX_BASE < VFC_MAILBOX_SIZE) {
        const uint32_t offset = address - VFC_MAILBOX_BASE;
        const uint32_t word = mailbox_read(vfc, offset & ~3u);
        return (word >> (8 * (offset & 3))) & (size == 4 ? 0xFFFFFFFFu : size == 2 ? 0xFFFFu : 0xFFu);
    }
    if (address - VFC_SCS_BASE < VFC_SCS_SIZE - (uint32_t)size + 1) {
        if (address == 0xE000ED00u) {
            return 0x410FC241u;             // CPUID: Cortex-M4 r0p1
        }
        memcpy(&value, vfc->scs + (address - VFC_SCS_BASE), (size_t)size);
        return value;
    }
    vfc_raise_fault(vfc, "bus fault reading %d bytes at 0x%08x (pc 0x%08x)", size, address, vfc->cpu.r[15]);
    return 0;
}

void vfc_bus_write(vfc_t *vfc, uint32_t address, uint32_t value, int size)
{
    if (address - VFC_RAM_BASE < VFC_RAM_SIZE - (uint32_t)size + 1) {
        memcpy(vfc->ram + (address - VFC_RAM_BASE), &value, (size_t)size);
        return;
    }
    if (address - VFC_MAILBOX_BASE < VFC_MAILBOX_SIZE) {
        mailbox_write(vfc, (address - VFC_MAILBOX_BASE) & ~3u, value);
        return;
    }
    if (address - VFC_SCS_BASE < VFC_SCS_SIZE - (uint32_t)size + 1) {
        memcpy(vfc->scs + (address - VFC_SCS_BASE), &value, (size_t)size);
        return;
    }
    vfc_raise_fault(vfc, "bus fault writing %d bytes at 0x%08x (pc 0x%08x)", size, address, vfc->cpu.r[15]);
}

// --- Lifecycle

vfc_t *vfc_create(void)
{
    vfc_t *vfc = calloc(1, sizeof(*vfc));
    if (!vfc) {
        return NULL;
    }
    vfc->flash = calloc(1, VFC_FLASH_SIZE);
    vfc->ram = calloc(1, VFC_RAM_SIZE);
    vfc->scs = calloc(1, VFC_SCS_SIZE);
    if (!vfc->flash || !vfc->ram || !vfc->scs) {
        vfc_destroy(vfc);
        return NULL;
    }
    memset(vfc->flash, 0xFF, VFC_FLASH_SIZE);
    vfc->clockHz = DEFAULT_CLOCK_HZ;
    vfc->vbatMv = 16800;
    return vfc;
}

void vfc_destroy(vfc_t *vfc)
{
    if (!vfc) {
        return;
    }
    free(vfc->flash);
    free(vfc->ram);
    free(vfc->scs);
    free(vfc);
}

static bool config_region_valid(const vfc_t *vfc)
{
    return vfc->eepromSize > 0
        && vfc->eepromAddress - VFC_RAM_BASE < VFC_RAM_SIZE
        && vfc->eepromAddress - VFC_RAM_BASE + vfc->eepromSize <= VFC_RAM_SIZE;
}

void vfc_reset(vfc_t *vfc)
{
    // RAM does not survive a power cycle, except the config storage, which
    // stands in for flash.
    uint8_t *saved = NULL;
    if (config_region_valid(vfc)) {
        saved = malloc(vfc->eepromSize);
        if (saved) {
            memcpy(saved, vfc->ram + (vfc->eepromAddress - VFC_RAM_BASE), vfc->eepromSize);
        }
    }
    memset(vfc->ram, 0, VFC_RAM_SIZE);
    if (saved) {
        memcpy(vfc->ram + (vfc->eepromAddress - VFC_RAM_BASE), saved, vfc->eepromSize);
        free(saved);
    }
    memset(vfc->scs, 0, VFC_SCS_SIZE);

    vfc->stopRequested = false;
    vfc->fault[0] = 0;
    vfc->idleUntilCycles = cycles_at(vfc, vfc->timeNs);
    vfc->guestAbi = 0;
    vfc->stage = 0;
    vfc->motorCount = 0;
    vfc->motorSeq = 0;
    memset(vfc->motor, 0, sizeof(vfc->motor));
    vfc->toGuest.head = vfc->toGuest.tail = 0;
    vfc->fromGuest.head = vfc->fromGuest.tail = 0;
    vfc_cpu_reset(vfc);
}

vfc_error_t vfc_load(vfc_t *vfc, const uint8_t *image, size_t length, uint32_t base)
{
    if (base != VFC_FLASH_BASE || length < 64 || length > VFC_FLASH_SIZE) {
        return VFC_ERR_IMAGE;
    }
    uint32_t vectors[8];
    memcpy(vectors, image, sizeof(vectors));
    const uint32_t info = vectors[7];
    if (vectors[0] - VFC_RAM_BASE > VFC_RAM_SIZE
        || vectors[1] - VFC_FLASH_BASE >= length
        || info - VFC_FLASH_BASE + 24 > length) {
        return VFC_ERR_IMAGE;
    }
    uint32_t fields[6];
    memcpy(fields, image + (info - VFC_FLASH_BASE), sizeof(fields));
    if (fields[0] != BOARD_INFO_MAGIC) {
        return VFC_ERR_IMAGE;
    }
    if (fields[1] != VFC_ABI) {
        return VFC_ERR_ABI;
    }

    memset(vfc->flash, 0xFF, VFC_FLASH_SIZE);
    memcpy(vfc->flash, image, length);
    vfc->flashUsed = (uint32_t)length;
    vfc->eepromAddress = fields[3];
    vfc->eepromSize = fields[4];
    if (!config_region_valid(vfc)) {
        return VFC_ERR_IMAGE;
    }
    vfc->firmwareVersion[0] = 0;
    if (fields[5] - VFC_FLASH_BASE < length) {
        const char *version = (const char *)vfc->flash + (fields[5] - VFC_FLASH_BASE);
        strncpy(vfc->firmwareVersion, version, sizeof(vfc->firmwareVersion) - 1);
    }
    vfc_reset(vfc);
    return VFC_OK;
}

// --- Running

static bool sleeping(const vfc_t *vfc)
{
    return vfc->stopRequested && vfc->stopReason == VFC_STOP_IDLE;
}

vfc_stop_t vfc_run(vfc_t *vfc, uint64_t budget)
{
    if (vfc->stopRequested && vfc->stopReason != VFC_STOP_IDLE) {
        return vfc->stopReason;             // faulted or awaiting reset
    }
    vfc->stopRequested = false;
    vfc_cpu_run(vfc, budget);
    return vfc->stopRequested ? vfc->stopReason : VFC_STOP_BUDGET;
}

void vfc_set_time_ns(vfc_t *vfc, uint64_t ns)
{
    vfc->timeNs = ns;
}

uint64_t vfc_time_ns(const vfc_t *vfc)
{
    return vfc->timeNs;
}

uint64_t vfc_wake_time_ns(const vfc_t *vfc)
{
    const int32_t ahead = (int32_t)(vfc->idleUntilCycles - cycles_at(vfc, vfc->timeNs));
    if (ahead <= 0) {
        return vfc->timeNs;
    }
    // The first nanosecond at which the cycle counter reaches the target.
    uint64_t wake = vfc->timeNs + ((uint64_t)ahead * 1000000000u + vfc->clockHz - 1) / vfc->clockHz;
    while (wake > vfc->timeNs && (int32_t)(vfc->idleUntilCycles - cycles_at(vfc, wake - 1)) <= 0) {
        wake--;
    }
    return wake;
}

vfc_stop_t vfc_advance_to(vfc_t *vfc, uint64_t ns, uint64_t budget)
{
    for (;;) {
        if (sleeping(vfc)) {
            const uint64_t wake = vfc_wake_time_ns(vfc);
            if (wake > ns) {
                if (ns > vfc->timeNs) {
                    vfc->timeNs = ns;
                }
                return VFC_STOP_IDLE;
            }
            if (wake > vfc->timeNs) {
                vfc->timeNs = wake;
            }
        }
        const vfc_stop_t stop = vfc_run(vfc, budget);
        if (stop != VFC_STOP_IDLE) {
            return stop;
        }
    }
}

uint32_t vfc_clock_hz(const vfc_t *vfc)
{
    return vfc->clockHz;
}

// --- Inputs and outputs

void vfc_post_sensor(vfc_t *vfc, const int16_t gyro[3], const int16_t acc[3])
{
    memcpy(vfc->gyro, gyro, sizeof(vfc->gyro));
    memcpy(vfc->acc, acc, sizeof(vfc->acc));
    vfc->sensorSeq++;
}

void vfc_post_rc(vfc_t *vfc, const uint16_t *channels, int count)
{
    if (count > 16) {
        count = 16;
    }
    if (count < 0) {
        count = 0;
    }
    memcpy(vfc->rc, channels, sizeof(uint16_t) * (size_t)count);
    vfc->rcCount = (uint32_t)count;
    vfc->rcSeq++;
}

void vfc_set_battery(vfc_t *vfc, uint32_t millivolts, uint32_t milliamps)
{
    vfc->vbatMv = millivolts;
    vfc->currentMa = milliamps;
}

void vfc_set_erpm(vfc_t *vfc, int motor, uint32_t erpm)
{
    if (motor >= 0 && motor < 8) {
        vfc->erpm[motor] = erpm;
    }
}

int vfc_motor_count(const vfc_t *vfc)
{
    return (int)vfc->motorCount;
}

float vfc_motor(const vfc_t *vfc, int motor)
{
    if (motor < 0 || motor >= 8) {
        return 0;
    }
    float value;
    memcpy(&value, &vfc->motor[motor], sizeof(value));
    return value;
}

uint32_t vfc_motor_seq(const vfc_t *vfc)
{
    return vfc->motorSeq;
}

uint32_t vfc_stage(const vfc_t *vfc)
{
    return vfc->stage;
}

void vfc_serial_write(vfc_t *vfc, const uint8_t *data, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        fifo_push(&vfc->toGuest, data[i]);
    }
}

size_t vfc_serial_read(vfc_t *vfc, uint8_t *out, size_t capacity)
{
    size_t n = 0;
    while (n < capacity) {
        const int byte = fifo_pop(&vfc->fromGuest);
        if (byte < 0) {
            break;
        }
        out[n++] = (uint8_t)byte;
    }
    return n;
}

size_t vfc_config_size(const vfc_t *vfc)
{
    return config_region_valid(vfc) ? vfc->eepromSize : 0;
}

size_t vfc_config_read(const vfc_t *vfc, uint8_t *out, size_t capacity)
{
    const size_t size = vfc_config_size(vfc);
    const size_t n = capacity < size ? capacity : size;
    if (n) {
        memcpy(out, vfc->ram + (vfc->eepromAddress - VFC_RAM_BASE), n);
    }
    return n;
}

void vfc_config_write(vfc_t *vfc, const uint8_t *data, size_t length)
{
    const size_t size = vfc_config_size(vfc);
    const size_t n = length < size ? length : size;
    if (n) {
        memcpy(vfc->ram + (vfc->eepromAddress - VFC_RAM_BASE), data, n);
    }
}

uint64_t vfc_instructions(const vfc_t *vfc)
{
    return vfc->instructions;
}

const char *vfc_fault(const vfc_t *vfc)
{
    return vfc->fault[0] ? vfc->fault : NULL;
}

uint32_t vfc_pc(const vfc_t *vfc)
{
    return vfc->cpu.r[15];
}

uint32_t vfc_reg(const vfc_t *vfc, int n)
{
    return (n >= 0 && n < 16) ? vfc->cpu.r[n] : 0;
}

const char *vfc_firmware_version(const vfc_t *vfc)
{
    return vfc->firmwareVersion;
}

size_t vfc_console_read(vfc_t *vfc, char *out, size_t capacity)
{
    const size_t n = capacity < vfc->consoleLength ? capacity : vfc->consoleLength;
    memcpy(out, vfc->console, n);
    memmove(vfc->console, vfc->console + n, vfc->consoleLength - n);
    vfc->consoleLength -= (uint32_t)n;
    return n;
}
