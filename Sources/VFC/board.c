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
    MBX_BLACKBOX_DATA = 0x150,
    MBX_BLACKBOX_CONTROL = 0x154,
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

enum { TIME_US_LO, TIME_US_HI, TIME_CYCLES, TIME_IDLE_UNTIL };
_Static_assert(MBX_TIME_US_LO == VFC_MBX_TIME_REGS && MBX_TIME_US_HI == MBX_TIME_US_LO + 4
               && MBX_CYCLES == MBX_TIME_US_LO + 8 && MBX_IDLE_UNTIL == MBX_TIME_US_LO + 12,
               "timeRegs mirrors four consecutive mailbox registers");

static void set_time(vfc_t *vfc, uint64_t ns)
{
    const uint64_t us = ns / 1000;
    vfc->timeNs = ns;
    vfc->timeRegs[TIME_US_LO] = (uint32_t)us;
    vfc->timeRegs[TIME_US_HI] = (uint32_t)(us >> 32);
    vfc->timeRegs[TIME_CYCLES] = cycles_at(vfc, ns);
}

// --- Mailbox

#ifdef VFC_TRACE
// Development builds only: every mailbox access, for replaying a run in a
// reference emulator.
void vfc_trace_mailbox(vfc_t *vfc, bool write, uint32_t offset, uint32_t value);
#endif

static uint32_t mailbox_register(vfc_t *vfc, uint32_t offset)
{
    switch (offset) {
    case MBX_MAGIC: return BOARD_INFO_MAGIC;
    case MBX_HOST_ABI: return VFC_ABI;
    case MBX_GUEST_ABI: return vfc->guestAbi;
    case MBX_STAGE: return vfc->stage;
    case MBX_TIME_US_LO: return vfc->timeRegs[TIME_US_LO];
    case MBX_TIME_US_HI: return vfc->timeRegs[TIME_US_HI];
    case MBX_CYCLES: return vfc->timeRegs[TIME_CYCLES];
    case MBX_IDLE_UNTIL: return vfc->timeRegs[TIME_IDLE_UNTIL];
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

static uint32_t mailbox_read(vfc_t *vfc, uint32_t offset)
{
    const uint32_t value = mailbox_register(vfc, offset);
#ifdef VFC_TRACE
    vfc_trace_mailbox(vfc, false, offset, value);
#endif
    return value;
}

static void mailbox_write(vfc_t *vfc, uint32_t offset, uint32_t value)
{
#ifdef VFC_TRACE
    vfc_trace_mailbox(vfc, true, offset, value);
#endif
    switch (offset) {
    case MBX_GUEST_ABI: vfc->guestAbi = value; return;
    case MBX_STAGE: vfc->stage = value; return;
    case MBX_IDLE_UNTIL: vfc->timeRegs[TIME_IDLE_UNTIL] = value; return;
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
                        value, vfc->instructionPc, vfc->cpu.r[14]);
        return;
    case MBX_MOTOR_COUNT: vfc->motorCount = value; return;
    case MBX_MOTOR_SEQ: vfc->motorSeq = value; return;
    case MBX_SERIAL_TX_DATA: fifo_push(&vfc->fromGuest, (uint8_t)value); return;
    case MBX_BLACKBOX_DATA:
        if (vfc->blackboxLength == vfc->blackboxCapacity) {
            const size_t capacity = vfc->blackboxCapacity ? vfc->blackboxCapacity * 2 : VFC_BLACKBOX_CHUNK;
            uint8_t *grown = realloc(vfc->blackbox, capacity);
            if (!grown) {
                return;
            }
            vfc->blackbox = grown;
            vfc->blackboxCapacity = capacity;
        }
        vfc->blackbox[vfc->blackboxLength++] = (uint8_t)value;
        return;
    case MBX_BLACKBOX_CONTROL:
        if (value == 1) {
            vfc->blackboxOpen = true;
            vfc->blackboxLogs++;
        } else if (value == 2) {
            vfc->blackboxOpen = false;
        }
        return;
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
    vfc_raise_fault(vfc, "bus fault reading %d bytes at 0x%08x (pc 0x%08x)", size, address, vfc->instructionPc);
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
    vfc_raise_fault(vfc, "bus fault writing %d bytes at 0x%08x (pc 0x%08x)", size, address, vfc->instructionPc);
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
    set_time(vfc, 0);
    vfc->vbatMv = 16800;
    const char *jit = getenv("VFC_JIT");
    vfc->jitEnabled = vfc_jit_available() && !(jit && jit[0] == '0');
    return vfc;
}

void vfc_destroy(vfc_t *vfc)
{
    if (!vfc) {
        return;
    }
    vfc_jit_free(vfc);
    free(vfc->blackbox);
    free(vfc->flash);
    free(vfc->ram);
    free(vfc->scs);
    free(vfc);
}

// Whether [address, address + size) is a non-empty part of RAM. The size is
// compared with what's left of RAM, never added to the address, so nothing
// can wrap.
static bool ram_region_valid(uint32_t address, uint32_t size)
{
    const uint32_t offset = address - VFC_RAM_BASE;
    return size > 0 && offset < VFC_RAM_SIZE && size <= VFC_RAM_SIZE - offset;
}

static bool config_region_valid(const vfc_t *vfc)
{
    return ram_region_valid(vfc->eepromAddress, vfc->eepromSize);
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
    set_time(vfc, vfc->timeNs);
    vfc->timeRegs[TIME_IDLE_UNTIL] = vfc->timeRegs[TIME_CYCLES];
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
    // Everything is checked before the board changes. Offsets into the image
    // are compared with what's left of it, never added to, so nothing wraps.
    uint32_t vectors[8];
    memcpy(vectors, image, sizeof(vectors));
    const uint32_t info = vectors[7] - VFC_FLASH_BASE;
    uint32_t fields[6];
    if (vectors[0] - VFC_RAM_BASE > VFC_RAM_SIZE
        || vectors[1] - VFC_FLASH_BASE >= length
        || info > length - sizeof(fields)) {
        return VFC_ERR_IMAGE;
    }
    memcpy(fields, image + info, sizeof(fields));
    if (fields[0] != BOARD_INFO_MAGIC) {
        return VFC_ERR_IMAGE;
    }
    if (fields[1] != VFC_ABI) {
        return VFC_ERR_ABI;
    }
    if (!ram_region_valid(fields[3], fields[4])) {
        return VFC_ERR_IMAGE;
    }

    vfc_jit_flush(vfc);
    memset(vfc->flash, 0xFF, VFC_FLASH_SIZE);
    memcpy(vfc->flash, image, length);
    vfc->flashUsed = (uint32_t)length;
    vfc->eepromAddress = fields[3];
    vfc->eepromSize = fields[4];
    // The version string, cut short at the end of the image.
    const uint32_t version = fields[5] - VFC_FLASH_BASE;
    size_t versionLength = 0;
    if (version < length) {
        const size_t limit = length - version < sizeof(vfc->firmwareVersion) - 1
                           ? length - version : sizeof(vfc->firmwareVersion) - 1;
        while (versionLength < limit && image[version + versionLength]) {
            versionLength++;
        }
        memcpy(vfc->firmwareVersion, image + version, versionLength);
    }
    vfc->firmwareVersion[versionLength] = 0;
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
    if (vfc->jitEnabled) {
        vfc_jit_run(vfc, budget);
    } else {
        vfc_cpu_run(vfc, budget);
    }
    return vfc->stopRequested ? vfc->stopReason : VFC_STOP_BUDGET;
}

void vfc_set_time_ns(vfc_t *vfc, uint64_t ns)
{
    set_time(vfc, ns);
}

uint64_t vfc_time_ns(const vfc_t *vfc)
{
    return vfc->timeNs;
}

uint64_t vfc_wake_time_ns(const vfc_t *vfc)
{
    const int32_t ahead = (int32_t)(vfc->timeRegs[TIME_IDLE_UNTIL] - vfc->timeRegs[TIME_CYCLES]);
    if (ahead <= 0) {
        return vfc->timeNs;
    }
    // The first nanosecond at which the cycle counter reaches the target.
    uint64_t wake = vfc->timeNs + ((uint64_t)ahead * 1000000000u + vfc->clockHz - 1) / vfc->clockHz;
    while (wake > vfc->timeNs && (int32_t)(vfc->timeRegs[TIME_IDLE_UNTIL] - cycles_at(vfc, wake - 1)) <= 0) {
        wake--;
    }
    return wake;
}

vfc_stop_t vfc_advance_to(vfc_t *vfc, uint64_t ns, uint64_t budget)
{
    for (;;) {
        if (sleeping(vfc)) {
            // Work due exactly at `ns` waits for the next call, so the host
            // can post the inputs for that moment first.
            const uint64_t wake = vfc_wake_time_ns(vfc);
            if (wake >= ns) {
                if (ns > vfc->timeNs) {
                    set_time(vfc, ns);
                }
                return VFC_STOP_IDLE;
            }
            if (wake > vfc->timeNs) {
                set_time(vfc, wake);
            }
        }
        const vfc_stop_t stop = vfc_run(vfc, budget);
        if (stop != VFC_STOP_IDLE) {
            return stop;
        }
    }
}

void vfc_set_jit(vfc_t *vfc, bool enabled)
{
    vfc->jitEnabled = enabled && vfc_jit_available();
}

bool vfc_jit_enabled(const vfc_t *vfc)
{
    return vfc->jitEnabled;
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

void vfc_set_erpm100(vfc_t *vfc, int motor, uint32_t erpm100)
{
    if (motor >= 0 && motor < 8) {
        vfc->erpm[motor] = erpm100;
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

size_t vfc_blackbox_read(vfc_t *vfc, uint8_t *out, size_t capacity)
{
    const size_t n = capacity < vfc->blackboxLength ? capacity : vfc->blackboxLength;
    memcpy(out, vfc->blackbox, n);
    memmove(vfc->blackbox, vfc->blackbox + n, vfc->blackboxLength - n);
    vfc->blackboxLength -= n;
    return n;
}

size_t vfc_blackbox_pending(const vfc_t *vfc)
{
    return vfc->blackboxLength;
}

uint32_t vfc_blackbox_logs(const vfc_t *vfc)
{
    return vfc->blackboxLogs;
}

bool vfc_blackbox_logging(const vfc_t *vfc)
{
    return vfc->blackboxOpen;
}

// --- Snapshots

typedef struct {
    uint32_t magic;             // "VFCS"
    uint32_t structSize;        // sizeof(vfc_t): the same build of the library
    uint32_t flashUsed;
    uint32_t flashHash;         // FNV-1a of the image
} snapshot_header_t;

#define SNAPSHOT_MAGIC 0x53434656u

static uint32_t image_hash(const vfc_t *vfc)
{
    uint32_t hash = 2166136261u;
    for (uint32_t i = 0; i < vfc->flashUsed; i++) {
        hash = (hash ^ vfc->flash[i]) * 16777619u;
    }
    return hash;
}

size_t vfc_snapshot_size(const vfc_t *vfc)
{
    (void)vfc;
    return sizeof(snapshot_header_t) + sizeof(vfc_t) + VFC_RAM_SIZE + VFC_SCS_SIZE;
}

// The JIT's fields: its code cache and settings belong to an instance, not to
// the machine state a snapshot carries.
#define JIT_STATE_START offsetof(vfc_t, jitBudget)
#define JIT_STATE_LENGTH (offsetof(vfc_t, flash) - offsetof(vfc_t, jitBudget))

void vfc_snapshot(const vfc_t *vfc, uint8_t *out)
{
    const snapshot_header_t header = {
        .magic = SNAPSHOT_MAGIC,
        .structSize = (uint32_t)sizeof(vfc_t),
        .flashUsed = vfc->flashUsed,
        .flashHash = image_hash(vfc),
    };
    memcpy(out, &header, sizeof(header));
    out += sizeof(header);
    vfc_t copy = *vfc;
    memset((uint8_t *)&copy + JIT_STATE_START, 0, JIT_STATE_LENGTH);
    copy.flash = copy.ram = copy.scs = copy.blackbox = NULL;
    copy.blackboxLength = copy.blackboxCapacity = 0;
    copy.consoleLength = 0;
    copy.fromGuest.head = copy.fromGuest.tail = 0;
    memcpy(out, &copy, sizeof(copy));
    out += sizeof(copy);
    memcpy(out, vfc->ram, VFC_RAM_SIZE);
    out += VFC_RAM_SIZE;
    memcpy(out, vfc->scs, VFC_SCS_SIZE);
}

_Static_assert(sizeof(bool) == 1, "snapshot bools are single bytes");

// A field of the vfc_t inside a snapshot.
#define STATE_FIELD(state, field, out) memcpy(&(out), (state) + offsetof(vfc_t, field), sizeof(out))

// A snapshot is a file, so anything restore would otherwise trust as an
// index, a length, a string or a bool is checked first.
static bool snapshot_state_valid(const uint8_t *state)
{
    uint32_t fifo[4], consoleLength;
    STATE_FIELD(state, toGuest.head, fifo[0]);
    STATE_FIELD(state, toGuest.tail, fifo[1]);
    STATE_FIELD(state, fromGuest.head, fifo[2]);
    STATE_FIELD(state, fromGuest.tail, fifo[3]);
    for (int i = 0; i < 4; i++) {
        if (fifo[i] >= VFC_SERIAL_CAPACITY) {
            return false;
        }
    }
    STATE_FIELD(state, consoleLength, consoleLength);
    if (consoleLength > VFC_CONSOLE_CAPACITY) {
        return false;
    }
    vfc_stop_t stopReason;
    STATE_FIELD(state, stopReason, stopReason);
    if ((uint32_t)stopReason > VFC_STOP_RESET) {
        return false;
    }
    static const size_t bools[] = {
        offsetof(vfc_t, cpu.n), offsetof(vfc_t, cpu.z), offsetof(vfc_t, cpu.c), offsetof(vfc_t, cpu.v),
        offsetof(vfc_t, cpu.q), offsetof(vfc_t, stopRequested), offsetof(vfc_t, blackboxOpen),
    };
    for (size_t i = 0; i < sizeof(bools) / sizeof(bools[0]); i++) {
        if (state[bools[i]] > 1) {
            return false;
        }
    }
    return memchr(state + offsetof(vfc_t, fault), 0, sizeof(((vfc_t *)0)->fault)) != NULL;
}

vfc_error_t vfc_restore(vfc_t *vfc, const uint8_t *snapshot, size_t length)
{
    snapshot_header_t header;
    if (length != vfc_snapshot_size(vfc)) {
        return VFC_ERR_IMAGE;
    }
    memcpy(&header, snapshot, sizeof(header));
    if (header.magic != SNAPSHOT_MAGIC || header.structSize != sizeof(vfc_t)
        || header.flashUsed != vfc->flashUsed || header.flashHash != image_hash(vfc)) {
        return VFC_ERR_IMAGE;
    }
    snapshot += sizeof(header);
    if (!snapshot_state_valid(snapshot)) {
        return VFC_ERR_IMAGE;
    }

    // This instance's own, not the snapshot's: its buffers, its JIT, and what
    // it read from the image when it was loaded.
    uint8_t *flash = vfc->flash, *ram = vfc->ram, *scs = vfc->scs, *blackbox = vfc->blackbox;
    const size_t blackboxCapacity = vfc->blackboxCapacity;
    const uint32_t flashUsed = vfc->flashUsed, clockHz = vfc->clockHz;
    const uint32_t eepromAddress = vfc->eepromAddress, eepromSize = vfc->eepromSize;
    char firmwareVersion[sizeof(vfc->firmwareVersion)];
    memcpy(firmwareVersion, vfc->firmwareVersion, sizeof(firmwareVersion));
    uint8_t jit[JIT_STATE_LENGTH];
    memcpy(jit, (uint8_t *)vfc + JIT_STATE_START, JIT_STATE_LENGTH);
    memcpy(vfc, snapshot, sizeof(vfc_t));
    memcpy((uint8_t *)vfc + JIT_STATE_START, jit, JIT_STATE_LENGTH);
    vfc->flash = flash;
    vfc->ram = ram;
    vfc->scs = scs;
    vfc->blackbox = blackbox;
    vfc->blackboxCapacity = blackboxCapacity;
    vfc->blackboxLength = 0;
    vfc->flashUsed = flashUsed;
    vfc->clockHz = clockHz;
    vfc->eepromAddress = eepromAddress;
    vfc->eepromSize = eepromSize;
    memcpy(vfc->firmwareVersion, firmwareVersion, sizeof(firmwareVersion));
    snapshot += sizeof(vfc_t);
    memcpy(vfc->ram, snapshot, VFC_RAM_SIZE);
    snapshot += VFC_RAM_SIZE;
    memcpy(vfc->scs, snapshot, VFC_SCS_SIZE);
    return VFC_OK;
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
