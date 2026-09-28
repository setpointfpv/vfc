// SPDX-License-Identifier: MIT
//
// vfc: a virtual flight controller.
//
// An interpreter for a plain ARMv7E-M core with single-precision floating
// point (a Cortex-M4F), wired to one invented peripheral, the mailbox, through
// which the host supplies time, sensor samples and stick input and reads back
// motor outputs and a serial byte stream. Firmware built for the matching
// virtual board runs unmodified; it is data to this library, never native code.
//
// Time belongs to the host and stands still while the firmware runs. The
// firmware says when it next has work to do and sleeps (WFI); vfc_run returns
// then, and the host moves time on.
//
// Copyright (c) 2026 Ross Bamford
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#ifndef VFC_H
#define VFC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// The mailbox layout this library implements. Firmware declares the ABI it
/// was built for in its board info; a mismatch is refused at load.
#define VFC_ABI 1

typedef struct vfc vfc_t;

typedef enum {
    VFC_OK = 0,
    VFC_ERR_IMAGE = -1,     ///< Not a virtual board image, or too large
    VFC_ERR_ABI = -2,       ///< Built for a different mailbox ABI
} vfc_error_t;

typedef enum {
    VFC_STOP_IDLE = 0,      ///< The firmware is asleep until vfc_wake_time_ns()
    VFC_STOP_BUDGET = 1,    ///< The instruction budget ran out
    VFC_STOP_FAULT = 2,     ///< See vfc_fault()
    VFC_STOP_RESET = 3,     ///< The firmware asked to be reset
} vfc_stop_t;

vfc_t *vfc_create(void);
void vfc_destroy(vfc_t *vfc);

/// Loads a flat image linked at `base` (normally 0x08000000) and resets the
/// core. The config storage is kept, as flash would be.
vfc_error_t vfc_load(vfc_t *vfc, const uint8_t *image, size_t length, uint32_t base);
/// Resets the core as a power cycle would, keeping config storage.
void vfc_reset(vfc_t *vfc);

/// Runs until the firmware sleeps, faults, asks for a reset, or has executed
/// `budget` more instructions.
vfc_stop_t vfc_run(vfc_t *vfc, uint64_t budget);

// Time.
void vfc_set_time_ns(vfc_t *vfc, uint64_t ns);
uint64_t vfc_time_ns(const vfc_t *vfc);
/// When the sleeping firmware next has work: never earlier than now.
uint64_t vfc_wake_time_ns(const vfc_t *vfc);
/// Moves time to the wake time and runs, repeatedly, for all work due before
/// `ns`; then sets time to `ns`. Work due at exactly `ns` runs in the next
/// call, after the host has posted that moment's inputs. Returns the last
/// stop reason.
vfc_stop_t vfc_advance_to(vfc_t *vfc, uint64_t ns, uint64_t budget);
uint32_t vfc_clock_hz(const vfc_t *vfc);

// Inputs.
/// A gyro and accelerometer sample in sensor counts (16.4 per deg/s at
/// 2000 deg/s full scale), read by the next gyro task.
void vfc_post_sensor(vfc_t *vfc, const int16_t gyro[3], const int16_t acc[3]);
/// A whole RC frame, channel values in microseconds.
void vfc_post_rc(vfc_t *vfc, const uint16_t *channels, int count);
void vfc_set_battery(vfc_t *vfc, uint32_t millivolts, uint32_t milliamps);
/// A motor's speed as bidirectional DShot reports it, in eRPM / 100 (the
/// unit a blackbox log records).
void vfc_set_erpm100(vfc_t *vfc, int motor, uint32_t erpm100);

// Outputs.
int vfc_motor_count(const vfc_t *vfc);
float vfc_motor(const vfc_t *vfc, int motor);
/// Incremented by the firmware after each motor update.
uint32_t vfc_motor_seq(const vfc_t *vfc);
uint32_t vfc_stage(const vfc_t *vfc);

// The serial byte stream (MSP and the CLI).
void vfc_serial_write(vfc_t *vfc, const uint8_t *data, size_t length);
size_t vfc_serial_read(vfc_t *vfc, uint8_t *out, size_t capacity);

// The blackbox log: an ordinary .bbl byte stream, one log per arm.
size_t vfc_blackbox_read(vfc_t *vfc, uint8_t *out, size_t capacity);
size_t vfc_blackbox_pending(const vfc_t *vfc);
/// How many logs the firmware has begun.
uint32_t vfc_blackbox_logs(const vfc_t *vfc);
/// Whether a log is open now.
bool vfc_blackbox_logging(const vfc_t *vfc);

// Config storage, the board's flash.
size_t vfc_config_size(const vfc_t *vfc);
size_t vfc_config_read(const vfc_t *vfc, uint8_t *out, size_t capacity);
void vfc_config_write(vfc_t *vfc, const uint8_t *data, size_t length);

// Snapshots: the whole board (core, RAM, mailbox, time) at a moment, to
// resume from later or copy into another instance running the same image.
// Pending serial, blackbox and console output are not included. A snapshot is
// only valid for the same firmware image and the same build of this library.
size_t vfc_snapshot_size(const vfc_t *vfc);
/// Writes a snapshot into `out`, which must hold vfc_snapshot_size() bytes.
void vfc_snapshot(const vfc_t *vfc, uint8_t *out);
/// Restores a snapshot. Fails with VFC_ERR_IMAGE if it was taken from another
/// image or another build.
vfc_error_t vfc_restore(vfc_t *vfc, const uint8_t *snapshot, size_t length);

// Diagnostics.
uint64_t vfc_instructions(const vfc_t *vfc);
/// Why the core stopped with VFC_STOP_FAULT, or NULL.
const char *vfc_fault(const vfc_t *vfc);
uint32_t vfc_pc(const vfc_t *vfc);
uint32_t vfc_reg(const vfc_t *vfc, int n);
/// Firmware version string from the image's board info.
const char *vfc_firmware_version(const vfc_t *vfc);
/// Characters the firmware wrote to its debug console since the last call.
size_t vfc_console_read(vfc_t *vfc, char *out, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif
