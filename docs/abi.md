# The virtual board ABI, version 1

This is the whole interface between vfc and firmware built for the virtual
board: a memory map, one block of mailbox registers, a board-info record in the
image, and a stepping contract. Firmware and library are separate programs that
agree on this document and nothing else. Bump `VFC_ABI` (and the firmware's
`VIRTUAL_MAILBOX_ABI`) for any change a built image would notice.

## The core

A Cortex-M4F: ARMv7E-M Thumb-2 with the DSP extension, FPv4-SP single
precision floating point, thread mode only, one stack pointer (MSP). There are
no interrupts or exceptions. `WFI` is the only way the firmware waits.

Build firmware with `-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`.

## Memory map

| Address | Size | What |
| --- | --- | --- |
| `0x0800_0000` | 2 MB | Flash: the image, read and execute only. Also readable at `0x0000_0000`. |
| `0x2000_0000` | 512 KB | RAM. Cleared at reset except the config storage region. |
| `0x4000_0000` | 4 KB | Mailbox registers |
| `0xE000_0000` | 1 MB | System control space: reads return what was written; `0xE000_ED00` (CPUID) reads `0x410F_C241` |

Anything else is a bus fault, which stops the core.

## Board info

Vector table entry 7 (reserved on ARMv7-M) points to this record in flash:

| Offset | Field | Value |
| --- | --- | --- |
| 0 | magic | `0x5646_4331` ("VFC1") |
| 4 | abi | 1 |
| 8 | mailboxBase | `0x4000_0000` |
| 12 | eepromAddress | Start of the config storage region, in RAM |
| 16 | eepromSize | Its size in bytes |
| 20 | firmwareVersion | Pointer to a NUL-terminated version string in flash |

The library refuses an image whose magic or ABI differs. The config storage
region survives resets and is what the host saves and restores as the board's
flash.

## Mailbox registers

32-bit registers at byte offsets from `0x4000_0000`. RO: the firmware only
reads it. WO: the firmware only writes it (the host may read it back).

| Offset | Name | Dir | Meaning |
| --- | --- | --- | --- |
| `0x000` | MAGIC | RO | `0x5646_4331` |
| `0x004` | HOST_ABI | RO | ABI the library implements |
| `0x008` | GUEST_ABI | WO | ABI the firmware was built for |
| `0x00C` | STAGE | WO | Boot progress: 1 system init, 3 running (scheduler idle seen) |
| `0x010` | TIME_US_LO | RO | Host time in microseconds, low word |
| `0x014` | TIME_US_HI | RO | High word |
| `0x018` | CYCLES | RO | Host time in virtual core cycles, wrapping |
| `0x01C` | IDLE_UNTIL | WO | Cycle count when the firmware next has work; write before `WFI` |
| `0x020` | CLOCK_HZ | RO | Virtual core clock (100 MHz) |
| `0x024` | PUTC | WO | Debug console, one character |
| `0x028` | RESET | WO | Any write: the firmware wants a reset |
| `0x02C` | FAULT | WO | Any write: the firmware hit a fault (the value says which) |
| `0x040`–`0x048` | GYRO_X..Z | RO | Gyro sample, int16 counts, 16.384 per deg/s |
| `0x04C`–`0x054` | ACC_X..Z | RO | Accelerometer sample, int16 counts |
| `0x058` | SENSOR_SEQ | RO | Changes each time the host posts a sample |
| `0x060` | RC_COUNT | RO | Channels in the current RC frame |
| `0x064` | RC_SEQ | RO | Changes each time the host posts a frame |
| `0x068` | VBAT_MV | RO | Pack voltage, millivolts |
| `0x06C` | CURRENT_MA | RO | Pack current, milliamps |
| `0x080`–`0x0BC` | RC[0..15] | RO | Channel values, microseconds |
| `0x0C0` | MOTOR_COUNT | WO | Motors the firmware drives |
| `0x0C4` | MOTOR_SEQ | WO | Incremented after each motor update |
| `0x0D0`–`0x0EC` | MOTOR[0..7] | WO | Motor outputs as float32 bits, in the protocol's own units |
| `0x100`–`0x11C` | ERPM[0..7] | RO | Motor speed, eRPM / 100, as bidirectional DShot reports it |
| `0x140` | SERIAL_RX_COUNT | RO | Bytes waiting from the host |
| `0x144` | SERIAL_RX_DATA | RO | Reading pops one byte |
| `0x148` | SERIAL_TX_DATA | WO | Writing sends one byte to the host |
| `0x150` | BLACKBOX_DATA | WO | One byte of blackbox log |
| `0x154` | BLACKBOX_CONTROL | WO | 1 begins a log, 2 ends it |

Unlisted offsets read as zero and ignore writes.

## Stepping

Time belongs to the host and does not move while the firmware runs. The
firmware reads time from TIME_US or CYCLES, and waits only by writing
IDLE_UNTIL and executing `WFI`.

The host's loop, which `vfc_advance_to` implements:

1. Post whatever inputs belong to the current time: a sensor sample, an RC
   frame, motor speeds.
2. If the firmware is asleep and its wake time is before the target, set time
   to the wake time and run until it sleeps again. Repeat.
3. Work due exactly at the target waits for the next step, after the host has
   posted that moment's inputs.
4. Set time to the target.

A sample posted at time *t* is read by the firmware's first gyro task at or
after *t*. Everything is deterministic: the same image and inputs produce the
same outputs, instruction for instruction.
