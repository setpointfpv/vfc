# vfc

A virtual flight controller: an interpreter for a Cortex-M4F (ARMv7E-M with
single-precision floating point) wired to one invented peripheral, the mailbox.
Firmware built for the matching virtual board runs unmodified, as data, and the
host drives it in lockstep: it posts sensor samples and stick input, lets the
firmware run until it sleeps, and reads the motor outputs back.

MIT licensed. The firmware it runs is a separate program under its own licence;
the only interface between them is the mailbox register map (`docs/abi.md`).

## Layout

| Path | What |
| --- | --- |
| `Sources/VFC/include/vfc.h` | The C API |
| `Sources/VFC/cpu.c` | The ARMv7E-M + FPv4-SP interpreter |
| `Sources/VFC/board.c` | Memory map, mailbox, loader and stepping |
| `tools/` | Development tools: a runner, and differential tests against Unicorn (never linked) |
