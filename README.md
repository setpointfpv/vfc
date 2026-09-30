# vfc

A virtual flight controller: an interpreter for a Cortex-M4F (ARMv7E-M with
single-precision floating point), and on Apple silicon a JIT that translates
the firmware to native code, wired to one invented peripheral, the mailbox.
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
| `Sources/VFC/jit.c`, `jit_emit.h` | Thumb-2 to AArch64 translation (`docs/jit.md`) |
| `Sources/VirtualFC/` | The Swift wrapper |
| `tools/` | Development tools: a runner, differential tests against Unicorn (never linked), the JIT against the interpreter (on firmware, and fuzzed), the JIT emulated for other hosts, and an encoder check |
| `tests/`, `Makefile` | Regression tests and proofs; `make check` (`docs/verification.md`) |
