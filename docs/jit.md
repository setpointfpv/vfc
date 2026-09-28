# The JIT

On Apple silicon, vfc translates the firmware's Thumb-2 code to AArch64 and
runs that instead of interpreting it. The results are the same, instruction
for instruction: registers, flags, floating point, memory, instruction counts
and the moment the firmware sleeps. Only the speed differs.

`VFC_JIT=0` in the environment, or `vfc_set_jit(vfc, false)` (Swift:
`usesJIT = false`), keeps a board on the interpreter. Elsewhere than arm64
macOS there is only the interpreter.

## Speed

Betaflight 2026.6.1 on the virtual board, M-series Mac, one core:

| | Interpreter | JIT |
| --- | --- | --- |
| Disarmed, 8 kHz loop (`tools/run`) | 235 MIPS, 6× real time | 2,800 MIPS, 75× real time |
| Armed replay of a real flight, blackbox on (`setpoint-cli replay`) | 4.2× real time | 40× real time |
| Firmware filter measurement, 8 s of flight (`setpoint-cli filters`) | 2.9 s | 0.3 s |

In the armed replay about 85% of the time is in the firmware, most of the rest
in parsing the blackbox log it writes.

## How it works

`jit.c` translates a block at a time: from a flash address up to the first
branch, or 48 instructions. `jit_emit.h` is the AArch64 encoder it uses.

- **Registers.** r0–r14 live in host registers for the whole run (x19–x26 and
  x9–x15); x28 holds the board, x27 the base of guest RAM. The guest's NZCV
  flags are the host's NZCV flags, which AArch64 sets the same way for the
  same operations, including the carry convention for subtraction.
- **Memory.** A load or store checks its address against RAM inline and
  goes straight to it. Anything else takes a cold path: flash for loads; the
  mailbox time registers (the firmware asks the time some thirty times a
  loop), read from a copy the board keeps as time moves; and otherwise a call
  into `vfc_bus_read`/`vfc_bus_write`, after which a fault or reset request
  stops the core.
- **Control flow.** A direct branch to a translated block jumps straight to
  it; one to a block not yet translated goes through a lookup, and is
  patched to jump straight there once the block exists. Indirect branches
  (returns, `POP {pc}`, `BX`) look the target up in a table of blocks by flash
  address, the lookup repeated at each site so the host's branch predictor
  learns each one separately.
- **Budget.** Each block subtracts its length from the budget on entry and
  leaves if it has run out; exits part way through a block add back what
  didn't run, so the instruction count stays exact.
- **Handing back.** `WFI` ends the run for the host to move time on. Anything
  the translator doesn't handle (a handful of rare instructions, a memory
  access to the mailbox inside an IT block, code outside flash) exits to C,
  where the interpreter takes that one instruction and translation carries on
  after it. In a flight loop this is a few hundred instructions in hundreds of
  millions.
- **Code cache.** 32 MB of `MAP_JIT` memory, written with
  `pthread_jit_write_protect_np`; the host app needs the
  `com.apple.security.cs.allow-jit` entitlement. Flash never changes while
  the firmware runs, so translations never go stale; loading an image drops
  them. If the cache fills, the interpreter carries on.
- **Snapshots** carry the machine, not the translations. A snapshot restored
  into another board keeps that board's own JIT and setting.

## Checking it

- `tools/encodings.c` checks the encoder's output against the system
  assembler, one instruction of each form the translator emits.
- `tools/jitdiff.c` runs two boards from the same image side by side, one
  on the JIT and one on the interpreter, and compares their whole state:
  after every block with chaining off, or at every sleep with it on (as in
  use). Clean through boot and 3 s disarmed, and through 6 s of armed flight
  from a snapshot with changing gyro, sticks and motor speeds: over 500 M
  instructions in all.
- Setpoint's replay, filter measurement and virtual-quad outputs are byte
  for byte the same with `VFC_JIT=0` and without.

The interpreter itself is checked against Unicorn (`tools/difftest.c`), so the
chain runs JIT → interpreter → Unicorn.

```sh
cc -O2 -ffp-contract=off -ISources/VFC/include -ISources/VFC Sources/VFC/*.c tools/jitdiff.c -o tools/jitdiff
tools/jitdiff firmware.bin 20000000                     # block by block from reset
tools/jitdiff firmware.bin 100000 --chain --snapshot armed.snap
```

Building with `-DVFC_JIT_STATS` prints, when a board is destroyed, where the
interpreter still ran and how often.
