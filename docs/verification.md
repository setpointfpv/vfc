# Verification

vfc is mostly written and maintained by agents, so its claims are checked
by machines rather than by review. Firmware runs (`tools/difftest.c` against
Unicorn, `tools/jitdiff.c` for the JIT) check the paths firmware takes. The
checks here cover the rest: every encoding, every machine state, every input
file. `make check` runs them all; CI (`.github/workflows/verify.yml`) runs
them all on Linux, with the JIT emulated, and all but the proofs on Apple
silicon, with it native.

| Command | What it establishes | Time |
| --- | --- | --- |
| `make test` | Regression tests for each bug below, on the interpreter and the JIT, under ASan and UBSan | seconds |
| `make fpu` | The interpreter's floating point matches Unicorn's Cortex-M4 bit for bit, NaNs included, on every combination of awkward operands | seconds |
| `make fuzz` | The JIT and the interpreter agree, bit for bit, on every 16-bit encoding, 32,768 random 32-bit ones and 4,096 IT blocks, each from 4 edge-biased states | a minute |
| `make encodings` | The AArch64 encoder agrees with an assembler | seconds |
| `make cbmc` | Proofs (CBMC): no out-of-bounds access or undefined behaviour in the loader, snapshot restore and bus for any input; the immediate encoder correct for all 2^32 values | minutes |
| `make prove` | Proofs (Z3): the JIT's code for each 16-bit instruction does what the instruction does, from every state | minutes |

They need Unicorn, CBMC, Z3 and Python's Unicorn: on macOS, `brew install
unicorn cbmc` and `pip3 install z3-solver unicorn`; on Linux, `apt-get install
libunicorn-dev cbmc clang llvm` and the same pip packages. The Makefile finds
Homebrew's Unicorn by itself, and a CMake-built one in `/usr/local`;
`UNICORN="-I<prefix>/include -L<prefix>/lib -lunicorn"` points it elsewhere.

## The checks

**Regression tests** (`tests/regress.c`). One test per bug found, each shown
to fail on the code before its fix. Hand-assembled programs run on both
engines; the NaN cases have expected bit patterns worked out from Arm's
pseudocode.

**Floating point against Unicorn's Cortex-M4** (`tests/fpu.c`). Every
FPv4-SP data-processing instruction, on every combination of 33 awkward
operands (signed zeros, subnormals, halves, the limits of the integer
conversions, infinities, quiet and signalling NaNs with payloads), the
conversions in each rounding mode, and the flags compares leave: 469,953
runs on the interpreter and on Unicorn's Cortex-M4, compared bit for bit.
Before the NaN fix below, 52,568 of them differed. This is what lets
`difftest` compare NaNs exactly, and `make fuzz` holds the JIT to the
interpreter on them in turn.

**The JIT on any host** (`tools/jitemu.c`). Built with `-DVFC_JIT_EMULATED`,
the library runs its generated code in Unicorn's AArch64 emulator instead of
natively; translation, chaining, budgets and the interpreter fallback are
the library's own. The emulator is stricter than hardware: every load and
store the generated code makes must fall in memory it has a reason to touch,
and a call into C leaves every register a callee may change holding junk.
`jitdiff` builds against it too, so firmware runs can be checked on Linux.

**Fuzzing the JIT against the interpreter** (`tools/jitfuzz.c`). Each test
is one instruction, or an IT block, followed by WFI, translated as a block
of its own, and run on both engines from the same random state: addresses at
the ends of RAM and flash and in the mailbox, the awkward integers, zeros,
infinities, subnormals and NaNs. Everything is compared: registers, flags,
IT state, instruction counts, stop reasons, fault messages, the mailbox, RAM.
A mutation check confirms it catches the RAM-edge bug if that is put back.

**CBMC** (`tests/cbmc/`). Bounded model checking of the C: for every input
within the bounds, no access out of bounds, no signed overflow, no undefined
shift or division.

- `load.c`: `vfc_load` on any image of up to 96 bytes; an accepted image has
  its config region in RAM. On the code before the fix it finds both loader
  bugs by itself.
- `restore.c`: `vfc_restore` on any snapshot, then every call that indexes
  with what it restored.
- `bus.c`: inductively, any read or write at any address, from any state
  within the board's invariants, keeps the invariants; so no sequence of
  them goes out of bounds either.
- `logical_imm.c`: the logical immediate encoder gives an encoding exactly
  when the value has one, and Arm's `DecodeBitMasks` turns it back into the
  value.

The proofs build with `-DVFC_CBMC`, which shrinks the memory map (flash,
RAM, the system control space, the FIFOs) to sizes CBMC can hold. The code
sees the sizes only through their names, and none of the bugs found depended
on them. `tests/cbmc/models.c` gives `memchr` and `vsnprintf` bodies from
their contracts in the C standard: without them CBMC treats their results as
arbitrary, which here invented a failure in restore's string check, and
could as easily hide one.

**Proving the JIT's translations** (`tests/z3/`). `jitdump` runs the real
translator on each instruction and prints the block it emits. `prove.py`
runs that block on a symbolic AArch64 machine (`a64.py`) and proves, with
Z3, that every path through it either hands the instruction to the
interpreter having changed nothing at all, or leaves the exit PC, all
fifteen registers, the flags and all of memory exactly as `thumb.py` says the
instruction does, with one instruction taken off the budget. Memory is
compared whole, so a stray store fails a proof; the top halves of host
registers are unknown, so code relying on them fails too.

`thumb.py` is written from the ARMv7-M Architecture Reference Manual's
pseudocode, independently of `cpu.c`; where the manual says UNPREDICTABLE it
follows the interpreter, since the claim is that the JIT agrees with it.
`check_a64.py` holds the AArch64 model to Unicorn first: every distinct word
the JIT emits (26,258 for the 16-bit instructions), from random states.

Of the 54,736 16-bit encodings the JIT translates, the 30,160 that don't
touch memory are proved. Loads and stores are left to `make fuzz`. Proofs of
them were written and dropped
([#2](https://github.com/setpointfpv/vfc/pull/2)): they found no bugs, and
they tripled what CI cost whenever everything had to be proved again. Of
eleven bugs planted in the JIT's memory code to test them, the fuzzer caught
nine. Catching the other two needs the first two items under Next.

## What this found

None of these shows up in firmware runs: each needs a malformed image or
snapshot, or an instruction or state that Betaflight never produces.

- `vfc_load` read about 4 GB past an image whose board-info pointer sat just
  below flash, and `vfc_reset` copied about 4 GB for a config region whose
  size wrapped the bounds check: a crash on a malformed image.
- `vfc_restore` trusted a snapshot's FIFO indices and lengths: an
  out-of-bounds write on the next serial byte.
- VLDR and VSTR of D16-D31, which FPv4-SP doesn't have, wrote past the
  interpreter's register file into the board's `ram` and `scs` pointers,
  handing host memory to the firmware.
- The translator read past the end of its register map for TBB, TBH and
  register-offset loads and stores with the PC as index.
- SMMLA and SMMLS overflowed `int64_t`.
- The JIT's inline RAM check looked at an access's first byte only, so one
  straddling the end of RAM (or flash) went into spare padding where the
  interpreter faults.
- NaN results were the host's, not Arm's: which NaN operand wins (Arm
  prefers a signalling one), the default NaN (x86's is negative), and VMLS,
  VNMLA and VNMLS, which both engines computed with a subtraction that keeps
  a NaN's sign where Arm's negation flips it. `difftest` let any two NaNs
  pass.
- Fault messages named different PCs from the two engines; a faulting VLDR
  left different registers.
- UNPREDICTABLE and UNDEFINED encodings the two engines treated differently:
  a branch before the end of its IT block, IT AL's else-slots, PLD and PLI
  with invalid addressing forms, LDRD writing back into a loaded register.
- The encoder emitted wrong code silently for an immediate it couldn't encode
  or a branch it couldn't reach; the block is now thrown away instead.

## What the checks rely on

- **The AArch64 model**, for the proofs, held to Unicorn on every word used.
- **Unicorn (QEMU)**, as that yardstick, as the Cortex-M4 for `make fpu`, and
  to run the JIT on Linux. On Apple silicon, CI fuzzes the native JIT against
  the interpreter as well.
- **`thumb.py`**, Arm's pseudocode transcribed by hand. It is not yet checked
  against Arm's machine-readable specification.
- **The reduced memory map and library models** under CBMC, above.
- **Z3 and CBMC** themselves.
- **Floating point control**: neither engine models FPSCR's default-NaN,
  flush-to-zero or rounding mode for arithmetic, so a Cortex-M4 would
  differ from both if firmware set them. Betaflight doesn't.

## Next

1. **Fuzzing blocks of several instructions**: each test is one instruction
   or one IT block, so nothing checks what the JIT carries from one
   instruction to the next, such as guest r8-lr, which live in caller-saved
   host registers and must be saved around every call into C.
2. **Distinct time registers in the fuzzer**: time never moves on its boards,
   so the four time registers nearly always all read zero, and a fast path
   that reads the wrong one passes. Each state should give them different
   values.
3. **Fuzzing 32-bit Thumb-2 and FPv4-SP in depth**: the 32,768 random
   encodings are spread over 13 classes, about 2,300 per class, and half the
   floating point data-processing ones are double precision, which FPv4-SP
   doesn't have. Choose tests per instruction rather than per class, and run
   the fuzzer under coverage (gcov over `jit.c`) to find the translator paths
   it never reaches.
4. **The interpreter against Arm's specification**: Arm publishes its
   pseudocode in machine-readable form (ASL) for A-profile, whose AArch32
   part includes the Thumb and VFP instructions vfc implements; through ASLp
   it could replace `thumb.py` as the reference.
5. **CBMC over one interpreter step and one translation**: both run out of
   memory whole. The step needs the bus abstracted (`bus.c` proves it
   separately), since each symbolic access costs as much as `bus.c` does; the
   translator needs its buffers shrunk under `VFC_CBMC`. Then a class of
   encodings at a time, passing the instruction as an argument: through flash,
   CBMC loses sight of its fixed bits.
6. **FPSCR's modes**, modelled in the interpreter and set in the host's FPCR
   by the JIT, whose bits match.
7. **Cortex-M4 silicon** as a yardstick for `difftest`, beside Unicorn.
