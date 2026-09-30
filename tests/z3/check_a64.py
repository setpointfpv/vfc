# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
#
# Holds the AArch64 model (a64.py) to Unicorn's AArch64: every distinct
# instruction word the JIT emits for the instructions prove.py proves, run
# from random states in both, comparing every register, the flags and memory.
# The proofs are only as good as the model, so this runs before them.
#
#   python3 tests/z3/check_a64.py <jitdump output> [states per word]

import json
import random
import sys

import z3
from unicorn import Uc, UcError, UC_ARCH_ARM64, UC_MODE_ARM, UC_ERR_FETCH_UNMAPPED
from unicorn.arm64_const import (UC_ARM64_REG_X0, UC_ARM64_REG_X29, UC_ARM64_REG_X30, UC_ARM64_REG_SP,
                                 UC_ARM64_REG_PC, UC_ARM64_REG_NZCV, UC_ARM64_REG_S0, UC_ARM64_REG_CPACR_EL1)

import a64

CODE = 0x10000000
DATA = 0x20000000
DATA_SIZE = 0x20000
MASK64 = (1 << 64) - 1


class Concrete(a64.Machine):
    """The model with concrete values, reading memory from Unicorn's."""

    def __init__(self, read):
        super().__init__('c')
        self.read = read
        self.written = {}

    def load(self, address, size):
        a = z3.simplify(address).as_long()
        data = bytes(self.written.get(a + i, self.read(a + i)) for i in range(size))
        return z3.BitVecVal(int.from_bytes(data, 'little'), 8 * size)

    def store(self, address, value, size):
        a, v = z3.simplify(address).as_long(), z3.simplify(value).as_long()
        for i in range(size):
            self.written[a + i] = (v >> (8 * i)) & 0xFF


def memory_operands(insn):
    """The base and index registers of a load or store the model decodes."""
    if insn & 0x3F200C00 == 0x38200800 or (insn & 0x3F000000 == 0x3C000000 and insn & 0x00200C00 == 0x00200800):
        return (insn >> 5) & 31, (insn >> 16) & 31
    if insn & 0x3F000000 in (0x39000000, 0x3D000000) or insn & 0x7FC00000 in (0x29000000, 0x29400000):
        return (insn >> 5) & 31, None
    return None, None


def value(e):
    return z3.simplify(e).as_long()


def check(uc, insn, rng):
    x = [rng.getrandbits(64) for _ in range(31)]
    sp = DATA + 0x1000
    nzcv = rng.getrandbits(4) << 28
    s = [rng.getrandbits(32) for _ in range(32)]
    base, index = memory_operands(insn)
    if base is not None:                    # keep loads and stores inside the data window
        address = DATA + 0x100 + rng.randrange(0, 0x8000, 1)
        if base == 31:
            sp = address
        else:
            x[base] = address
        if index is not None and index != 31:
            x[index] = (rng.getrandbits(32) & 0xFFF) | (rng.getrandbits(32) << 32)
        if base == index:
            return True
    data = rng.randbytes(DATA_SIZE)

    uc.mem_write(DATA, data)
    uc.mem_write(CODE, insn.to_bytes(4, 'little'))
    for i in range(29):
        uc.reg_write(UC_ARM64_REG_X0 + i, x[i])
    uc.reg_write(UC_ARM64_REG_X29, x[29])
    uc.reg_write(UC_ARM64_REG_X30, x[30])
    uc.reg_write(UC_ARM64_REG_SP, sp)
    uc.reg_write(UC_ARM64_REG_NZCV, nzcv)
    for i in range(32):
        uc.reg_write(UC_ARM64_REG_S0 + i, s[i])
    try:
        uc.emu_start(CODE, CODE + 4, count=1)
    except UcError as e:
        # A branch out of the mapped code: Unicorn has taken it and failed to
        # fetch there, leaving the PC on the target, which is what's checked.
        if e.errno != UC_ERR_FETCH_UNMAPPED:
            raise

    m = Concrete(lambda a: data[a - DATA] if DATA <= a < DATA + DATA_SIZE else 0)
    m.x = [z3.BitVecVal(v, 64) for v in x]
    m.sp = z3.BitVecVal(sp, 64)
    m.set_nzcv(z3.BitVecVal(nzcv, 64))
    m.s = [z3.BitVecVal(v, 32) for v in s]
    ways = a64.execute(m, insn, 0)
    taken = [target for cond, target in ways if z3.is_true(z3.simplify(cond))]
    assert len(taken) == 1, 'no single way on'
    target = taken[0]
    if isinstance(target, tuple):
        expected_pc = value(target[1])
    else:
        expected_pc = (CODE + 4 * target) & MASK64

    problems = []
    got_pc = uc.reg_read(UC_ARM64_REG_PC)
    if got_pc != expected_pc:
        problems.append('pc: model %x unicorn %x' % (expected_pc, got_pc))
    for i in range(31):
        if i == 30 and isinstance(target, tuple) and target[0] == 'call':
            continue                        # BLR's link register: the proofs treat calls separately
        reg = UC_ARM64_REG_X0 + i if i < 29 else (UC_ARM64_REG_X29 if i == 29 else UC_ARM64_REG_X30)
        got, want = uc.reg_read(reg), value(m.x[i])
        if got != want:
            problems.append('x%d: model %x unicorn %x' % (i, want, got))
    if uc.reg_read(UC_ARM64_REG_SP) != value(m.sp):
        problems.append('sp')
    got = uc.reg_read(UC_ARM64_REG_NZCV) & 0xF0000000
    if got != value(m.nzcv()):
        problems.append('nzcv: model %x unicorn %x' % (value(m.nzcv()), got))
    for i in range(32):
        got, want = uc.reg_read(UC_ARM64_REG_S0 + i) & 0xFFFFFFFF, value(m.s[i])
        if got != want:
            problems.append('s%d: model %x unicorn %x' % (i, want, got))
    after = bytes(uc.mem_read(DATA, DATA_SIZE))
    for a, byte in m.written.items():
        if not DATA <= a < DATA + DATA_SIZE or after[a - DATA] != byte:
            problems.append('store at %x' % a)
    if after != data:
        changed = [i for i in range(DATA_SIZE) if after[i] != data[i] and DATA + i not in m.written]
        if changed:
            problems.append('unicorn also wrote %x' % (DATA + changed[0]))
    if problems:
        print('MISMATCH %08x: %s' % (insn, '; '.join(problems[:6])))
        return False
    return True


def main():
    path = sys.argv[1]
    states = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    words = set()
    with open(path) as f:
        next(f)
        for line in f:
            d = json.loads(line)
            if d['translated']:
                words.update(d['code'])
    uc = Uc(UC_ARCH_ARM64, UC_MODE_ARM)
    uc.mem_map(CODE, 0x1000)
    uc.mem_map(DATA, DATA_SIZE)
    uc.reg_write(UC_ARM64_REG_CPACR_EL1, 3 << 20)
    rng = random.Random(1)
    bad = unsupported = 0
    for insn in sorted(words):
        try:
            ok = all(check(uc, insn, rng) for _ in range(states))
        except a64.Unsupported as e:
            unsupported += 1
            print('UNSUPPORTED %08x: %s' % (insn, e))
            continue
        bad += not ok
    print('%d instruction words, %d states each: %d mismatched, %d not modelled'
          % (len(words), states, bad, unsupported))
    return 1 if bad or unsupported else 0


if __name__ == '__main__':
    sys.exit(main())
