# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
#
# What 16-bit Thumb instructions do, over Z3 bit-vectors, for prove.py:
# written from the ARMv7-M Architecture Reference Manual's pseudocode
# (AddWithCarry, Shift_C, ConditionPassed, BranchWritePC, BXWritePC...),
# independently of the interpreter in cpu.c. Where the manual says
# UNPREDICTABLE, it does what the interpreter does, since what is proved is
# that the JIT agrees with the interpreter.
#
# Covers the instructions that don't touch memory; the rest raise NotCovered.

import z3


class NotCovered(Exception):
    pass


def bv(value, bits=32):
    return z3.BitVecVal(value & ((1 << bits) - 1), bits)


TRUE, FALSE = z3.BoolVal(True), z3.BoolVal(False)


class State:
    """The part of the core a 16-bit instruction can see or change."""

    def __init__(self, r, n, z, c, v, itstate, primask, faultmask):
        self.r = list(r)                    # r0-r14; the PC is separate
        self.n, self.z, self.c, self.v = n, z, c, v
        self.itstate = itstate              # 8 bits
        self.primask, self.faultmask = primask, faultmask

    def copy(self):
        s = State(self.r, self.n, self.z, self.c, self.v, self.itstate, self.primask, self.faultmask)
        return s


class Outcome:
    """What happened: `kind` is 'next' (on to `pc`), 'wfi' (asleep, on
    waking at `pc`) or 'fault' (stopped at the instruction)."""

    def __init__(self, kind, pc, state):
        self.kind, self.pc, self.state = kind, pc, state


def add_with_carry(x, y, carry):
    unsigned = z3.ZeroExt(1, x) + z3.ZeroExt(1, y) + z3.If(carry, bv(1, 33), bv(0, 33))
    signed = z3.SignExt(1, x) + z3.SignExt(1, y) + z3.If(carry, bv(1, 33), bv(0, 33))
    result = z3.Extract(31, 0, unsigned)
    return result, z3.Extract(32, 32, unsigned) == 1, z3.SignExt(1, result) != signed


LSL, LSR, ASR, ROR, RRX = range(5)


def shift_c(value, kind, amount, carry_in):
    """Shift_C, for an amount that is a Python int or an 8-bit-range bit-vector."""
    if isinstance(amount, int):
        if kind == RRX:
            return z3.Concat(z3.If(carry_in, bv(1, 1), bv(0, 1)), z3.Extract(31, 1, value)), z3.Extract(0, 0, value) == 1
        if amount == 0:
            return value, carry_in
        amount = bv(amount, 64)
        zero = FALSE
    else:
        zero = amount == 0
        amount = z3.ZeroExt(64 - amount.size(), amount)
    wide = z3.ZeroExt(32, value)
    if kind == LSL:
        shifted = wide << amount
        result, carry = z3.Extract(31, 0, shifted), z3.Extract(32, 32, shifted) == 1
    elif kind == LSR:
        result = z3.Extract(31, 0, z3.LShR(wide, amount))
        carry = z3.Extract(0, 0, z3.LShR(wide << 1, amount)) == 1
    elif kind == ASR:
        signed = z3.SignExt(32, value)
        result = z3.Extract(31, 0, signed >> amount)
        carry = z3.Extract(0, 0, (signed << 1) >> amount) == 1
    else:                                   # ROR, by the amount modulo 32
        result = z3.RotateRight(value, z3.Extract(31, 0, amount) & 31)
        carry = z3.Extract(31, 31, result) == 1
    return z3.If(zero, value, result), z3.If(zero, carry_in, carry)


def condition_passed(s, cond):
    base = [s.z, s.c, s.n, s.v, z3.And(s.c, z3.Not(s.z)), s.n == s.v,
            z3.And(s.n == s.v, z3.Not(s.z)), TRUE][cond >> 1]
    if cond & 1 and cond != 15:             # 111x: always
        base = z3.Not(base)
    return base


def set_nz(s, result):
    s.n = z3.Extract(31, 31, result) == 1
    s.z = result == 0


def step16(state, hw, pc):
    """One 16-bit instruction at `pc`, outside an IT block."""
    s = state.copy()
    setflags = True                         # outside an IT block
    reg = lambda i: bv(pc + 4) if i == 15 else s.r[i]
    nxt = bv(pc + 2)
    op = hw >> 11

    def done():
        return Outcome('next', nxt, s)

    def branch(address):                    # BranchWritePC
        return Outcome('next', address & bv(0xFFFFFFFE), s)

    if op <= 2:                             # LSL, LSR, ASR (immediate)
        d, m, imm5 = hw & 7, (hw >> 3) & 7, (hw >> 6) & 31
        kind, amount = [(LSL, imm5), (LSR, imm5 or 32), (ASR, imm5 or 32)][op]
        result, carry = shift_c(s.r[m], kind, amount, s.c)
        s.r[d] = result
        if setflags:
            set_nz(s, result)
            s.c = carry
        return done()
    if op == 3:                             # ADD, SUB (register or 3-bit immediate)
        d, n, x = hw & 7, (hw >> 3) & 7, (hw >> 6) & 7
        operand = bv(x) if hw & 0x400 else s.r[x]
        if hw & 0x200:
            result, carry, overflow = add_with_carry(s.r[n], ~operand, TRUE)
        else:
            result, carry, overflow = add_with_carry(s.r[n], operand, FALSE)
        s.r[d] = result
        if setflags:
            set_nz(s, result)
            s.c, s.v = carry, overflow
        return done()
    if op == 4:                             # MOV (immediate)
        d = (hw >> 8) & 7
        s.r[d] = bv(hw & 0xFF)
        if setflags:
            set_nz(s, s.r[d])
        return done()
    if op == 5:                             # CMP (immediate)
        result, carry, overflow = add_with_carry(s.r[(hw >> 8) & 7], ~bv(hw & 0xFF), TRUE)
        set_nz(s, result)
        s.c, s.v = carry, overflow
        return done()
    if op in (6, 7):                        # ADD, SUB (8-bit immediate)
        dn, imm = (hw >> 8) & 7, bv(hw & 0xFF)
        if op == 7:
            result, carry, overflow = add_with_carry(s.r[dn], ~imm, TRUE)
        else:
            result, carry, overflow = add_with_carry(s.r[dn], imm, FALSE)
        s.r[dn] = result
        if setflags:
            set_nz(s, result)
            s.c, s.v = carry, overflow
        return done()
    if op == 8 and not hw & 0x400:          # data processing (register)
        dn, m = hw & 7, (hw >> 3) & 7
        a, b = s.r[dn], s.r[m]
        opcode = (hw >> 6) & 15
        carry, overflow = s.c, s.v
        write = True
        if opcode == 0:
            result = a & b                                      # AND
        elif opcode == 1:
            result = a ^ b                                      # EOR
        elif opcode in (2, 3, 4, 7):                            # LSL, LSR, ASR, ROR (register)
            kind = {2: LSL, 3: LSR, 4: ASR, 7: ROR}[opcode]
            result, carry = shift_c(a, kind, z3.Extract(7, 0, b), s.c)
        elif opcode == 5:
            result, carry, overflow = add_with_carry(a, b, s.c)             # ADC
        elif opcode == 6:
            result, carry, overflow = add_with_carry(a, ~b, s.c)            # SBC
        elif opcode == 8:
            result, write = a & b, False                        # TST
        elif opcode == 9:
            result, carry, overflow = add_with_carry(~b, bv(0), TRUE)       # RSB #0: Rd = 0 - Rn
        elif opcode == 10:
            result, carry, overflow = add_with_carry(a, ~b, TRUE)           # CMP
            write = False
        elif opcode == 11:
            result, carry, overflow = add_with_carry(a, b, FALSE)           # CMN
            write = False
        elif opcode == 12:
            result = a | b                                      # ORR
        elif opcode == 13:
            result = a * b                                      # MUL: N and Z only
        elif opcode == 14:
            result = a & ~b                                     # BIC
        else:
            result = ~b                                         # MVN
        if write:
            s.r[dn] = result
        if setflags or not write:
            set_nz(s, result)
            s.c, s.v = carry, overflow
        return done()
    if op == 8:                             # special data processing, branch and exchange
        sub, m, dn = (hw >> 8) & 3, (hw >> 3) & 15, (hw & 7) | ((hw >> 4) & 8)
        if sub == 0:                                            # ADD (register), no flags
            result = reg(dn) + reg(m)
            if dn == 15:
                return branch(result)
            s.r[dn] = result
            return done()
        if sub == 1:                                            # CMP (register)
            result, carry, overflow = add_with_carry(reg(dn), ~reg(m), TRUE)
            set_nz(s, result)
            s.c, s.v = carry, overflow
            return done()
        if sub == 2:                                            # MOV (register)
            if dn == 15:
                return branch(reg(m))
            s.r[dn] = reg(m)
            return done()
        target = reg(m)                                         # BX, BLX (register)
        thumb = z3.Extract(0, 0, target) == 1
        if hw & 0x80:
            s.r[14] = bv((pc + 2) | 1)
        # BXWritePC: to ARM state is an INVSTATE fault, taken before LR changes.
        return (Outcome('next', target & bv(0xFFFFFFFE), s), thumb, Outcome('fault', bv(pc), state.copy()))
    if op == 0x14:                          # ADR
        s.r[(hw >> 8) & 7] = bv(((pc + 4) & ~3) + ((hw & 0xFF) << 2))
        return done()
    if op == 0x15:                          # ADD (SP plus immediate)
        s.r[(hw >> 8) & 7] = s.r[13] + bv((hw & 0xFF) << 2)
        return done()
    if op in (0x16, 0x17):                  # miscellaneous
        if hw & 0xFF00 == 0xB000:                               # ADD, SUB SP, SP, #imm
            imm = bv((hw & 0x7F) << 2)
            s.r[13] = s.r[13] - imm if hw & 0x80 else s.r[13] + imm
            return done()
        if hw & 0xF500 == 0xB100:                               # CBZ, CBNZ
            n = hw & 7
            imm = (((hw >> 9) & 1) << 6) | (((hw >> 3) & 31) << 1)
            zero = s.r[n] == 0
            taken = z3.Not(zero) if hw & 0x800 else zero
            return (Outcome('next', bv(pc + 4 + imm), s), taken, Outcome('next', nxt, s))
        if hw & 0xFF00 == 0xB200:                               # SXTH, SXTB, UXTH, UXTB
            d, v = hw & 7, s.r[(hw >> 3) & 7]
            s.r[d] = [z3.SignExt(16, z3.Extract(15, 0, v)), z3.SignExt(24, z3.Extract(7, 0, v)),
                      z3.ZeroExt(16, z3.Extract(15, 0, v)), z3.ZeroExt(24, z3.Extract(7, 0, v))][(hw >> 6) & 3]
            return done()
        if hw & 0xFFE8 == 0xB660:                               # CPS
            value = bv(1 if hw & 0x10 else 0)
            if hw & 1:
                s.faultmask = value
            if hw & 2:
                s.primask = value
            return done()
        if hw & 0xFF00 == 0xBA00 and (hw >> 6) & 3 != 2:        # REV, REV16, REVSH
            d, v = hw & 7, s.r[(hw >> 3) & 7]
            b = [z3.Extract(8 * i + 7, 8 * i, v) for i in range(4)]
            s.r[d] = {0: z3.Concat(b[0], b[1], b[2], b[3]),
                      1: z3.Concat(b[2], b[3], b[0], b[1]),
                      3: z3.SignExt(16, z3.Concat(b[0], b[1]))}[(hw >> 6) & 3]
            return done()
        if hw & 0xFF00 == 0xBF00:
            if hw & 0xF:                                        # IT
                s.itstate = bv(hw & 0xFF, 8)
                return done()
            if (hw >> 4) & 0xF == 3:                            # WFI
                return Outcome('wfi', nxt, s)
            return done()                                       # NOP, YIELD, WFE, SEV and other hints
        raise NotCovered('miscellaneous %04x' % hw)
    if op in (0x1A, 0x1B):                  # B<c>
        cond = (hw >> 8) & 15
        if cond >= 14:
            raise NotCovered('UDF, SVC')
        offset = ((hw & 0xFF) ^ 0x80) - 0x80
        return (Outcome('next', bv(pc + 4 + 2 * offset), s), condition_passed(s, cond), Outcome('next', nxt, s))
    if op == 0x1C:                          # B
        offset = ((hw & 0x7FF) ^ 0x400) - 0x400
        return Outcome('next', bv(pc + 4 + 2 * offset), s)
    raise NotCovered('memory access %04x' % hw)


def outcomes(state, hw, pc):
    """step16 as a list of (condition, Outcome)."""
    result = step16(state, hw, pc)
    if isinstance(result, tuple):
        taken, cond, other = result
        return [(cond, taken), (z3.Not(cond), other)]
    return [(TRUE, result)]
