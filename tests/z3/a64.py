# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
#
# A symbolic model of the AArch64 instructions the JIT emits (jit_emit.h),
# over Z3 bit-vectors, for tests/z3/prove.py. It decodes instruction words
# and runs them on a symbolic machine: X0-X30 and SP, NZCV, S0-S31, and one
# byte-addressed memory. Anything it doesn't decode is an error, never a
# guess. tests/z3/check_a64.py holds it to Unicorn's AArch64, instruction by
# instruction, from random states.

import z3

B32 = z3.BitVecSort(32)
B64 = z3.BitVecSort(64)
MASK64 = (1 << 64) - 1


class Unsupported(Exception):
    pass


def bv(value, bits):
    return z3.BitVecVal(value & ((1 << bits) - 1), bits)


class Machine:
    """X registers (64-bit), SP, NZCV as four booleans, S registers
    (32-bit, the low half of V registers) and memory as an array of bytes."""

    def __init__(self, name='a64'):
        self.x = [z3.BitVec('%s_x%d' % (name, i), 64) for i in range(31)]
        self.sp = z3.BitVec('%s_sp' % name, 64)
        self.n, self.z, self.c, self.v = (z3.Bool('%s_%s' % (name, f)) for f in 'nzcv')
        self.s = [z3.BitVec('%s_s%d' % (name, i), 32) for i in range(32)]
        self.mem = z3.Array('%s_mem' % name, B64, z3.BitVecSort(8))
        self.calls = []                 # (target, x0..x3) of each BLR made

    def copy(self):
        m = Machine.__new__(Machine)
        m.x = list(self.x)
        m.sp = self.sp
        m.n, m.z, m.c, m.v = self.n, self.z, self.c, self.v
        m.s = list(self.s)
        m.mem = self.mem
        m.calls = list(self.calls)
        return m

    # Registers: 31 is ZR in most places and SP in some; `sp` says which.
    def rx(self, i, sp=False):
        if i == 31:
            return self.sp if sp else bv(0, 64)
        return self.x[i]

    def rw(self, i, sp=False):
        return z3.Extract(31, 0, self.rx(i, sp))

    def wx(self, i, value, sp=False):
        if i == 31:
            if sp:
                self.sp = value
            return
        self.x[i] = z3.simplify(value)

    def ww(self, i, value, sp=False):
        self.wx(i, z3.ZeroExt(32, value), sp)

    def nzcv(self):
        bit = lambda b, shift: z3.If(b, bv(1 << shift, 64), bv(0, 64))
        return bit(self.n, 31) | bit(self.z, 30) | bit(self.c, 29) | bit(self.v, 28)

    def set_nzcv(self, value):
        bit = lambda shift: z3.Extract(shift, shift, value) == 1
        self.n, self.z, self.c, self.v = bit(31), bit(30), bit(29), bit(28)

    # Memory, little-endian.
    def load(self, address, size):
        parts = [z3.Select(self.mem, address + i) for i in range(size)]
        return z3.simplify(z3.Concat(*reversed(parts))) if size > 1 else parts[0]

    def store(self, address, value, size):
        for i in range(size):
            self.mem = z3.Store(self.mem, address + i, z3.Extract(8 * i + 7, 8 * i, value))


def add_with_carry(x, y, carry, bits):
    """AddWithCarry: the result and N, Z, C, V."""
    wide = bits + 1
    unsigned = z3.ZeroExt(1, x) + z3.ZeroExt(1, y) + z3.ZeroExt(bits, z3.If(carry, bv(1, 1), bv(0, 1)))
    signed = z3.SignExt(1, x) + z3.SignExt(1, y) + z3.ZeroExt(bits, z3.If(carry, bv(1, 1), bv(0, 1)))
    result = z3.Extract(bits - 1, 0, unsigned)
    n = z3.Extract(bits - 1, bits - 1, result) == 1
    zf = result == 0
    c = z3.Extract(wide - 1, wide - 1, unsigned) == 1
    v = z3.SignExt(1, result) != signed
    return result, (n, zf, c, v)


def shift(value, kind, amount, bits):
    if amount == 0:
        return value
    if kind == 0:
        return value << amount
    if kind == 1:
        return z3.LShR(value, amount)
    if kind == 2:
        return value >> amount
    return z3.RotateRight(value, amount)


def decode_bit_masks(n, imms, immr, bits):
    """DecodeBitMasks for logical immediates: the wmask."""
    combined = (n << 6) | (~imms & 0x3F)
    length = combined.bit_length() - 1
    if length < 1:
        raise Unsupported('reserved logical immediate')
    levels = (1 << length) - 1
    s, r = imms & levels, immr & levels
    if s == levels:
        raise Unsupported('reserved logical immediate')
    esize = 1 << length
    welem = (1 << (s + 1)) - 1
    element = ((welem >> r) | (welem << (esize - r))) & ((1 << esize) - 1) if r else welem
    value = 0
    for i in range(0, bits, esize):
        value |= element << i
    return value


def bitfield_masks(imms, immr, bits):
    """DecodeBitMasks for SBFM/BFM/UBFM (immediate FALSE, element = register):
    wmask and tmask."""
    welem = (1 << (imms + 1)) - 1
    wmask = ((welem >> immr) | (welem << (bits - immr))) & ((1 << bits) - 1) if immr else welem
    d = (imms - immr) % bits
    return wmask, (1 << (d + 1)) - 1


def condition(m, cond):
    base = {
        0: m.z, 1: m.c, 2: m.n, 3: m.v,
        4: z3.And(m.c, z3.Not(m.z)), 5: m.n == m.v,
        6: z3.And(m.n == m.v, z3.Not(m.z)), 7: z3.BoolVal(True),
    }[cond >> 1]
    if cond & 1 and cond != 15:
        base = z3.Not(base)
    return base


def signed_offset(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def execute(m, insn, index):
    """Runs the instruction at `index` on `m` (changed in place). Returns the
    ways on: a list of (condition, next index), where a next index of
    ('call', target) or ('branch', register value) means control left the
    buffer that way."""
    rd, rn, rm = insn & 31, (insn >> 5) & 31, (insn >> 16) & 31
    sf = insn >> 31
    bits = 64 if sf else 32
    rr = (lambda i: m.rx(i)) if sf else (lambda i: m.rw(i))
    wr = (lambda i, v: m.wx(i, v)) if sf else (lambda i, v: m.ww(i, v))
    nxt = [(z3.BoolVal(True), index + 1)]

    # Add, subtract, logical: shifted register.
    if insn & 0x1F200000 == 0x0B000000:                         # ADD/ADDS/SUB/SUBS (shifted register)
        op, s, kind, amount = (insn >> 30) & 1, (insn >> 29) & 1, (insn >> 22) & 3, (insn >> 10) & 0x3F
        if kind == 3:
            raise Unsupported('shift type')
        y = shift(rr(rm), kind, amount, bits)
        if op:
            y = ~y
        result, flags = add_with_carry(rr(rn), y, z3.BoolVal(bool(op)), bits)
        if s:
            m.n, m.z, m.c, m.v = flags
        wr(rd, result)
        return nxt
    if insn & 0x1F000000 == 0x0A000000:                         # AND/BIC/ORR/ORN/EOR/EON/ANDS/BICS
        opc, invert, kind, amount = (insn >> 29) & 3, (insn >> 21) & 1, (insn >> 22) & 3, (insn >> 10) & 0x3F
        y = shift(rr(rm), kind, amount, bits)
        if invert:
            y = ~y
        x = rr(rn)
        result = [x & y, x | y, x ^ y, x & y][opc]
        if opc == 3:
            m.n, m.z, m.c, m.v = z3.Extract(bits - 1, bits - 1, result) == 1, result == 0, z3.BoolVal(False), z3.BoolVal(False)
        wr(rd, result)
        return nxt
    if insn & 0x1FE0FC00 == 0x1A000000:                         # ADC/ADCS/SBC/SBCS
        op, s = (insn >> 30) & 1, (insn >> 29) & 1
        y = ~rr(rm) if op else rr(rm)
        result, flags = add_with_carry(rr(rn), y, m.c, bits)
        if s:
            m.n, m.z, m.c, m.v = flags
        wr(rd, result)
        return nxt

    # Data processing, two sources.
    if insn & 0x7FE0F000 == 0x1AC02000:                         # LSLV/LSRV/ASRV/RORV
        kind = (insn >> 10) & 3
        amount = rr(rm) & bv(bits - 1, bits)
        x = rr(rn)
        wr(rd, [x << amount, z3.LShR(x, amount), x >> amount, z3.RotateRight(x, amount)][kind])
        return nxt
    if insn & 0x7FE0F800 == 0x1AC00800:                         # UDIV/SDIV
        x, y = rr(rn), rr(rm)
        if (insn >> 10) & 1:
            quotient = z3.If(y == 0, bv(0, bits), z3.If(z3.And(x == bv(1 << (bits - 1), bits), y == bv(-1, bits)), x, x / y))
        else:
            quotient = z3.If(y == 0, bv(0, bits), z3.UDiv(x, y))
        wr(rd, quotient)
        return nxt

    # Multiply.
    if insn & 0x7FE00000 == 0x1B000000:                         # MADD/MSUB
        ra = (insn >> 10) & 31
        product = rr(rn) * rr(rm)
        wr(rd, rr(ra) - product if (insn >> 15) & 1 else rr(ra) + product)
        return nxt
    if insn & 0xFF608000 == 0x9B200000:                         # SMADDL/UMADDL
        ra = (insn >> 10) & 31
        extend = z3.ZeroExt if (insn >> 23) & 1 else z3.SignExt
        product = extend(32, m.rw(rn)) * extend(32, m.rw(rm))
        m.wx(rd, m.rx(ra) + product)
        return nxt

    # Data processing, one source.
    if insn & 0x7FFF0000 == 0x5AC00000:
        opcode = (insn >> 10) & 0x3F
        x = rr(rn)
        if opcode == 0:                                         # RBIT
            wr(rd, z3.Concat(*[z3.Extract(i, i, x) for i in range(bits)]))
        elif opcode == 1:                                       # REV16
            halves = [z3.Extract(i + 15, i, x) for i in range(0, bits, 16)]
            swapped = [z3.Concat(z3.Extract(7, 0, h), z3.Extract(15, 8, h)) for h in halves]
            wr(rd, z3.Concat(*reversed(swapped)))
        elif opcode == 2 and not sf:                            # REV (32-bit)
            wr(rd, z3.Concat(*[z3.Extract(i + 7, i, x) for i in range(0, 32, 8)]))
        elif opcode == 4:                                       # CLZ
            count = bv(bits, bits)
            for i in range(bits):
                count = z3.If(z3.Extract(i, i, x) == 1, bv(bits - 1 - i, bits), count)
            wr(rd, count)
        else:
            raise Unsupported('one-source opcode %d' % opcode)
        return nxt

    # Conditional select.
    if insn & 0x7FE00C00 == 0x1A800000:                         # CSEL
        cond = (insn >> 12) & 15
        wr(rd, z3.If(condition(m, cond), rr(rn), rr(rm)))
        return nxt

    # Bitfields and EXTR.
    if insn & 0x1F800000 == 0x13000000:                         # SBFM/BFM/UBFM
        opc, immr, imms = (insn >> 29) & 3, (insn >> 16) & 0x3F, (insn >> 10) & 0x3F
        if (insn >> 22) & 1 != sf or (not sf and (immr >= 32 or imms >= 32)):
            raise Unsupported('bitfield form')
        src, dst = rr(rn), rr(rd)
        wmask, tmask = bitfield_masks(imms, immr, bits)
        rotated = z3.RotateRight(src, immr) if immr else src
        bot = (dst & bv(~wmask, bits)) | (rotated & bv(wmask, bits)) if opc == 1 else rotated & bv(wmask, bits)
        if opc == 0:                                            # SBFM: top filled with the sign bit
            top = z3.If(z3.Extract(imms, imms, src) == 1, bv(-1, bits), bv(0, bits))
        else:
            top = dst if opc == 1 else bv(0, bits)
        wr(rd, (top & bv(~tmask, bits)) | (bot & bv(tmask, bits)))
        return nxt
    if insn & 0x7FA00000 == 0x13800000:                         # EXTR
        lsb = (insn >> 10) & 0x3F
        both = z3.Concat(rr(rn), rr(rm))
        wr(rd, z3.Extract(lsb + bits - 1, lsb, both))
        return nxt

    # Add, subtract, logical, move: immediate.
    if insn & 0x1F800000 == 0x11000000:                         # ADD/ADDS/SUB/SUBS (immediate)
        op, s, sh, imm = (insn >> 30) & 1, (insn >> 29) & 1, (insn >> 22) & 1, (insn >> 10) & 0xFFF
        x = m.rx(rn, sp=True) if sf else m.rw(rn, sp=True)
        y = bv(imm << (12 if sh else 0), bits)
        result, flags = add_with_carry(x, ~y if op else y, z3.BoolVal(bool(op)), bits)
        if s:
            m.n, m.z, m.c, m.v = flags
            wr(rd, result)
        elif sf:
            m.wx(rd, result, sp=True)
        else:
            m.ww(rd, result, sp=True)
        return nxt
    if insn & 0x1F800000 == 0x12000000:                         # AND/ORR/EOR/ANDS (immediate)
        opc, nbit, immr, imms = (insn >> 29) & 3, (insn >> 22) & 1, (insn >> 16) & 0x3F, (insn >> 10) & 0x3F
        if nbit and not sf:
            raise Unsupported('logical immediate form')
        imm = bv(decode_bit_masks(nbit, imms, immr, bits), bits)
        x = rr(rn)
        result = [x & imm, x | imm, x ^ imm, x & imm][opc]
        if opc == 3:
            m.n, m.z, m.c, m.v = z3.Extract(bits - 1, bits - 1, result) == 1, result == 0, z3.BoolVal(False), z3.BoolVal(False)
            wr(rd, result)
        elif sf:
            m.wx(rd, result, sp=True)
        else:
            m.ww(rd, result, sp=True)
        return nxt
    if insn & 0x1F800000 == 0x12800000:                         # MOVN/MOVZ/MOVK
        opc, hw, imm = (insn >> 29) & 3, (insn >> 21) & 3, (insn >> 5) & 0xFFFF
        if opc == 1 or (not sf and hw > 1):
            raise Unsupported('move wide form')
        if opc == 3:
            keep = ~(0xFFFF << (16 * hw))
            wr(rd, (rr(rd) & bv(keep, bits)) | bv(imm << (16 * hw), bits))
        else:
            value = imm << (16 * hw)
            wr(rd, bv(~value if opc == 0 else value, bits))
        return nxt

    # System: NZCV only.
    if insn & 0xFFFFFFE0 == 0xD53B4200:                         # MRS Xt, NZCV
        m.wx(rd, m.nzcv())
        return nxt
    if insn & 0xFFFFFFE0 == 0xD51B4200:                         # MSR NZCV, Xt
        m.set_nzcv(m.rx(rd))
        return nxt

    # Branches.
    if insn & 0x7C000000 == 0x14000000:                         # B, BL
        if insn >> 31:
            raise Unsupported('BL')
        return [(z3.BoolVal(True), index + signed_offset(insn & 0x3FFFFFF, 26))]
    if insn & 0xFF000010 == 0x54000000:                         # B.cond
        target = index + signed_offset((insn >> 5) & 0x7FFFF, 19)
        taken = condition(m, insn & 15)
        return [(taken, target), (z3.Not(taken), index + 1)]
    if insn & 0x7E000000 == 0x34000000:                         # CBZ, CBNZ
        target = index + signed_offset((insn >> 5) & 0x7FFFF, 19)
        zero = rr(rd) == 0
        taken = z3.Not(zero) if (insn >> 24) & 1 else zero
        return [(taken, target), (z3.Not(taken), index + 1)]
    if insn & 0x7E000000 == 0x36000000:                         # TBZ, TBNZ
        bit = ((insn >> 31) << 5) | ((insn >> 19) & 31)
        target = index + signed_offset((insn >> 5) & 0x3FFF, 14)
        one = z3.Extract(bit, bit, m.rx(rd)) == 1
        taken = one if (insn >> 24) & 1 else z3.Not(one)
        return [(taken, target), (z3.Not(taken), index + 1)]
    if insn & 0xFFFFFC1F == 0xD61F0000:                         # BR
        return [(z3.BoolVal(True), ('branch', m.rx(rn)))]
    if insn & 0xFFFFFC1F == 0xD63F0000:                         # BLR
        return [(z3.BoolVal(True), ('call', m.rx(rn)))]

    # Loads and stores.
    size = insn >> 30
    if insn & 0x3F200C00 == 0x38200800 and (insn >> 13) & 7 in (2, 3):   # register offset, UXTW or LSL
        opc, option, s = (insn >> 22) & 3, (insn >> 13) & 7, (insn >> 12) & 1
        offset = z3.ZeroExt(32, m.rw(rm)) if option == 2 else m.rx(rm)
        address = m.rx(rn, sp=True) + (offset << (size if s else 0))
        return load_store(m, insn, size, opc, address, nxt)
    if insn & 0x3F000000 == 0x39000000:                         # unsigned immediate offset
        opc = (insn >> 22) & 3
        address = m.rx(rn, sp=True) + bv(((insn >> 10) & 0xFFF) << size, 64)
        return load_store(m, insn, size, opc, address, nxt)
    if insn & 0x3F000000 == 0x3C000000 or insn & 0x3F000000 == 0x3D000000:  # S registers
        opc = (insn >> 22) & 3
        if size != 2 or opc > 1:
            raise Unsupported('SIMD&FP load/store form')
        if insn & 0x3F000000 == 0x3D000000:
            address = m.rx(rn, sp=True) + bv(((insn >> 10) & 0xFFF) << 2, 64)
        elif insn & 0x00200C00 == 0x00200800 and (insn >> 13) & 7 == 2:
            address = m.rx(rn, sp=True) + (z3.ZeroExt(32, m.rw(rm)) << ((insn >> 12) & 1) * 2)
        else:
            raise Unsupported('SIMD&FP load/store form')
        if opc:
            m.s[rd] = m.load(address, 4)
        else:
            m.store(address, m.s[rd], 4)
        return nxt
    if insn & 0x7FC00000 in (0x29000000, 0x29400000):           # STP/LDP W, signed offset
        rt2, imm = (insn >> 10) & 31, signed_offset((insn >> 15) & 0x7F, 7) * 4
        address = m.rx(rn, sp=True) + bv(imm, 64)
        if (insn >> 22) & 1:
            first, second = m.load(address, 4), m.load(address + 4, 4)
            m.ww(rd, first)
            m.ww(rt2, second)
        else:
            m.store(address, m.rw(rd), 4)
            m.store(address + 4, m.rw(rt2), 4)
        return nxt

    # Floating point, single precision: moves only here; arithmetic is in
    # fp.py, which prove.py passes in when it needs it.
    if insn & 0xFFFFFC00 == 0x1E260000:                         # FMOV Wd, Sn
        m.ww(rd, m.s[rn])
        return nxt
    if insn & 0xFFFFFC00 == 0x1E270000:                         # FMOV Sd, Wn
        m.s[rd] = m.rw(rn)
        return nxt
    if FLOAT is not None:
        result = FLOAT(m, insn)
        if result:
            return nxt
    raise Unsupported('instruction %08x' % insn)


def load_store(m, insn, size, opc, address, nxt):
    t = insn & 31
    width = 8 << size
    if opc == 0:                                                # store
        value = m.rx(t) if size == 3 else z3.Extract(width - 1, 0, m.rx(t))
        m.store(address, value, 1 << size)
    elif opc == 1:                                              # load, zero-extended
        value = m.load(address, 1 << size)
        m.wx(t, z3.ZeroExt(64 - width, value) if width < 64 else value)
    elif opc == 3 and size < 2:                                 # load, sign-extended to 32 bits
        value = m.load(address, 1 << size)
        m.ww(t, z3.SignExt(32 - width, value))
    else:
        raise Unsupported('load/store form')
    return nxt


# Floating point arithmetic, set by fp.py if loaded: FLOAT(m, insn) -> bool.
FLOAT = None
