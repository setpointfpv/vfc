// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// CBMC: the JIT's AArch64 logical immediate encoder, for every 32-bit value.
// When it gives an encoding, Arm's DecodeBitMasks turns that encoding back
// into the value; and every value an N=0 encoding stands for is one it
// encodes. Together: it encodes exactly the encodable values, correctly.

#include "jit_emit.h"

uint32_t nondet_u32(void);

// DecodeBitMasks(N=0, imms, immr, immediate=TRUE, M=32), from the Arm ARM.
static bool decode_bit_masks(uint32_t imms, uint32_t immr, uint32_t *value)
{
    int len = -1;
    for (int i = 5; i >= 0; i--) {                  // HighestSetBit(N:NOT(imms)), N = 0
        if (!((imms >> i) & 1)) {
            len = i;
            break;
        }
    }
    if (len < 1) {
        return false;                               // reserved
    }
    const uint32_t levels = (1u << len) - 1;
    const uint32_t s = imms & levels, r = immr & levels;
    if (s == levels) {
        return false;                               // reserved: all ones
    }
    const uint32_t esize = 1u << len;
    const uint32_t welem = (s + 1 == 32) ? 0xFFFFFFFFu : ((1u << (s + 1)) - 1);
    const uint32_t mask = esize == 32 ? 0xFFFFFFFFu : ((1u << esize) - 1);
    const uint32_t element = r ? (((welem >> r) | (welem << (esize - r))) & mask) : welem;
    uint32_t result = 0;
    for (uint32_t i = 0; i < 32; i += esize) {
        result |= element << i;
    }
    *value = result;
    return true;
}

int main(void)
{
    const uint32_t value = nondet_u32();
    uint32_t encoding;
    if (a64_logical_imm(value, &encoding)) {
        __CPROVER_assert((encoding & ~((0x3Fu << 16) | (0x3Fu << 10))) == 0, "only immr and imms are set");
        uint32_t decoded;
        __CPROVER_assert(decode_bit_masks((encoding >> 10) & 0x3F, (encoding >> 16) & 0x3F, &decoded) && decoded == value,
                         "the encoding decodes to the value");
    }
    const uint32_t imms = nondet_u32() & 0x3F, immr = nondet_u32() & 0x1F;
    uint32_t v;
    if (decode_bit_masks(imms, immr, &v)) {
        __CPROVER_assert(a64_logical_imm(v, &encoding), "every encodable value is encoded");
    }
    return 0;
}
