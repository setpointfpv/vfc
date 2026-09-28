// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// A small AArch64 assembler for the JIT: just the encodings it uses, all
// 32-bit (W) forms unless the name says X.

#ifndef VFC_JIT_EMIT_H
#define VFC_JIT_EMIT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t *code;
    uint32_t count;
    uint32_t capacity;
    bool overflow;
} a64_buf_t;

static inline void a64(a64_buf_t *b, uint32_t insn)
{
    if (b->count < b->capacity) {
        b->code[b->count++] = insn;
    } else {
        b->overflow = true;
    }
}

enum { A64_ZR = 31, A64_SP = 31 };

enum {
    COND_EQ = 0, COND_NE = 1, COND_CS = 2, COND_CC = 3, COND_MI = 4, COND_PL = 5, COND_VS = 6, COND_VC = 7,
    COND_HI = 8, COND_LS = 9, COND_GE = 10, COND_LT = 11, COND_GT = 12, COND_LE = 13, COND_AL = 14,
};

enum { SH_LSL = 0, SH_LSR = 1, SH_ASR = 2, SH_ROR = 3 };

// --- Data processing, register

#define A64_RRR(op, d, n, m) ((op) | ((uint32_t)(m) << 16) | ((uint32_t)(n) << 5) | (uint32_t)(d))
#define A64_SHIFTED(op, d, n, m, sh, amt) \
    ((op) | ((uint32_t)(sh) << 22) | ((uint32_t)(m) << 16) | ((uint32_t)(amt) << 10) | ((uint32_t)(n) << 5) | (uint32_t)(d))

static inline void a64_add(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x0B000000u, d, n, m, sh, amt)); }
static inline void a64_adds(a64_buf_t *b, int d, int n, int m, int sh, int amt) { a64(b, A64_SHIFTED(0x2B000000u, d, n, m, sh, amt)); }
static inline void a64_sub(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x4B000000u, d, n, m, sh, amt)); }
static inline void a64_subs(a64_buf_t *b, int d, int n, int m, int sh, int amt) { a64(b, A64_SHIFTED(0x6B000000u, d, n, m, sh, amt)); }
static inline void a64_and(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x0A000000u, d, n, m, sh, amt)); }
static inline void a64_bic(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x0A200000u, d, n, m, sh, amt)); }
static inline void a64_orr(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x2A000000u, d, n, m, sh, amt)); }
static inline void a64_orn(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x2A200000u, d, n, m, sh, amt)); }
static inline void a64_eor(a64_buf_t *b, int d, int n, int m, int sh, int amt)  { a64(b, A64_SHIFTED(0x4A000000u, d, n, m, sh, amt)); }
static inline void a64_ands(a64_buf_t *b, int d, int n, int m, int sh, int amt) { a64(b, A64_SHIFTED(0x6A000000u, d, n, m, sh, amt)); }
static inline void a64_adc(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_RRR(0x1A000000u, d, n, m)); }
static inline void a64_adcs(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x3A000000u, d, n, m)); }
static inline void a64_sbc(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_RRR(0x5A000000u, d, n, m)); }
static inline void a64_sbcs(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x7A000000u, d, n, m)); }
static inline void a64_mov(a64_buf_t *b, int d, int m)         { a64_orr(b, d, A64_ZR, m, SH_LSL, 0); }

// Variable shifts (amount taken modulo 32).
static inline void a64_lslv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC02000u, d, n, m)); }
static inline void a64_lsrv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC02400u, d, n, m)); }
static inline void a64_asrv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC02800u, d, n, m)); }
static inline void a64_rorv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC02C00u, d, n, m)); }
static inline void a64_lslv_x(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x9AC02000u, d, n, m)); }
static inline void a64_lsrv_x(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x9AC02400u, d, n, m)); }
static inline void a64_asrv_x(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x9AC02800u, d, n, m)); }

static inline void a64_udiv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC00800u, d, n, m)); }
static inline void a64_sdiv(a64_buf_t *b, int d, int n, int m) { a64(b, A64_RRR(0x1AC00C00u, d, n, m)); }
static inline void a64_madd(a64_buf_t *b, int d, int n, int m, int a)
{
    a64(b, 0x1B000000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_msub(a64_buf_t *b, int d, int n, int m, int a)
{
    a64(b, 0x1B008000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
// X-register long multiplies from W sources: d = a + n * m.
static inline void a64_smaddl(a64_buf_t *b, int d, int n, int m, int a)
{
    a64(b, 0x9B200000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_umaddl(a64_buf_t *b, int d, int n, int m, int a)
{
    a64(b, 0x9BA00000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}

static inline void a64_clz(a64_buf_t *b, int d, int n)   { a64(b, 0x5AC01000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_rbit(a64_buf_t *b, int d, int n)  { a64(b, 0x5AC00000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_rev(a64_buf_t *b, int d, int n)   { a64(b, 0x5AC00800u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_rev16(a64_buf_t *b, int d, int n) { a64(b, 0x5AC00400u | ((uint32_t)n << 5) | (uint32_t)d); }

static inline void a64_csel(a64_buf_t *b, int d, int n, int m, int cond)
{
    a64(b, 0x1A800000u | ((uint32_t)m << 16) | ((uint32_t)cond << 12) | ((uint32_t)n << 5) | (uint32_t)d);
}

// --- Bitfields

static inline void a64_ubfm(a64_buf_t *b, int d, int n, int immr, int imms)
{
    a64(b, 0x53000000u | ((uint32_t)immr << 16) | ((uint32_t)imms << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_sbfm(a64_buf_t *b, int d, int n, int immr, int imms)
{
    a64(b, 0x13000000u | ((uint32_t)immr << 16) | ((uint32_t)imms << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_ubfm_x(a64_buf_t *b, int d, int n, int immr, int imms)
{
    a64(b, 0xD3400000u | ((uint32_t)immr << 16) | ((uint32_t)imms << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_bfm(a64_buf_t *b, int d, int n, int immr, int imms)
{
    a64(b, 0x33000000u | ((uint32_t)immr << 16) | ((uint32_t)imms << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_lsl_imm(a64_buf_t *b, int d, int n, int s) { a64_ubfm(b, d, n, (32 - s) & 31, 31 - s); }
static inline void a64_lsr_imm(a64_buf_t *b, int d, int n, int s) { a64_ubfm(b, d, n, s, 31); }
static inline void a64_asr_imm(a64_buf_t *b, int d, int n, int s) { a64_sbfm(b, d, n, s, 31); }
static inline void a64_ror_imm(a64_buf_t *b, int d, int n, int s)
{
    a64(b, 0x13800000u | ((uint32_t)n << 16) | ((uint32_t)s << 10) | ((uint32_t)n << 5) | (uint32_t)d);   // EXTR
}
static inline void a64_ubfx(a64_buf_t *b, int d, int n, int lsb, int width) { a64_ubfm(b, d, n, lsb, lsb + width - 1); }
static inline void a64_sbfx(a64_buf_t *b, int d, int n, int lsb, int width) { a64_sbfm(b, d, n, lsb, lsb + width - 1); }
static inline void a64_bfi(a64_buf_t *b, int d, int n, int lsb, int width)  { a64_bfm(b, d, n, (32 - lsb) & 31, width - 1); }
static inline void a64_uxtb(a64_buf_t *b, int d, int n) { a64_ubfm(b, d, n, 0, 7); }
static inline void a64_uxth(a64_buf_t *b, int d, int n) { a64_ubfm(b, d, n, 0, 15); }
static inline void a64_sxtb(a64_buf_t *b, int d, int n) { a64_sbfm(b, d, n, 0, 7); }
static inline void a64_sxth(a64_buf_t *b, int d, int n) { a64_sbfm(b, d, n, 0, 15); }

// --- Immediates

static inline void a64_add_imm(a64_buf_t *b, int d, int n, uint32_t imm12, bool shift12)
{
    a64(b, 0x11000000u | ((uint32_t)shift12 << 22) | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_adds_imm(a64_buf_t *b, int d, int n, uint32_t imm12)
{
    a64(b, 0x31000000u | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_sub_imm(a64_buf_t *b, int d, int n, uint32_t imm12, bool shift12)
{
    a64(b, 0x51000000u | ((uint32_t)shift12 << 22) | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_subs_imm(a64_buf_t *b, int d, int n, uint32_t imm12)
{
    a64(b, 0x71000000u | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_add_x_imm(a64_buf_t *b, int d, int n, uint32_t imm12)
{
    a64(b, 0x91000000u | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_sub_x_imm(a64_buf_t *b, int d, int n, uint32_t imm12)
{
    a64(b, 0xD1000000u | ((imm12 & 0xFFF) << 10) | ((uint32_t)n << 5) | (uint32_t)d);
}
static inline void a64_movz(a64_buf_t *b, int d, uint32_t imm16, int hw)
{
    a64(b, 0x52800000u | ((uint32_t)hw << 21) | ((imm16 & 0xFFFF) << 5) | (uint32_t)d);
}
static inline void a64_movk(a64_buf_t *b, int d, uint32_t imm16, int hw)
{
    a64(b, 0x72800000u | ((uint32_t)hw << 21) | ((imm16 & 0xFFFF) << 5) | (uint32_t)d);
}
static inline void a64_movn(a64_buf_t *b, int d, uint32_t imm16, int hw)
{
    a64(b, 0x12800000u | ((uint32_t)hw << 21) | ((imm16 & 0xFFFF) << 5) | (uint32_t)d);
}
static inline void a64_mov32(a64_buf_t *b, int d, uint32_t value)
{
    if ((value & 0xFFFF0000u) == 0) {
        a64_movz(b, d, value, 0);
    } else if ((value & 0xFFFFu) == 0) {
        a64_movz(b, d, value >> 16, 1);
    } else if ((value & 0xFFFF0000u) == 0xFFFF0000u) {
        a64_movn(b, d, ~value & 0xFFFF, 0);
    } else {
        a64_movz(b, d, value & 0xFFFF, 0);
        a64_movk(b, d, value >> 16, 1);
    }
}
static inline void a64_mov64(a64_buf_t *b, int d, uint64_t value)
{
    a64(b, 0xD2800000u | (((uint32_t)value & 0xFFFF) << 5) | (uint32_t)d);        // MOVZ X
    for (int hw = 1; hw < 4; hw++) {
        const uint32_t part = (uint32_t)(value >> (16 * hw)) & 0xFFFF;
        if (part) {
            a64(b, 0xF2800000u | ((uint32_t)hw << 21) | (part << 5) | (uint32_t)d);   // MOVK X
        }
    }
}

// W-register logical immediate, when the value has the right shape.
static inline bool a64_logical_imm(uint32_t value, uint32_t *encoding)
{
    if (value == 0 || value == 0xFFFFFFFFu) {
        return false;
    }
    for (uint32_t size = 2; size <= 32; size *= 2) {
        const uint32_t mask = size == 32 ? 0xFFFFFFFFu : ((1u << size) - 1);
        const uint32_t element = value & mask;
        bool repeats = true;
        for (uint32_t i = size; i < 32; i += size) {
            if (((value >> i) & mask) != element) {
                repeats = false;
                break;
            }
        }
        if (!repeats) {
            continue;
        }
        // Find a rotation that makes the element a run of ones from bit 0.
        for (uint32_t r = 0; r < size; r++) {
            const uint32_t rotated = size == 32
                ? ((element >> r) | (element << ((32 - r) & 31))) & mask
                : ((element >> r) | (element << (size - r))) & mask;
            if (r == 0 && size < 32) {
                // (handled by the general case)
            }
            if ((rotated & (rotated + 1)) == 0 && rotated != 0) {
                const uint32_t ones = (uint32_t)__builtin_popcount(rotated);
                const uint32_t immr = (size - r) % size;
                const uint32_t imms = ((~(size - 1) << 1) & 0x3F) | (ones - 1);
                *encoding = (immr << 16) | (imms << 10);
                return true;
            }
        }
        return false;
    }
    return false;
}

static inline bool a64_and_imm(a64_buf_t *b, int d, int n, uint32_t value)
{
    uint32_t e;
    if (!a64_logical_imm(value, &e)) return false;
    a64(b, 0x12000000u | e | ((uint32_t)n << 5) | (uint32_t)d);
    return true;
}
static inline bool a64_orr_imm(a64_buf_t *b, int d, int n, uint32_t value)
{
    uint32_t e;
    if (!a64_logical_imm(value, &e)) return false;
    a64(b, 0x32000000u | e | ((uint32_t)n << 5) | (uint32_t)d);
    return true;
}

// --- System

static inline void a64_mrs_nzcv(a64_buf_t *b, int t) { a64(b, 0xD53B4200u | (uint32_t)t); }
static inline void a64_msr_nzcv(a64_buf_t *b, int t) { a64(b, 0xD51B4200u | (uint32_t)t); }

// --- Branches (offsets in instructions, patched later when not yet known)

static inline uint32_t a64_b(int32_t offset)        { return 0x14000000u | ((uint32_t)offset & 0x03FFFFFFu); }
static inline uint32_t a64_bcond(int cond, int32_t offset) { return 0x54000000u | (((uint32_t)offset & 0x7FFFF) << 5) | (uint32_t)cond; }
static inline uint32_t a64_cbz(int t, int32_t offset)  { return 0x34000000u | (((uint32_t)offset & 0x7FFFF) << 5) | (uint32_t)t; }
static inline uint32_t a64_cbnz(int t, int32_t offset) { return 0x35000000u | (((uint32_t)offset & 0x7FFFF) << 5) | (uint32_t)t; }
static inline uint32_t a64_cbz_x(int t, int32_t offset)  { return 0xB4000000u | (((uint32_t)offset & 0x7FFFF) << 5) | (uint32_t)t; }
static inline uint32_t a64_tbz(int t, int bit, int32_t offset)
{
    return 0x36000000u | ((uint32_t)(bit >> 5) << 31) | ((uint32_t)(bit & 31) << 19) | (((uint32_t)offset & 0x3FFF) << 5) | (uint32_t)t;
}
static inline uint32_t a64_tbnz(int t, int bit, int32_t offset)
{
    return 0x37000000u | ((uint32_t)(bit >> 5) << 31) | ((uint32_t)(bit & 31) << 19) | (((uint32_t)offset & 0x3FFF) << 5) | (uint32_t)t;
}
static inline void a64_br(a64_buf_t *b, int n)  { a64(b, 0xD61F0000u | ((uint32_t)n << 5)); }
static inline void a64_blr(a64_buf_t *b, int n) { a64(b, 0xD63F0000u | ((uint32_t)n << 5)); }
static inline void a64_ret(a64_buf_t *b)        { a64(b, 0xD65F03C0u); }

// Re-targets a branch at `at` to `target` (both indices in the same buffer).
static inline void a64_patch(uint32_t *code, uint32_t at, uint32_t target)
{
    const int32_t offset = (int32_t)target - (int32_t)at;
    uint32_t insn = code[at];
    if ((insn & 0x7C000000u) == 0x14000000u) {                 // B, BL
        insn = (insn & 0xFC000000u) | ((uint32_t)offset & 0x03FFFFFFu);
    } else if ((insn & 0xFF000010u) == 0x54000000u) {          // B.cond
        insn = (insn & 0xFF00001Fu) | (((uint32_t)offset & 0x7FFFF) << 5);
    } else if ((insn & 0x7E000000u) == 0x34000000u) {          // CBZ, CBNZ
        insn = (insn & 0xFF00001Fu) | (((uint32_t)offset & 0x7FFFF) << 5);
    } else if ((insn & 0x7E000000u) == 0x36000000u) {          // TBZ, TBNZ
        insn = (insn & 0xFFF8001Fu) | (((uint32_t)offset & 0x3FFF) << 5);
    }
    code[at] = insn;
}

// --- Loads and stores

// [Xn, Wm, UXTW]
static inline void a64_ldr_uxtw(a64_buf_t *b, int t, int n, int m)   { a64(b, 0xB8604800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_str_uxtw(a64_buf_t *b, int t, int n, int m)   { a64(b, 0xB8204800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldrb_uxtw(a64_buf_t *b, int t, int n, int m)  { a64(b, 0x38604800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_strb_uxtw(a64_buf_t *b, int t, int n, int m)  { a64(b, 0x38204800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldrh_uxtw(a64_buf_t *b, int t, int n, int m)  { a64(b, 0x78604800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_strh_uxtw(a64_buf_t *b, int t, int n, int m)  { a64(b, 0x78204800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldrsb_uxtw(a64_buf_t *b, int t, int n, int m) { a64(b, 0x38E04800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldrsh_uxtw(a64_buf_t *b, int t, int n, int m) { a64(b, 0x78E04800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldr_s_uxtw(a64_buf_t *b, int t, int n, int m) { a64(b, 0xBC604800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_str_s_uxtw(a64_buf_t *b, int t, int n, int m) { a64(b, 0xBC204800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
// [Xn, #imm], unsigned scaled offsets
static inline void a64_ldr_imm(a64_buf_t *b, int t, int n, uint32_t off)   { a64(b, 0xB9400000u | ((off / 4) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_str_imm(a64_buf_t *b, int t, int n, uint32_t off)   { a64(b, 0xB9000000u | ((off / 4) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldr_x_imm(a64_buf_t *b, int t, int n, uint32_t off) { a64(b, 0xF9400000u | ((off / 8) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_str_x_imm(a64_buf_t *b, int t, int n, uint32_t off) { a64(b, 0xF9000000u | ((off / 8) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldrb_imm(a64_buf_t *b, int t, int n, uint32_t off)  { a64(b, 0x39400000u | (off << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_strb_imm(a64_buf_t *b, int t, int n, uint32_t off)  { a64(b, 0x39000000u | (off << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_ldr_s_imm(a64_buf_t *b, int t, int n, uint32_t off) { a64(b, 0xBD400000u | ((off / 4) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
static inline void a64_str_s_imm(a64_buf_t *b, int t, int n, uint32_t off) { a64(b, 0xBD000000u | ((off / 4) << 10) | ((uint32_t)n << 5) | (uint32_t)t); }
// X-register [Xn, Xm, LSL #3]
static inline void a64_ldr_x_lsl3(a64_buf_t *b, int t, int n, int m)       { a64(b, 0xF8607800u | ((uint32_t)m << 16) | ((uint32_t)n << 5) | (uint32_t)t); }
// Pairs
static inline void a64_stp_w(a64_buf_t *b, int t1, int t2, int n, int off)   { a64(b, 0x29000000u | (((uint32_t)(off / 4) & 0x7F) << 15) | ((uint32_t)t2 << 10) | ((uint32_t)n << 5) | (uint32_t)t1); }
static inline void a64_ldp_w(a64_buf_t *b, int t1, int t2, int n, int off)   { a64(b, 0x29400000u | (((uint32_t)(off / 4) & 0x7F) << 15) | ((uint32_t)t2 << 10) | ((uint32_t)n << 5) | (uint32_t)t1); }
static inline void a64_stp_x_pre(a64_buf_t *b, int t1, int t2, int n, int off) { a64(b, 0xA9800000u | (((uint32_t)(off / 8) & 0x7F) << 15) | ((uint32_t)t2 << 10) | ((uint32_t)n << 5) | (uint32_t)t1); }
static inline void a64_ldp_x_post(a64_buf_t *b, int t1, int t2, int n, int off) { a64(b, 0xA8C00000u | (((uint32_t)(off / 8) & 0x7F) << 15) | ((uint32_t)t2 << 10) | ((uint32_t)n << 5) | (uint32_t)t1); }

// --- Floating point, single precision

#define A64_FRRR(op, d, n, m) ((op) | ((uint32_t)(m) << 16) | ((uint32_t)(n) << 5) | (uint32_t)(d))
static inline void a64_fadd(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_FRRR(0x1E202800u, d, n, m)); }
static inline void a64_fsub(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_FRRR(0x1E203800u, d, n, m)); }
static inline void a64_fmul(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_FRRR(0x1E200800u, d, n, m)); }
static inline void a64_fdiv(a64_buf_t *b, int d, int n, int m)  { a64(b, A64_FRRR(0x1E201800u, d, n, m)); }
static inline void a64_fnmul(a64_buf_t *b, int d, int n, int m) { a64(b, A64_FRRR(0x1E208800u, d, n, m)); }
// d = a + n*m, a - n*m, -a - n*m, -a + n*m (fused)
static inline void a64_fmadd(a64_buf_t *b, int d, int n, int m, int a)  { a64(b, 0x1F000000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fmsub(a64_buf_t *b, int d, int n, int m, int a)  { a64(b, 0x1F008000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fnmadd(a64_buf_t *b, int d, int n, int m, int a) { a64(b, 0x1F200000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fnmsub(a64_buf_t *b, int d, int n, int m, int a) { a64(b, 0x1F208000u | ((uint32_t)m << 16) | ((uint32_t)a << 10) | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fabs(a64_buf_t *b, int d, int n)   { a64(b, 0x1E20C000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fneg(a64_buf_t *b, int d, int n)   { a64(b, 0x1E214000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fsqrt(a64_buf_t *b, int d, int n)  { a64(b, 0x1E21C000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fcmp(a64_buf_t *b, int n, int m)   { a64(b, 0x1E202000u | ((uint32_t)m << 16) | ((uint32_t)n << 5)); }
static inline void a64_fcmp0(a64_buf_t *b, int n)         { a64(b, 0x1E202008u | ((uint32_t)n << 5)); }
static inline void a64_fcvtzs(a64_buf_t *b, int d, int n) { a64(b, 0x1E380000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fcvtzu(a64_buf_t *b, int d, int n) { a64(b, 0x1E390000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fcvtns(a64_buf_t *b, int d, int n) { a64(b, 0x1E200000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fcvtnu(a64_buf_t *b, int d, int n) { a64(b, 0x1E210000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_scvtf(a64_buf_t *b, int d, int n)  { a64(b, 0x1E220000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_ucvtf(a64_buf_t *b, int d, int n)  { a64(b, 0x1E230000u | ((uint32_t)n << 5) | (uint32_t)d); }
static inline void a64_fmov_ws(a64_buf_t *b, int d, int n) { a64(b, 0x1E260000u | ((uint32_t)n << 5) | (uint32_t)d); }   // W <- S
static inline void a64_fmov_sw(a64_buf_t *b, int d, int n) { a64(b, 0x1E270000u | ((uint32_t)n << 5) | (uint32_t)d); }   // S <- W

#endif
