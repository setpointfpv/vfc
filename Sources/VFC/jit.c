// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// A JIT from Thumb-2 to AArch64, for Apple silicon.
//
// Firmware code in flash is translated a basic block at a time and cached;
// nothing in flash can change, so a translation stays good for the life of
// the image. Guest and host are both ARM, which keeps translation close to
// one-for-one:
//
// - Guest r0-r7 live in callee-saved x19-x26 and r8-lr in caller-saved
//   x9-x15, for as long as generated code runs. x28 holds the vfc_t and x27
//   the RAM buffer. x0-x8, x16 and x17 are scratch.
// - The guest's condition flags live in the host's NZCV. The two agree on
//   what C means for subtraction and on the condition code numbering.
// - Floating point registers stay in the vfc_t, loaded per instruction.
// - Loads and stores check inline, without touching the flags, whether the
//   address is in RAM (flash on a cold path), and call into C for anything
//   else, which is how the mailbox is reached.
// - Anything the translator doesn't handle, and any fast path that doesn't
//   apply (an unusual branch target, a mailbox access inside an IT block),
//   leaves generated code at that exact instruction and lets the interpreter
//   execute it. Results are identical to the interpreter's, instruction for
//   instruction, including the instruction count.

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vfc_internal.h"

#if defined(__aarch64__) && defined(__APPLE__)

#include <libkern/OSCacheControl.h>
#include <pthread.h>
#include <sys/mman.h>

#include "jit_emit.h"

#define CACHE_INSNS (8u << 20)          // 32 MB of generated code
#define MAX_BLOCK 48                    // guest instructions per block
#define HOT_CAP 8192
#define COLD_CAP 16384
#define MAX_FIXUPS 2048

enum { EXIT_NORMAL = 0, EXIT_INTERPRET = 1, EXIT_BUDGET = 2, EXIT_WFI = 3, EXIT_STOP = 4 };

static const uint8_t G[15] = { 19, 20, 21, 22, 23, 24, 25, 26, 9, 10, 11, 12, 13, 14, 15 };
enum { CTX = 28, RAMB = 27, X16 = 16, X17 = 17 };

#define OFF(field) ((uint32_t)offsetof(vfc_t, field))
#define OFF_R(i) (OFF(cpu.r) + 4u * (uint32_t)(i))
#define OFF_S(i) (OFF(cpu.s) + 4u * (uint32_t)(i))

_Static_assert(offsetof(vfc_t, cpu.r) == 0, "guest registers first");
_Static_assert(offsetof(vfc_t, jitBudget) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, jitTable) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, jitFlash) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, jitRam) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, jitLoad) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, jitStore) % 8 == 0, "8-aligned for LDR X");
_Static_assert(offsetof(vfc_t, stopReason) < 4096, "within LDR W range");
_Static_assert(offsetof(vfc_t, timeRegs) < 4096, "within ADD immediate range");

struct vfc_jit {
    uint32_t *cache;
    uint32_t used;
    uint32_t enter, exit, dispatch;     // shared code, as cache indices
    void **table;
    uint8_t *untranslatable;            // per flash halfword: the first instruction isn't handled
    bool noChain;                       // debugging: every block returns to C
    // Branches that go through the dispatcher only because their target had
    // no translation yet; pointed straight at it once it has.
    struct { uint32_t at, target; } *pending;
    uint32_t npending, pendingCap;
    // Scratch for translating one block.
    uint32_t hot[HOT_CAP], cold[COLD_CAP], assembled[HOT_CAP + COLD_CAP];
};

// --- Helpers generated code calls for anything but RAM and flash

static uint32_t jit_load(vfc_t *vfc, uint32_t address, uint32_t size)
{
    return vfc_bus_read(vfc, address, (int)size);
}

static void jit_store(vfc_t *vfc, uint32_t address, uint32_t value, uint32_t size)
{
    vfc_bus_write(vfc, address, value, (int)size);
}

// --- Assembling one block

typedef struct {
    uint8_t fromCold;
    uint8_t toKind;         // 0 hot, 1 cold, 2 cache
    uint32_t at, to;
} fixup_t;

typedef struct {
    uint32_t coldAt;        // the ADD whose immediate is patched
    int k;
    bool after;             // the instruction completed before the exit
} addback_t;

typedef struct {
    vfc_t *vfc;
    struct vfc_jit *jit;
    a64_buf_t hot, cold;
    fixup_t fixups[MAX_FIXUPS];
    int nfixups;
    addback_t addbacks[MAX_FIXUPS];
    int naddbacks;
    struct { uint8_t fromCold; uint32_t at, target; } links[MAX_FIXUPS];
    int nlinks;
    uint32_t startPc;
    uint32_t pc;            // this instruction
    uint32_t size;          // its length
    int k;                  // its index in the block
    bool inIT;
    uint8_t itstate;        // EPSR.IT as it stands before this instruction
    uint32_t lenPatchAt;
    bool failed;
} tr_t;

static void fixup(tr_t *t, bool fromCold, uint32_t at, int toKind, uint32_t to)
{
    if (t->nfixups >= MAX_FIXUPS) {
        t->failed = true;
        return;
    }
    t->fixups[t->nfixups++] = (fixup_t){ (uint8_t)fromCold, (uint8_t)toKind, at, to };
}

// A branch instruction (with a zero offset) whose target is filled in later.
static void branch_hot_to_cold(tr_t *t, uint32_t insn, uint32_t coldTarget)
{
    a64(&t->hot, insn);
    fixup(t, false, t->hot.count - 1, 1, coldTarget);
}

static void branch_cold_to_hot(tr_t *t, uint32_t insn, uint32_t hotTarget)
{
    a64(&t->cold, insn);
    fixup(t, true, t->cold.count - 1, 0, hotTarget);
}

static void branch_to_cache(tr_t *t, bool cold, uint32_t insn, uint32_t cacheTarget)
{
    a64_buf_t *b = cold ? &t->cold : &t->hot;
    a64(b, insn);
    fixup(t, cold, b->count - 1, 2, cacheTarget);
}

static uint32_t set_offset(uint32_t insn, int32_t offset)
{
    if ((insn & 0x7C000000u) == 0x14000000u) {
        return (insn & 0xFC000000u) | ((uint32_t)offset & 0x03FFFFFFu);
    }
    if ((insn & 0xFF000010u) == 0x54000000u || (insn & 0x7E000000u) == 0x34000000u) {
        return (insn & 0xFF00001Fu) | (((uint32_t)offset & 0x7FFFF) << 5);
    }
    if ((insn & 0x7E000000u) == 0x36000000u) {
        return (insn & 0xFFF8001Fu) | (((uint32_t)offset & 0x3FFF) << 5);
    }
    return insn;
}

// Leaves generated code: pc and exit kind for the C loop. `addback` says how
// much of the block's instruction budget to return: -1 none (not yet taken),
// 0 when this instruction has not run, 1 when it has.
static uint32_t exit_stub(tr_t *t, uint32_t pc, int kind, int addback, bool setIT)
{
    a64_buf_t *c = &t->cold;
    const uint32_t start = c->count;
    if (addback >= 0) {
        a64_ldr_x_imm(c, X16, CTX, OFF(jitBudget));
        if (t->naddbacks >= MAX_FIXUPS) {
            t->failed = true;
        } else {
            t->addbacks[t->naddbacks++] = (addback_t){ c->count, t->k, addback == 1 };
        }
        a64_add_x_imm(c, X16, X16, 0);
        a64_str_x_imm(c, X16, CTX, OFF(jitBudget));
    }
    if (setIT) {
        a64_movz(c, X16, t->itstate, 0);
        a64_strb_imm(c, X16, CTX, OFF(cpu.itstate));
    }
    a64_mov32(c, 0, pc);
    a64_movz(c, 1, (uint32_t)kind, 0);
    branch_to_cache(t, true, a64_b(0), t->jit->exit);
    return start;
}

// Hands this instruction to the interpreter.
static uint32_t interpret_stub(tr_t *t)
{
    return exit_stub(t, t->pc, EXIT_INTERPRET, 0, t->inIT);
}

// Continues at a guest address: straight into its translation when there is
// one already, through the dispatcher otherwise.
static void goto_pc(tr_t *t, a64_buf_t *b, uint32_t target)
{
    const uint32_t offset = target - VFC_FLASH_BASE;
    const bool cold = b == &t->cold;
    if (t->jit->noChain) {
        a64_mov32(b, 0, target);
        a64_movz(b, 1, EXIT_NORMAL, 0);
        branch_to_cache(t, cold, a64_b(0), t->jit->exit);
        return;
    }
    if (offset < VFC_FLASH_SIZE && (offset & 1) == 0 && t->jit->table[offset / 2]) {
        const uint32_t cacheIndex = (uint32_t)((uint32_t *)t->jit->table[offset / 2] - t->jit->cache);
        branch_to_cache(t, cold, a64_b(0), cacheIndex);
        return;
    }
    a64_mov32(b, 0, target);
    branch_to_cache(t, cold, a64_b(0), t->jit->dispatch);
    if (offset < VFC_FLASH_SIZE && (offset & 1) == 0 && t->nlinks < MAX_FIXUPS) {
        t->links[t->nlinks].fromCold = cold;
        t->links[t->nlinks].at = b->count - 1;
        t->links[t->nlinks].target = target;
        t->nlinks++;
    }
}

// An indirect branch: the target is in w0.
static void goto_w0(tr_t *t)
{
    if (t->jit->noChain) {
        a64_movz(&t->hot, 1, EXIT_NORMAL, 0);
        branch_to_cache(t, false, a64_b(0), t->jit->exit);
        return;
    }
    // The table lookup, here rather than in one shared stub, so that each
    // indirect branch has its own `br` for the predictor to learn.
    a64_buf_t *h = &t->hot;
    a64_movz(h, X16, 0x0800, 1);
    a64_sub(h, X16, 0, X16, SH_LSL, 0);
    a64_lsr_imm(h, X17, X16, 21);
    const uint32_t miss1 = h->count;
    a64(h, a64_cbnz(X17, 0));
    a64_lsr_imm(h, X16, X16, 1);
    a64_ldr_x_imm(h, X17, CTX, OFF(jitTable));
    a64_ldr_x_lsl3(h, X16, X17, X16);
    const uint32_t miss2 = h->count;
    a64(h, a64_cbz_x(X16, 0));
    a64_br(h, X16);
    a64_patch(h->code, miss1, h->count);
    a64_patch(h->code, miss2, h->count);
    branch_to_cache(t, false, a64_b(0), t->jit->dispatch);
}

// --- Operands

static inline uint32_t flash_word(tr_t *t, uint32_t address, bool *ok)
{
    const uint32_t offset = address - VFC_FLASH_BASE;
    if (offset > t->vfc->flashUsed - 4 || offset >= VFC_FLASH_SIZE) {
        *ok = false;
        return 0;
    }
    uint32_t v;
    memcpy(&v, t->vfc->flash + offset, 4);
    *ok = true;
    return v;
}

// The host register holding guest register n; the PC is materialised.
static int rn(tr_t *t, int n, int scratch)
{
    if (n == 15) {
        a64_mov32(&t->hot, scratch, t->pc + 4);
        return scratch;
    }
    return G[n];
}

// --- Flags

// N and Z from `res`, C and V kept.
static void flags_nz(tr_t *t, int res)
{
    a64_buf_t *b = &t->hot;
    a64_mrs_nzcv(b, X16);
    a64_ands(b, A64_ZR, res, res, SH_LSL, 0);
    a64_mrs_nzcv(b, X17);
    a64_and_imm(b, X16, X16, 0x30000000u);
    a64_orr(b, X17, X17, X16, SH_LSL, 0);
    a64_msr_nzcv(b, X17);
}

// N and Z from `res`, C from bit 0 of `carry`, V kept.
static void flags_nzc(tr_t *t, int res, int carry)
{
    a64_buf_t *b = &t->hot;
    a64_mrs_nzcv(b, X16);
    a64_ands(b, A64_ZR, res, res, SH_LSL, 0);
    a64_mrs_nzcv(b, X17);
    a64_and_imm(b, X16, X16, 0x10000000u);
    a64_orr(b, X17, X17, X16, SH_LSL, 0);
    a64_bfi(b, X17, carry, 29, 1);
    a64_msr_nzcv(b, X17);
}

// --- Memory

// Loads `size` bytes from the guest address in `addr` into `dst`, or stores
// `dst` there. Fast path: RAM, inline. Cold: flash for loads, then the C
// helper, which reaches the mailbox and faults on anything unmapped.
static void access(tr_t *t, bool store, int size, bool sign, int addr, int dst)
{
    a64_buf_t *h = &t->hot, *c = &t->cold;
    a64_movz(h, X16, 0x2000, 1);
    a64_sub(h, X16, addr, X16, SH_LSL, 0);
    a64_lsr_imm(h, X17, X16, 19);
    branch_hot_to_cold(t, a64_cbnz(X17, 0), c->count);
    if (store) {
        switch (size) {
        case 1: a64_strb_uxtw(h, dst, RAMB, X16); break;
        case 2: a64_strh_uxtw(h, dst, RAMB, X16); break;
        default: a64_str_uxtw(h, dst, RAMB, X16); break;
        }
    } else {
        switch (size) {
        case 1: if (sign) a64_ldrsb_uxtw(h, dst, RAMB, X16); else a64_ldrb_uxtw(h, dst, RAMB, X16); break;
        case 2: if (sign) a64_ldrsh_uxtw(h, dst, RAMB, X16); else a64_ldrh_uxtw(h, dst, RAMB, X16); break;
        default: a64_ldr_uxtw(h, dst, RAMB, X16); break;
        }
    }
    const uint32_t back = h->count;

    // Cold: flash.
    if (!store) {
        a64_movz(c, X16, 0x0800, 1);
        a64_sub(c, X16, addr, X16, SH_LSL, 0);
        a64_lsr_imm(c, X17, X16, 21);
        const uint32_t toMmio = c->count;
        a64(c, a64_cbnz(X17, 0));
        a64_ldr_x_imm(c, X17, CTX, OFF(jitFlash));
        switch (size) {
        case 1: if (sign) a64_ldrsb_uxtw(c, dst, X17, X16); else a64_ldrb_uxtw(c, dst, X17, X16); break;
        case 2: if (sign) a64_ldrsh_uxtw(c, dst, X17, X16); else a64_ldrh_uxtw(c, dst, X17, X16); break;
        default: a64_ldr_uxtw(c, dst, X17, X16); break;
        }
        branch_cold_to_hot(t, a64_b(0), back);
        a64_patch(c->code, toMmio, c->count);
    }

    // Cold: the mailbox's time registers, read directly, since the firmware
    // asks the time dozens of times a loop.
    if (!store && size == 4) {
        a64_mov32(c, X16, VFC_MAILBOX_BASE + VFC_MBX_TIME_REGS);
        a64_sub(c, X16, addr, X16, SH_LSL, 0);
        a64_and_imm(c, X17, X16, ~(uint32_t)0xC);          // 0, 4, 8 or 12 only
        const uint32_t notTime = c->count;
        a64(c, a64_cbnz(X17, 0));
        a64_add_x_imm(c, X17, CTX, OFF(timeRegs));
        a64_ldr_uxtw(c, dst, X17, X16);
        branch_cold_to_hot(t, a64_b(0), back);
        a64_patch(c->code, notTime, c->count);
    }

    // Cold: anything else. Inside an IT block the interpreter takes it, since
    // the IT state would otherwise have to be rebuilt mid-block.
    if (t->inIT) {
        const uint32_t here = c->count;
        a64(c, a64_b(0));
        const uint32_t stub = interpret_stub(t);
        a64_patch(c->code, here, stub);
        return;
    }
    a64_mov32(c, X16, t->pc);
    a64_str_imm(c, X16, CTX, OFF_R(15));
    a64_mov(c, X16, addr);
    if (store) a64_mov(c, X17, dst);
    a64_stp_w(c, 9, 10, CTX, (int)OFF_R(8));
    a64_stp_w(c, 11, 12, CTX, (int)OFF_R(10));
    a64_stp_w(c, 13, 14, CTX, (int)OFF_R(12));
    a64_str_imm(c, 15, CTX, OFF_R(14));
    a64_mrs_nzcv(c, 3);
    a64_str_imm(c, 3, CTX, OFF(jitSavedNzcv));
    a64_mov(c, 1, X16);
    if (store) {
        a64_mov(c, 2, X17);
        a64_movz(c, 3, (uint32_t)size, 0);
        a64(c, 0xAA0003E0u | (CTX << 16));          // mov x0, x28
        a64_ldr_x_imm(c, X16, CTX, OFF(jitStore));
    } else {
        a64_movz(c, 2, (uint32_t)size, 0);
        a64(c, 0xAA0003E0u | (CTX << 16));
        a64_ldr_x_imm(c, X16, CTX, OFF(jitLoad));
    }
    a64_blr(c, X16);
    a64_ldp_w(c, 9, 10, CTX, (int)OFF_R(8));
    a64_ldp_w(c, 11, 12, CTX, (int)OFF_R(10));
    a64_ldp_w(c, 13, 14, CTX, (int)OFF_R(12));
    a64_ldr_imm(c, 15, CTX, OFF_R(14));
    a64_ldr_imm(c, X16, CTX, OFF(jitSavedNzcv));
    a64_msr_nzcv(c, X16);
    if (!store) {
        if (sign && size == 1) a64_sxtb(c, dst, 0);
        else if (sign && size == 2) a64_sxth(c, dst, 0);
        else a64_mov(c, dst, 0);
    }
    // A fault, or the firmware asking for a reset, stops the core.
    a64_ldrb_imm(c, X16, CTX, OFF(stopRequested));
    const uint32_t toStop = c->count;
    a64(c, a64_cbnz(X16, 0));
    branch_cold_to_hot(t, a64_b(0), back);
    a64_patch(c->code, toStop, c->count);
    a64_ldr_imm(c, X16, CTX, OFF(stopReason));
    a64_sub_imm(c, X16, X16, VFC_STOP_FAULT, false);
    const uint32_t toFault = c->count;
    a64(c, a64_cbz(X16, 0));
    exit_stub(t, t->pc + t->size, EXIT_STOP, 1, false);
    a64_patch(c->code, toFault, c->count);
    exit_stub(t, t->pc, EXIT_STOP, 1, false);
}

// Checks that [addr, addr + bytes) is all in RAM, leaving the offset in w16;
// otherwise the interpreter takes the instruction.
static void ram_range(tr_t *t, int addr, uint32_t bytes)
{
    a64_buf_t *h = &t->hot;
    a64_movz(h, X16, 0x2000, 1);
    a64_sub(h, X16, addr, X16, SH_LSL, 0);
    a64_add_imm(h, X17, X16, bytes - 1, false);
    a64_orr(h, X17, X17, X16, SH_LSL, 0);
    a64_lsr_imm(h, X17, X17, 19);
    branch_hot_to_cold(t, a64_cbnz(X17, 0), interpret_stub(t));
}

// Word at [x27 + w16 + off] to or from a W register.
static void ram_word(tr_t *t, bool store, int reg, uint32_t off)
{
    a64_buf_t *h = &t->hot;
    if (off) {
        a64_add_imm(h, X17, X16, off, false);
        if (store) a64_str_uxtw(h, reg, RAMB, X17); else a64_ldr_uxtw(h, reg, RAMB, X17);
    } else {
        if (store) a64_str_uxtw(h, reg, RAMB, X16); else a64_ldr_uxtw(h, reg, RAMB, X16);
    }
}

// --- Shifts

enum { SHIFT_LSL = 0, SHIFT_LSR = 1, SHIFT_ASR = 2, SHIFT_ROR = 3, SHIFT_RRX = 4 };

static void decode_imm_shift(unsigned type, unsigned imm5, int *outType, unsigned *amount)
{
    switch (type) {
    case 0: *outType = SHIFT_LSL; *amount = imm5; break;
    case 1: *outType = SHIFT_LSR; *amount = imm5 ? imm5 : 32; break;
    case 2: *outType = SHIFT_ASR; *amount = imm5 ? imm5 : 32; break;
    default: if (imm5 == 0) { *outType = SHIFT_RRX; *amount = 1; } else { *outType = SHIFT_ROR; *amount = imm5; } break;
    }
}

// dst = src shifted by an immediate. False for RRX.
static bool shift_imm(tr_t *t, int dst, int src, int type, unsigned amount)
{
    a64_buf_t *h = &t->hot;
    switch (type) {
    case SHIFT_LSL: if (amount) a64_lsl_imm(h, dst, src, (int)amount); else if (dst != src) a64_mov(h, dst, src); return true;
    case SHIFT_LSR: if (amount >= 32) a64_movz(h, dst, 0, 0); else a64_lsr_imm(h, dst, src, (int)amount); return true;
    case SHIFT_ASR: a64_asr_imm(h, dst, src, amount >= 32 ? 31 : (int)amount); return true;
    case SHIFT_ROR: a64_ror_imm(h, dst, src, (int)(amount & 31)); return true;
    default: return false;
    }
}

// The shifter's carry out into bit 0 of `dst`; false when C is unchanged.
static bool shift_carry(tr_t *t, int dst, int src, int type, unsigned amount)
{
    a64_buf_t *h = &t->hot;
    switch (type) {
    case SHIFT_LSL: if (amount == 0) return false; a64_ubfx(h, dst, src, 32 - (int)amount, 1); return true;
    case SHIFT_LSR: case SHIFT_ASR: a64_ubfx(h, dst, src, amount >= 32 ? 31 : (int)amount - 1, 1); return true;
    case SHIFT_ROR: a64_ubfx(h, dst, src, ((int)amount - 1) & 31, 1); return true;
    default: return false;
    }
}

// Shift by register: by the bottom byte of `amount`, where 32 and more shift
// everything out (AArch64 would take the amount modulo 32). With flags, C is
// the last bit shifted out, or kept for an amount of 0; V is kept.
static void shift_register(tr_t *t, int dst, int src, int amount, int type, bool setflags)
{
    a64_buf_t *h = &t->hot;
    if (!setflags) {
        a64_and_imm(h, 1, amount, 0xFF);
        if (type == SHIFT_ROR) { a64_rorv(h, dst, src, 1); return; }   // modulo 32 anyway
        // mask = all ones when the amount is under 32.
        a64_lsr_imm(h, 3, 1, 5);
        a64_sub_imm(h, 3, 3, 1, false);
        a64_asr_imm(h, 3, 3, 31);
        switch (type) {
        case SHIFT_LSL: a64_lslv(h, 2, src, 1); a64_and(h, dst, 2, 3, SH_LSL, 0); break;
        case SHIFT_LSR: a64_lsrv(h, 2, src, 1); a64_and(h, dst, 2, 3, SH_LSL, 0); break;
        default:
            a64_asrv(h, 2, src, 1);
            a64_asr_imm(h, 4, src, 31);
            a64_and(h, 2, 2, 3, SH_LSL, 0);
            a64_bic(h, 4, 4, 3, SH_LSL, 0);
            a64_orr(h, dst, 2, 4, SH_LSL, 0);
            break;
        }
        return;
    }
    // In 64 bits, with the amount capped at 33, the result and the carry
    // sit side by side.
    a64_mrs_nzcv(h, 7);
    a64_and_imm(h, 1, amount, 0xFF);
    if (type == SHIFT_ROR) {
        a64_rorv(h, 2, src, 1);
        a64_lsr_imm(h, 6, 2, 31);
    } else {
        a64_movz(h, 3, 33, 0);
        a64_subs_imm(h, A64_ZR, 1, 33);
        a64_csel(h, 1, 1, 3, COND_CC);
        if (type == SHIFT_LSL) {
            a64_mov(h, 2, src);
            a64_lslv_x(h, 2, 2, 1);
            a64_ubfm_x(h, 6, 2, 32, 32);                // carry: bit 32
        } else {
            a64_ubfm_x(h, 2, src, 32, 31);              // src << 32
            if (type == SHIFT_LSR) a64_lsrv_x(h, 2, 2, 1); else a64_asrv_x(h, 2, 2, 1);
            a64_ubfx(h, 6, 2, 31, 1);                   // carry: bit 31
            a64_ubfm_x(h, 2, 2, 32, 63);                // the result: the top half
        }
    }
    a64_ubfx(h, 5, 7, 29, 1);
    a64_subs_imm(h, A64_ZR, 1, 0);
    a64_csel(h, 6, 5, 6, COND_EQ);
    a64_msr_nzcv(h, 7);
    a64_mov(h, dst, 2);
    flags_nzc(t, dst, 6);
}

static uint32_t thumb_expand_imm(uint32_t imm12, int *carry)
{
    *carry = -1;
    if ((imm12 >> 10) == 0) {
        const uint32_t imm8 = imm12 & 0xFF;
        switch ((imm12 >> 8) & 3) {
        case 0: return imm8;
        case 1: return (imm8 << 16) | imm8;
        case 2: return (imm8 << 24) | (imm8 << 8);
        default: return imm8 * 0x01010101u;
        }
    }
    const uint32_t unrotated = 0x80 | (imm12 & 0x7F);
    const unsigned r = (imm12 >> 7) & 0x1F;
    const uint32_t result = (unrotated >> r) | (unrotated << ((32 - r) & 31));
    *carry = (int)(result >> 31);
    return result;
}

// --- Data processing shared by the 32-bit forms

// op as the interpreter's (inst[24:21]). `operand` holds the second operand;
// `carry` is -1 (unchanged), 0/1 (constant) or 2 (bit 0 of w6).
static bool data_processing(tr_t *t, unsigned op, bool setflags, unsigned d, unsigned n, int operand, int carry)
{
    a64_buf_t *h = &t->hot;
    const bool noWrite = (op == 0x0 || op == 0x4 || op == 0x8 || op == 0xD) && d == 15;
    if (d == 15 && !(noWrite && setflags)) {
        return false;                           // a PC write: unpredictable here
    }
    if (d == 13 || n == 13) {
        // Fine: SP is an ordinary register here.
    }
    const int rd = noWrite ? 5 : G[d];
    const int left = n == 15 && op != 0x2 && op != 0x3 ? rn(t, 15, 4) : (n == 15 ? A64_ZR : G[n]);
    bool logical = true;
    // Constant carries are materialised before the operation writes rd.
    if (setflags && carry >= 0 && carry <= 1) {
        a64_movz(h, 6, (uint32_t)carry, 0);
    }
    switch (op) {
    case 0x0: a64_and(h, rd, left, operand, SH_LSL, 0); break;
    case 0x1: a64_bic(h, rd, left, operand, SH_LSL, 0); break;
    case 0x2: a64_orr(h, rd, left, operand, SH_LSL, 0); break;     // ORR, or MOV with n == 15 (left = ZR)
    case 0x3: a64_orn(h, rd, left, operand, SH_LSL, 0); break;     // ORN, or MVN
    case 0x4: a64_eor(h, rd, left, operand, SH_LSL, 0); break;
    case 0x8:
        logical = false;
        if (setflags) a64_adds(h, noWrite ? A64_ZR : rd, left, operand, SH_LSL, 0);
        else a64_add(h, rd, left, operand, SH_LSL, 0);
        break;
    case 0xA:
        logical = false;
        if (setflags) a64_adcs(h, rd, left, operand); else a64_adc(h, rd, left, operand);
        break;
    case 0xB:
        logical = false;
        if (setflags) a64_sbcs(h, rd, left, operand); else a64_sbc(h, rd, left, operand);
        break;
    case 0xD:
        logical = false;
        if (setflags) a64_subs(h, noWrite ? A64_ZR : rd, left, operand, SH_LSL, 0);
        else a64_sub(h, rd, left, operand, SH_LSL, 0);
        break;
    case 0xE:
        logical = false;
        if (setflags) a64_subs(h, rd, operand, left, SH_LSL, 0); else a64_sub(h, rd, operand, left, SH_LSL, 0);
        break;
    default:
        return false;
    }
    if (setflags && logical) {
        if (carry >= 0) flags_nzc(t, rd, 6); else flags_nz(t, rd);
    }
    return true;
}

// --- Branch helpers

// Ends the block with a conditional branch: `cond` taken goes to `target`.
static void conditional_goto(tr_t *t, int cond, uint32_t target)
{
    const uint32_t cold = t->cold.count;
    branch_hot_to_cold(t, a64_bcond(cond, 0), cold);
    goto_pc(t, &t->cold, target);
    (void)cold;
    goto_pc(t, &t->hot, t->pc + t->size);
}

// BX-style: w0 holds the target; bit 0 clear means ARM state, which the
// interpreter turns into the fault.
static void bx_w0(tr_t *t)
{
    branch_hot_to_cold(t, a64_tbz(0, 0, 0), interpret_stub(t));
    a64_and_imm(&t->hot, 0, 0, 0xFFFFFFFEu);
    goto_w0(t);
}

// --- Translation results

enum { T_OK = 0, T_END = 1, T_UNSUPPORTED = 2 };

// --- 16-bit instructions

static int translate16(tr_t *t, uint32_t hw)
{
    a64_buf_t *h = &t->hot;
    const bool setflags = !t->inIT;

    switch (hw >> 11) {
    case 0x00: case 0x01: case 0x02: {                  // LSL, LSR, ASR (immediate)
        const unsigned d = hw & 7, m = (hw >> 3) & 7, imm5 = (hw >> 6) & 0x1F;
        int type;
        unsigned amount;
        decode_imm_shift(hw >> 11, imm5, &type, &amount);
        const bool carry = setflags && shift_carry(t, 6, G[m], type, amount);
        shift_imm(t, G[d], G[m], type, amount);
        if (setflags) {
            if (carry) flags_nzc(t, G[d], 6); else flags_nz(t, G[d]);
        }
        return T_OK;
    }
    case 0x03: {                                        // ADD/SUB register or imm3
        const unsigned d = hw & 7, n = (hw >> 3) & 7, x = (hw >> 6) & 7;
        const bool sub = hw & 0x200;
        if (hw & 0x400) {
            if (sub) { if (setflags) a64_subs_imm(h, G[d], G[n], x); else a64_sub_imm(h, G[d], G[n], x, false); }
            else { if (setflags) a64_adds_imm(h, G[d], G[n], x); else a64_add_imm(h, G[d], G[n], x, false); }
        } else {
            if (sub) { if (setflags) a64_subs(h, G[d], G[n], G[x], SH_LSL, 0); else a64_sub(h, G[d], G[n], G[x], SH_LSL, 0); }
            else { if (setflags) a64_adds(h, G[d], G[n], G[x], SH_LSL, 0); else a64_add(h, G[d], G[n], G[x], SH_LSL, 0); }
        }
        return T_OK;
    }
    case 0x04: {                                        // MOV immediate
        const unsigned d = (hw >> 8) & 7;
        a64_movz(h, G[d], hw & 0xFF, 0);
        if (setflags) flags_nz(t, G[d]);
        return T_OK;
    }
    case 0x05:                                          // CMP immediate
        a64_subs_imm(h, A64_ZR, G[(hw >> 8) & 7], hw & 0xFF);
        return T_OK;
    case 0x06: case 0x07: {                             // ADD/SUB 8-bit immediate
        const unsigned dn = (hw >> 8) & 7;
        if (hw & 0x800) { if (setflags) a64_subs_imm(h, G[dn], G[dn], hw & 0xFF); else a64_sub_imm(h, G[dn], G[dn], hw & 0xFF, false); }
        else { if (setflags) a64_adds_imm(h, G[dn], G[dn], hw & 0xFF); else a64_add_imm(h, G[dn], G[dn], hw & 0xFF, false); }
        return T_OK;
    }
    case 0x08: {
        if ((hw & 0x400) == 0) {                        // data processing (register)
            const unsigned dn = hw & 7, m = (hw >> 3) & 7;
            const int a = G[dn], b = G[m];
            switch ((hw >> 6) & 0xF) {
            case 0x0: a64_and(h, a, a, b, SH_LSL, 0); if (setflags) flags_nz(t, a); return T_OK;
            case 0x1: a64_eor(h, a, a, b, SH_LSL, 0); if (setflags) flags_nz(t, a); return T_OK;
            case 0x5: if (setflags) a64_adcs(h, a, a, b); else a64_adc(h, a, a, b); return T_OK;
            case 0x6: if (setflags) a64_sbcs(h, a, a, b); else a64_sbc(h, a, a, b); return T_OK;
            case 0x8: a64_and(h, 5, a, b, SH_LSL, 0); flags_nz(t, 5); return T_OK;          // TST
            case 0x9: if (setflags) a64_subs(h, a, A64_ZR, b, SH_LSL, 0); else a64_sub(h, a, A64_ZR, b, SH_LSL, 0); return T_OK;  // RSB #0
            case 0xA: a64_subs(h, A64_ZR, a, b, SH_LSL, 0); return T_OK;                     // CMP
            case 0xB: a64_adds(h, A64_ZR, a, b, SH_LSL, 0); return T_OK;                     // CMN
            case 0xC: a64_orr(h, a, a, b, SH_LSL, 0); if (setflags) flags_nz(t, a); return T_OK;
            case 0xD: a64_madd(h, a, a, b, A64_ZR); if (setflags) flags_nz(t, a); return T_OK;
            case 0xE: a64_bic(h, a, a, b, SH_LSL, 0); if (setflags) flags_nz(t, a); return T_OK;
            case 0xF: a64_orn(h, a, A64_ZR, b, SH_LSL, 0); if (setflags) flags_nz(t, a); return T_OK;
            case 0x2: shift_register(t, a, a, b, SHIFT_LSL, setflags); return T_OK;
            case 0x3: shift_register(t, a, a, b, SHIFT_LSR, setflags); return T_OK;
            case 0x4: shift_register(t, a, a, b, SHIFT_ASR, setflags); return T_OK;
            case 0x7: shift_register(t, a, a, b, SHIFT_ROR, setflags); return T_OK;
            default: return T_UNSUPPORTED;
            }
        }
        const unsigned op = (hw >> 8) & 3;
        const unsigned m = (hw >> 3) & 0xF;
        const unsigned dn = (hw & 7) | ((hw >> 4) & 8);
        switch (op) {
        case 0:                                         // ADD (register), high registers
            if (dn == 15) {
                a64_add(h, 0, rn(t, 15, 2), rn(t, (int)m, 3), SH_LSL, 0);
                a64_and_imm(h, 0, 0, 0xFFFFFFFEu);
                goto_w0(t);
                return T_END;
            }
            a64_add(h, G[dn], G[dn], rn(t, (int)m, 3), SH_LSL, 0);
            return T_OK;
        case 1:                                         // CMP, high registers
            a64_subs(h, A64_ZR, rn(t, (int)dn, 2), rn(t, (int)m, 3), SH_LSL, 0);
            return T_OK;
        case 2:                                         // MOV (register)
            if (dn == 15) {
                a64_mov(h, 0, rn(t, (int)m, 3));
                a64_and_imm(h, 0, 0, 0xFFFFFFFEu);
                goto_w0(t);
                return T_END;
            }
            a64_mov(h, G[dn], rn(t, (int)m, 3));
            return T_OK;
        default:                                        // BX, BLX
            a64_mov(h, 0, rn(t, (int)m, 3));
            // The target's Thumb bit is checked before LR changes, so the
            // interpreter can take the instruction again from scratch.
            branch_hot_to_cold(t, a64_tbz(0, 0, 0), interpret_stub(t));
            if (hw & 0x80) a64_mov32(h, G[14], (t->pc + 2) | 1);
            a64_and_imm(h, 0, 0, 0xFFFFFFFEu);
            goto_w0(t);
            return T_END;
        }
    }
    case 0x09: {                                        // LDR (literal)
        bool ok;
        const uint32_t v = flash_word(t, ((t->pc + 4) & ~3u) + ((hw & 0xFF) << 2), &ok);
        if (!ok) return T_UNSUPPORTED;
        a64_mov32(h, G[(hw >> 8) & 7], v);
        return T_OK;
    }
    case 0x0A: case 0x0B: {                             // load/store register offset
        const unsigned tt = hw & 7, n = (hw >> 3) & 7, m = (hw >> 6) & 7;
        a64_add(h, 0, G[n], G[m], SH_LSL, 0);
        switch ((hw >> 9) & 7) {
        case 0: access(t, true, 4, false, 0, G[tt]); break;
        case 1: access(t, true, 2, false, 0, G[tt]); break;
        case 2: access(t, true, 1, false, 0, G[tt]); break;
        case 3: access(t, false, 1, true, 0, G[tt]); break;
        case 4: access(t, false, 4, false, 0, G[tt]); break;
        case 5: access(t, false, 2, false, 0, G[tt]); break;
        case 6: access(t, false, 1, false, 0, G[tt]); break;
        case 7: access(t, false, 2, true, 0, G[tt]); break;
        }
        return T_OK;
    }
    case 0x0C: case 0x0D: case 0x0E: case 0x0F: case 0x10: case 0x11: {   // imm5 loads and stores
        const unsigned tt = hw & 7, n = (hw >> 3) & 7, imm5 = (hw >> 6) & 0x1F;
        const bool isLoad = hw & 0x800;
        int size;
        uint32_t offset;
        switch (hw >> 11) {
        case 0x0C: case 0x0D: size = 4; offset = imm5 << 2; break;
        case 0x0E: case 0x0F: size = 1; offset = imm5; break;
        default: size = 2; offset = imm5 << 1; break;
        }
        a64_add_imm(h, 0, G[n], offset, false);
        access(t, !isLoad, size, false, 0, G[tt]);
        return T_OK;
    }
    case 0x12: case 0x13: {                             // SP-relative
        const unsigned tt = (hw >> 8) & 7;
        a64_add_imm(h, 0, G[13], (hw & 0xFF) << 2, false);
        access(t, !(hw & 0x800), 4, false, 0, G[tt]);
        return T_OK;
    }
    case 0x14:                                          // ADR
        a64_mov32(h, G[(hw >> 8) & 7], ((t->pc + 4) & ~3u) + ((hw & 0xFF) << 2));
        return T_OK;
    case 0x15:                                          // ADD Rd, SP, #imm
        a64_add_imm(h, G[(hw >> 8) & 7], G[13], (hw & 0xFF) << 2, false);
        return T_OK;
    case 0x16: case 0x17:
        if ((hw & 0xFF00) == 0xB000) {                  // ADD/SUB SP, SP, #imm
            const uint32_t imm = (hw & 0x7F) << 2;
            if (hw & 0x80) a64_sub_imm(h, G[13], G[13], imm, false); else a64_add_imm(h, G[13], G[13], imm, false);
            return T_OK;
        }
        if ((hw & 0xF500) == 0xB100) {                  // CBZ, CBNZ
            const unsigned n = hw & 7;
            const uint32_t imm = (((hw >> 9) & 1) << 6) | (((hw >> 3) & 0x1F) << 1);
            const uint32_t target = t->pc + 4 + imm;
            const uint32_t cold = t->cold.count;
            branch_hot_to_cold(t, (hw & 0x800) ? a64_cbnz(G[n], 0) : a64_cbz(G[n], 0), cold);
            goto_pc(t, &t->cold, target);
            goto_pc(t, &t->hot, t->pc + 2);
            return T_END;
        }
        if ((hw & 0xFF00) == 0xB200) {                  // SXTH, SXTB, UXTH, UXTB
            const unsigned d = hw & 7, m = (hw >> 3) & 7;
            switch ((hw >> 6) & 3) {
            case 0: a64_sxth(h, G[d], G[m]); break;
            case 1: a64_sxtb(h, G[d], G[m]); break;
            case 2: a64_uxth(h, G[d], G[m]); break;
            case 3: a64_uxtb(h, G[d], G[m]); break;
            }
            return T_OK;
        }
        if ((hw & 0xFE00) == 0xB400) {                  // PUSH
            const uint32_t list = (hw & 0xFF) | ((hw & 0x100) ? (1u << 14) : 0);
            const uint32_t count = (uint32_t)__builtin_popcount(list);
            a64_sub_imm(h, 0, G[13], 4 * count, false);
            ram_range(t, 0, 4 * count);
            uint32_t off = 0;
            for (int i = 0; i < 15; i++) {
                if (list & (1u << i)) {
                    ram_word(t, true, G[i], off);
                    off += 4;
                }
            }
            a64_mov(h, G[13], 0);
            return T_OK;
        }
        if ((hw & 0xFFE8) == 0xB660) {                  // CPS
            if (hw & 1) { a64_movz(h, X16, (hw & 0x10) ? 1 : 0, 0); a64_str_imm(h, X16, CTX, OFF(cpu.faultmask)); }
            if (hw & 2) { a64_movz(h, X16, (hw & 0x10) ? 1 : 0, 0); a64_str_imm(h, X16, CTX, OFF(cpu.primask)); }
            return T_OK;
        }
        if ((hw & 0xFF00) == 0xBA00) {                  // REV, REV16, REVSH
            const unsigned d = hw & 7, m = (hw >> 3) & 7;
            switch ((hw >> 6) & 3) {
            case 0: a64_rev(h, G[d], G[m]); return T_OK;
            case 1: a64_rev16(h, G[d], G[m]); return T_OK;
            case 3: a64_rev16(h, G[d], G[m]); a64_sxth(h, G[d], G[d]); return T_OK;
            default: return T_UNSUPPORTED;
            }
        }
        if ((hw & 0xFE00) == 0xBC00) {                  // POP
            const bool pc = hw & 0x100;
            const uint32_t list = hw & 0xFF;
            const uint32_t count = (uint32_t)__builtin_popcount(list) + (pc ? 1 : 0);
            ram_range(t, G[13], 4 * count);
            if (pc) {
                ram_word(t, false, 0, 4 * (count - 1));
                branch_hot_to_cold(t, a64_tbz(0, 0, 0), interpret_stub(t));
            }
            uint32_t off = 0;
            for (int i = 0; i < 8; i++) {
                if (list & (1u << i)) {
                    ram_word(t, false, G[i], off);
                    off += 4;
                }
            }
            a64_add_imm(h, G[13], G[13], 4 * count, false);
            if (pc) {
                a64_and_imm(h, 0, 0, 0xFFFFFFFEu);
                goto_w0(t);
                return T_END;
            }
            return T_OK;
        }
        if ((hw & 0xFF00) == 0xBF00) {
            if (hw & 0xF) {                             // IT: handled by the block loop
                return T_OK;
            }
            switch ((hw >> 4) & 0xF) {
            case 3:                                     // WFI: handled by the block loop
                return T_UNSUPPORTED;
            default:
                return T_OK;                            // NOP, YIELD, WFE, SEV
            }
        }
        return T_UNSUPPORTED;
    case 0x18: {                                        // STM (STMIA Rn!)
        const unsigned n = (hw >> 8) & 7;
        const uint32_t list = hw & 0xFF;
        const uint32_t count = (uint32_t)__builtin_popcount(list);
        if (list & (1u << n)) return T_UNSUPPORTED;     // stores the base: leave to the interpreter
        ram_range(t, G[n], 4 * count);
        uint32_t off = 0;
        for (int i = 0; i < 8; i++) {
            if (list & (1u << i)) {
                ram_word(t, true, G[i], off);
                off += 4;
            }
        }
        a64_add_imm(h, G[n], G[n], 4 * count, false);
        return T_OK;
    }
    case 0x19: {                                        // LDM
        const unsigned n = (hw >> 8) & 7;
        const uint32_t list = hw & 0xFF;
        const uint32_t count = (uint32_t)__builtin_popcount(list);
        a64_mov(h, 0, G[n]);
        ram_range(t, 0, 4 * count);
        uint32_t off = 0;
        for (int i = 0; i < 8; i++) {
            if (list & (1u << i)) {
                ram_word(t, false, G[i], off);
                off += 4;
            }
        }
        if (!(list & (1u << n))) a64_add_imm(h, G[n], 0, 4 * count, false);
        return T_OK;
    }
    case 0x1A: case 0x1B: {                             // B<c>
        const unsigned cond = (hw >> 8) & 0xF;
        if (cond >= 0xE) return T_UNSUPPORTED;          // UDF, SVC
        conditional_goto(t, (int)cond, t->pc + 4 + (uint32_t)((int32_t)(int8_t)(hw & 0xFF) * 2));
        return T_END;
    }
    case 0x1C: {                                        // B
        const int32_t imm = ((int32_t)((hw & 0x7FF) << 21)) >> 20;
        goto_pc(t, &t->hot, t->pc + 4 + (uint32_t)imm);
        return T_END;
    }
    default:
        return T_UNSUPPORTED;
    }
}

// --- 32-bit: loads and stores

static int translate_load_store_single(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned size = (hw1 >> 5) & 3;
    const bool isLoad = hw1 & 0x10;
    const bool sign = hw1 & 0x100;
    const unsigned n = hw1 & 0xF, tt = hw2 >> 12;
    const int bytes = size == 0 ? 1 : size == 1 ? 2 : 4;
    if (size == 3) return T_UNSUPPORTED;
    if (isLoad && tt == 15 && size != 2) return T_OK;   // PLD, PLI: hints
    if (!isLoad && tt == 15) return T_UNSUPPORTED;

    if (n == 15) {                                      // literal
        if (!isLoad) return T_UNSUPPORTED;
        const uint32_t imm12 = hw2 & 0xFFF;
        const uint32_t base = (t->pc + 4) & ~3u;
        const uint32_t address = (hw1 & 0x80) ? base + imm12 : base - imm12;
        bool ok;
        uint32_t v = flash_word(t, address & ~3u, &ok);
        if (!ok || (address & 3) + (uint32_t)bytes > 4) return T_UNSUPPORTED;
        v >>= 8 * (address & 3);
        if (bytes == 1) v = sign ? (uint32_t)(int32_t)(int8_t)v : (v & 0xFF);
        if (bytes == 2) v = sign ? (uint32_t)(int32_t)(int16_t)v : (v & 0xFFFF);
        if (tt == 15) {
            a64_mov32(h, 0, v);
            bx_w0(t);
            return T_END;
        }
        a64_mov32(h, G[tt], v);
        return T_OK;
    }

    // Address into w0; base writeback before the access, which the
    // interpreter also completes before writing the destination.
    int writebackSrc = -1;
    if (hw1 & 0x80) {
        const uint32_t imm12 = hw2 & 0xFFF;
        if (imm12 < 4096) a64_add_imm(h, 0, G[n], imm12, false);
    } else if (hw2 & 0x800) {
        const uint32_t imm8 = hw2 & 0xFF;
        const bool index = hw2 & 0x400, add = hw2 & 0x200, wback = hw2 & 0x100;
        if ((hw2 & 0xF00) == 0xE00) {                   // LDRT/STRT: the same here
            if (add) a64_add_imm(h, 0, G[n], imm8, false); else a64_sub_imm(h, 0, G[n], imm8, false);
        } else {
            if (!index && !wback) return T_UNSUPPORTED;
            // Written back before the access; inside an IT block the access
            // may be handed to the interpreter, which must see the old base.
            if (wback && t->inIT) return T_UNSUPPORTED;
            if (add) a64_add_imm(h, 1, G[n], imm8, false); else a64_sub_imm(h, 1, G[n], imm8, false);
            a64_mov(h, 0, index ? 1 : G[n]);
            if (wback) writebackSrc = 1;
        }
    } else if ((hw2 & 0xFC0) == 0) {
        a64_add(h, 0, G[n], G[hw2 & 0xF], SH_LSL, (int)((hw2 >> 4) & 3));
    } else {
        return T_UNSUPPORTED;
    }

    if (isLoad) {
        if (tt == 15) {
            // Into the PC. Only RAM and flash; the interpreter takes anything else.
            if (writebackSrc >= 0) return T_UNSUPPORTED;
            ram_range(t, 0, 4);
            ram_word(t, false, 0, 0);
            bx_w0(t);
            return T_END;
        }
        if (writebackSrc >= 0) a64_mov(h, G[n], writebackSrc);
        access(t, false, bytes, sign, 0, G[tt]);
    } else {
        int value = G[tt];
        if (writebackSrc >= 0) {
            if (tt == n) { a64_mov(h, 2, G[tt]); value = 2; }
            a64_mov(h, G[n], writebackSrc);
        }
        access(t, true, bytes, false, 0, value);
    }
    return T_OK;
}

static int translate_load_store_dual(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned op1 = (hw1 >> 7) & 3, op2 = (hw1 >> 4) & 3, op3 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, tt = hw2 >> 12, t2 = (hw2 >> 8) & 0xF;

    if (op1 == 1 && op2 == 1 && (op3 == 0 || op3 == 1)) {       // TBB, TBH
        const unsigned m = hw2 & 0xF;
        const int base = rn(t, (int)n, 1);
        if (op3 == 0) {
            a64_add(h, 0, base, G[m], SH_LSL, 0);
            access(t, false, 1, false, 0, 0);
        } else {
            a64_add(h, 0, base, G[m], SH_LSL, 1);
            access(t, false, 2, false, 0, 0);
        }
        a64_mov32(h, 1, t->pc + 4);
        a64_add(h, 0, 1, 0, SH_LSL, 1);
        goto_w0(t);
        return T_END;
    }
    if (op1 == 0 && op2 <= 1) return T_UNSUPPORTED;             // STREX, LDREX
    if (op1 == 1 && op2 <= 1) return T_UNSUPPORTED;             // exclusive byte/halfword

    // LDRD / STRD (immediate)
    const bool index = hw1 & 0x100, add = hw1 & 0x80, wback = hw1 & 0x20, isLoad = hw1 & 0x10;
    const uint32_t imm = (hw2 & 0xFF) << 2;
    if (n == 15) {
        if (!isLoad || wback) return T_UNSUPPORTED;
        const uint32_t base = (t->pc + 4) & ~3u;
        const uint32_t address = add ? base + imm : base - imm;
        bool ok1, ok2;
        const uint32_t lo = flash_word(t, address, &ok1), hi = flash_word(t, address + 4, &ok2);
        if (!ok1 || !ok2 || tt == 15 || t2 == 15) return T_UNSUPPORTED;
        a64_mov32(h, G[tt], lo);
        a64_mov32(h, G[t2], hi);
        return T_OK;
    }
    if (tt == 15 || t2 == 15) return T_UNSUPPORTED;
    if (add) a64_add_imm(h, 1, G[n], imm, false); else a64_sub_imm(h, 1, G[n], imm, false);
    a64_mov(h, 0, index ? 1 : G[n]);
    ram_range(t, 0, 8);
    if (isLoad) {
        ram_word(t, false, 2, 0);
        ram_word(t, false, 3, 4);
        if (wback) a64_mov(h, G[n], 1);
        a64_mov(h, G[tt], 2);
        a64_mov(h, G[t2], 3);
    } else {
        ram_word(t, true, G[tt], 0);
        ram_word(t, true, G[t2], 4);
        if (wback) a64_mov(h, G[n], 1);
    }
    return T_OK;
}

static int translate_load_store_multiple(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned n = hw1 & 0xF;
    const uint32_t list = hw2;
    const bool wback = hw1 & 0x20, isLoad = hw1 & 0x10;
    const uint32_t count = (uint32_t)__builtin_popcount(list & 0xFFFF);
    const unsigned mode = (hw1 >> 7) & 3;
    if ((mode != 1 && mode != 2) || n == 15 || count == 0) return T_UNSUPPORTED;
    if (!isLoad && (list & (1u << 15))) return T_UNSUPPORTED;
    if (list & (1u << 13)) return T_UNSUPPORTED;                // SP in the list

    if (mode == 1) a64_mov(h, 0, G[n]); else a64_sub_imm(h, 0, G[n], 4 * count, false);
    ram_range(t, 0, 4 * count);
    const bool pc = isLoad && (list & (1u << 15));
    if (pc) {
        ram_word(t, false, 1, 4 * (count - 1));
        branch_hot_to_cold(t, a64_tbz(1, 0, 0), interpret_stub(t));
    }
    // The base written back first when it isn't loaded; its new value is
    // known before any register changes.
    uint32_t off = 0;
    if (!isLoad) {
        for (int i = 0; i < 15; i++) {
            if (list & (1u << i)) { ram_word(t, true, G[i], off); off += 4; }
        }
        if (wback) {
            if (mode == 1) a64_add_imm(h, G[n], G[n], 4 * count, false); else a64_mov(h, G[n], 0);
        }
        return T_OK;
    }
    const bool baseLoaded = list & (1u << n);
    if (wback && !baseLoaded) {
        if (mode == 1) a64_add_imm(h, 2, G[n], 4 * count, false); else a64_mov(h, 2, 0);
    }
    for (int i = 0; i < 15; i++) {
        if (list & (1u << i)) { ram_word(t, false, G[i], off); off += 4; }
    }
    if (wback && !baseLoaded) a64_mov(h, G[n], 2);
    if (pc) {
        a64_and_imm(h, 0, 1, 0xFFFFFFFEu);
        goto_w0(t);
        return T_END;
    }
    return T_OK;
}

// --- 32-bit: data processing

static int translate_plain_immediate(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned op = (hw1 >> 4) & 0x1F;
    const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF;
    const uint32_t imm12 = (((hw1 >> 10) & 1) << 11) | (((hw2 >> 12) & 7) << 8) | (hw2 & 0xFF);
    const unsigned imm5 = (((hw2 >> 12) & 7) << 2) | ((hw2 >> 6) & 3);
    const unsigned field = hw2 & 0x1F;
    if (d == 15) return T_UNSUPPORTED;
    switch (op) {
    case 0x00:                                                  // ADDW, ADR
        if (d == 15) return T_UNSUPPORTED;
        if (n == 15) { a64_mov32(h, G[d], ((t->pc + 4) & ~3u) + imm12); return T_OK; }
        a64_add_imm(h, G[d], G[n], imm12, false);
        return T_OK;
    case 0x0A:                                                  // SUBW, ADR
        if (d == 15) return T_UNSUPPORTED;
        if (n == 15) { a64_mov32(h, G[d], ((t->pc + 4) & ~3u) - imm12); return T_OK; }
        a64_sub_imm(h, G[d], G[n], imm12, false);
        return T_OK;
    case 0x04:                                                  // MOVW
        if (d == 15) return T_UNSUPPORTED;
        a64_movz(h, G[d], ((hw1 & 0xF) << 12) | imm12, 0);
        return T_OK;
    case 0x0C:                                                  // MOVT
        if (d == 15) return T_UNSUPPORTED;
        a64_movk(h, G[d], ((hw1 & 0xF) << 12) | imm12, 1);
        return T_OK;
    case 0x14: {                                                // SBFX
        const unsigned width = field + 1;
        if (imm5 + width > 32 || d == 15 || n == 15) return T_UNSUPPORTED;
        a64_sbfx(h, G[d], G[n], (int)imm5, (int)width);
        return T_OK;
    }
    case 0x1C: {                                                // UBFX
        const unsigned width = field + 1;
        if (imm5 + width > 32 || d == 15 || n == 15) return T_UNSUPPORTED;
        a64_ubfx(h, G[d], G[n], (int)imm5, (int)width);
        return T_OK;
    }
    case 0x16: {                                                // BFI, BFC
        if (field < imm5 || d == 15) return T_UNSUPPORTED;
        const unsigned width = field - imm5 + 1;
        a64_bfi(h, G[d], n == 15 ? A64_ZR : G[n], (int)imm5, (int)width);
        return T_OK;
    }
    default:
        return T_UNSUPPORTED;                                   // SSAT, USAT and the 16-bit forms
    }
}

static int translate_data_register(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned op1 = (hw1 >> 4) & 0xF, op2 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
    if (d == 15 || m == 15) return T_UNSUPPORTED;

    if (op2 == 0 && (op1 & 8) == 0) {                           // shift by register
        if (n == 15) return T_UNSUPPORTED;
        shift_register(t, G[d], G[n], G[m], (int)((op1 >> 1) & 3), op1 & 1);
        return T_OK;
    }
    if ((op1 & 8) == 0 && (op2 & 8)) {                          // extend (and add)
        const unsigned rot = ((hw2 >> 4) & 3) * 8;
        const int src = G[m];
        if (rot) a64_ror_imm(h, 1, src, (int)rot); else a64_mov(h, 1, src);
        switch (op1) {
        case 0: a64_sxth(h, 1, 1); break;
        case 1: a64_uxth(h, 1, 1); break;
        case 4: a64_sxtb(h, 1, 1); break;
        case 5: a64_uxtb(h, 1, 1); break;
        default: return T_UNSUPPORTED;                          // the 16-bit pair forms
        }
        if (n == 15) a64_mov(h, G[d], 1); else a64_add(h, G[d], G[n], 1, SH_LSL, 0);
        return T_OK;
    }
    if ((op1 & 0xC) == 0x8 && (op2 & 0xC) == 0x8) {            // miscellaneous
        switch (((op1 & 3) << 2) | (op2 & 3)) {
        case 0x4: a64_rev(h, G[d], G[m]); return T_OK;
        case 0x5: a64_rev16(h, G[d], G[m]); return T_OK;
        case 0x6: a64_rbit(h, G[d], G[m]); return T_OK;
        case 0x7: a64_rev16(h, G[d], G[m]); a64_sxth(h, G[d], G[d]); return T_OK;
        case 0xC: a64_clz(h, G[d], G[m]); return T_OK;
        default: return T_UNSUPPORTED;
        }
    }
    return T_UNSUPPORTED;
}

static int translate_multiply(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 3;
    const unsigned n = hw1 & 0xF, a = hw2 >> 12, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
    if (d == 15 || n == 15 || m == 15) return T_UNSUPPORTED;
    if (op1 == 0 && op2 == 0) {                                 // MLA, MUL
        a64_madd(h, G[d], G[n], G[m], a == 15 ? A64_ZR : G[a]);
        return T_OK;
    }
    if (op1 == 0 && op2 == 1 && a != 15) {                      // MLS
        a64_msub(h, G[d], G[n], G[m], G[a]);
        return T_OK;
    }
    if (op1 == 1 && a == 15) {                                  // SMULxy
        if (hw2 & 0x20) a64_asr_imm(h, 1, G[n], 16); else a64_sxth(h, 1, G[n]);
        if (hw2 & 0x10) a64_asr_imm(h, 2, G[m], 16); else a64_sxth(h, 2, G[m]);
        a64_madd(h, G[d], 1, 2, A64_ZR);
        return T_OK;
    }
    return T_UNSUPPORTED;
}

static int translate_long_multiply(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const unsigned op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, lo = hw2 >> 12, hi = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
    if (n == 15 || m == 15) return T_UNSUPPORTED;
    if (op1 == 1 && op2 == 0xF) { if (hi == 15) return T_UNSUPPORTED; a64_sdiv(h, G[hi], G[n], G[m]); return T_OK; }
    if (op1 == 3 && op2 == 0xF) { if (hi == 15) return T_UNSUPPORTED; a64_udiv(h, G[hi], G[n], G[m]); return T_OK; }
    if (lo == 15 || hi == 15) return T_UNSUPPORTED;
    const bool accumulate = op1 == 4 || op1 == 6;
    const bool isSigned = op1 == 0 || op1 == 4;
    if (!((op1 == 0 || op1 == 2 || op1 == 4 || op1 == 6) && op2 == 0)) return T_UNSUPPORTED;
    int acc = A64_ZR;
    if (accumulate) {
        a64_mov(h, 1, G[lo]);                                   // zero-extended
        a64(h, 0xAA000000u | ((uint32_t)G[hi] << 16) | (32u << 10) | (1u << 5) | 1u);   // orr x1, x1, xhi, lsl #32
        acc = 1;
    }
    if (isSigned) a64_smaddl(h, 2, G[n], G[m], acc); else a64_umaddl(h, 2, G[n], G[m], acc);
    a64_mov(h, G[lo], 2);
    a64(h, 0xD3400000u | (32u << 16) | (63u << 10) | (2u << 5) | (uint32_t)G[hi]);      // lsr xhi, x2, #32
    return T_OK;
}

// --- 32-bit: branches and control

static int translate_branch_misc(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const uint32_t s = (hw1 >> 10) & 1, j1 = (hw2 >> 13) & 1, j2 = (hw2 >> 11) & 1;
    switch (hw2 & 0x5000) {
    case 0x0000: {
        const unsigned op = (hw1 >> 4) & 0x7F;
        if ((op & 0x38) != 0x38) {                              // B<c>.W
            const uint32_t imm = (s << 20) | (j2 << 19) | (j1 << 18) | ((hw1 & 0x3F) << 12) | ((hw2 & 0x7FF) << 1);
            const int32_t offset = (int32_t)(imm << 11) >> 11;
            conditional_goto(t, (int)((hw1 >> 6) & 0xF), t->pc + 4 + (uint32_t)offset);
            return T_END;
        }
        switch (op) {
        case 0x38: case 0x39: {                                 // MSR
            const unsigned sysm = hw2 & 0xFF, src = hw1 & 0xF;
            if (src == 15) return T_UNSUPPORTED;
            if (sysm == 17) { a64_and_imm(h, X16, G[src], 0xFF); a64_str_imm(h, X16, CTX, OFF(cpu.basepri)); return T_OK; }
            if (sysm == 16) { a64_and_imm(h, X16, G[src], 1); a64_str_imm(h, X16, CTX, OFF(cpu.primask)); return T_OK; }
            return T_UNSUPPORTED;
        }
        case 0x3A:                                              // hints
            if ((hw2 & 0xFF) == 3) return T_UNSUPPORTED;        // WFI.W: interpreter
            return T_OK;
        case 0x3B:                                              // barriers, CLREX
            return T_OK;
        case 0x3E: case 0x3F: {                                 // MRS
            const unsigned sysm = hw2 & 0xFF, dst = (hw2 >> 8) & 0xF;
            if (dst >= 13) return T_UNSUPPORTED;
            if (sysm == 17 || sysm == 18) { a64_ldr_imm(h, G[dst], CTX, OFF(cpu.basepri)); return T_OK; }
            if (sysm == 16) { a64_ldr_imm(h, G[dst], CTX, OFF(cpu.primask)); return T_OK; }
            return T_UNSUPPORTED;
        }
        default:
            return T_UNSUPPORTED;
        }
    }
    case 0x1000: case 0x5000: {                                 // B.W, BL
        const uint32_t i1 = !(j1 ^ s), i2 = !(j2 ^ s);
        const uint32_t imm = (s << 24) | (i1 << 23) | (i2 << 22) | ((hw1 & 0x3FF) << 12) | ((hw2 & 0x7FF) << 1);
        const int32_t offset = (int32_t)(imm << 7) >> 7;
        if (hw2 & 0x4000) a64_mov32(h, G[14], (t->pc + 4) | 1);
        goto_pc(t, h, t->pc + 4 + (uint32_t)offset);
        return T_END;
    }
    default:
        return T_UNSUPPORTED;
    }
}

// --- FPv4-SP

static void s_load(tr_t *t, int host, unsigned guest) { a64_ldr_s_imm(&t->hot, host, CTX, OFF_S(guest)); }
static void s_store(tr_t *t, int host, unsigned guest) { a64_str_s_imm(&t->hot, host, CTX, OFF_S(guest)); }
static void sw_load(tr_t *t, int w, unsigned guest) { a64_ldr_imm(&t->hot, w, CTX, OFF_S(guest)); }
static void sw_store(tr_t *t, int w, unsigned guest) { a64_str_imm(&t->hot, w, CTX, OFF_S(guest)); }

static uint32_t vfp_expand_imm(uint32_t imm8)
{
    const uint32_t sign = (imm8 >> 7) & 1;
    const uint32_t b6 = (imm8 >> 6) & 1;
    const uint32_t exponent = ((b6 ^ 1) << 7) | (b6 ? 0x7C : 0) | ((imm8 >> 4) & 3);
    return (sign << 31) | (exponent << 23) | ((imm8 & 0xF) << 19);
}

static int translate_vfp_data(tr_t *t, uint32_t inst)
{
    a64_buf_t *h = &t->hot;
    if (inst & 0x100) return T_UNSUPPORTED;
    const unsigned d = (((inst >> 12) & 0xF) << 1) | ((inst >> 22) & 1);
    const unsigned n = (((inst >> 16) & 0xF) << 1) | ((inst >> 7) & 1);
    const unsigned m = ((inst & 0xF) << 1) | ((inst >> 5) & 1);
    const bool op = (inst >> 6) & 1;
    const unsigned opc1 = (((inst >> 23) & 1) << 2) | ((inst >> 20) & 3);

    if (opc1 < 7) {
        s_load(t, 1, n);
        s_load(t, 2, m);
        switch (opc1) {
        case 0:                                                 // VMLA, VMLS
            s_load(t, 0, d);
            a64_fmul(h, 3, 1, 2);
            if (op) a64_fsub(h, 0, 0, 3); else a64_fadd(h, 0, 0, 3);
            break;
        case 1:                                                 // VNMLS, VNMLA
            s_load(t, 0, d);
            a64_fmul(h, 3, 1, 2);
            if (op) { a64_fneg(h, 0, 0); a64_fsub(h, 0, 0, 3); }
            else a64_fsub(h, 0, 3, 0);
            break;
        case 2:
            if (op) a64_fnmul(h, 0, 1, 2); else a64_fmul(h, 0, 1, 2);
            break;
        case 3:
            if (op) a64_fsub(h, 0, 1, 2); else a64_fadd(h, 0, 1, 2);
            break;
        case 4:
            if (op) return T_UNSUPPORTED;
            a64_fdiv(h, 0, 1, 2);
            break;
        case 5:                                                 // VFNMS, VFNMA
            s_load(t, 3, d);
            if (op) a64_fnmadd(h, 0, 1, 2, 3); else a64_fnmsub(h, 0, 1, 2, 3);
            break;
        case 6:                                                 // VFMA, VFMS
            s_load(t, 3, d);
            if (op) a64_fmsub(h, 0, 1, 2, 3); else a64_fmadd(h, 0, 1, 2, 3);
            break;
        }
        s_store(t, 0, d);
        return T_OK;
    }

    const unsigned opc2 = (inst >> 16) & 0xF, opc3 = (inst >> 6) & 3;
    if ((opc3 & 1) == 0) {                                      // VMOV immediate
        a64_mov32(h, 1, vfp_expand_imm((opc2 << 4) | (inst & 0xF)));
        sw_store(t, 1, d);
        return T_OK;
    }
    switch (opc2) {
    case 0x0:                                                   // VMOV, VABS (bits)
        sw_load(t, 1, m);
        if (opc3 != 1) a64_and_imm(h, 1, 1, 0x7FFFFFFFu);
        sw_store(t, 1, d);
        return T_OK;
    case 0x1:
        if (opc3 == 1) {                                        // VNEG (bits)
            sw_load(t, 1, m);
            a64_movz(h, 2, 0x8000, 1);
            a64_eor(h, 1, 1, 2, SH_LSL, 0);
            sw_store(t, 1, d);
        } else {                                                // VSQRT
            s_load(t, 1, m);
            a64_fsqrt(h, 0, 1);
            s_store(t, 0, d);
        }
        return T_OK;
    case 0x4: case 0x5:                                         // VCMP{E}: flags to FPSCR, APSR kept
        s_load(t, 0, d);
        if (opc2 == 0x4) s_load(t, 1, m);
        a64_mrs_nzcv(h, 2);
        if (opc2 == 0x4) a64_fcmp(h, 0, 1); else a64_fcmp0(h, 0);
        a64_mrs_nzcv(h, 3);
        a64_msr_nzcv(h, 2);
        a64_ldr_imm(h, 4, CTX, OFF(cpu.fpscr));
        a64_and_imm(h, 4, 4, 0x0FFFFFFFu);
        a64_orr(h, 4, 4, 3, SH_LSL, 0);
        a64_str_imm(h, 4, CTX, OFF(cpu.fpscr));
        return T_OK;
    case 0x8:                                                   // VCVT.F32.{S32,U32}
        sw_load(t, 1, m);
        if (inst & 0x80) a64_scvtf(h, 0, 1); else a64_ucvtf(h, 0, 1);
        s_store(t, 0, d);
        return T_OK;
    case 0xC: case 0xD:                                         // VCVT.{U32,S32}.F32, round towards zero
        if (!(inst & 0x80)) return T_UNSUPPORTED;               // VCVTR: FPSCR's mode
        s_load(t, 1, m);
        if (opc2 & 1) a64_fcvtzs(h, 1, 1); else a64_fcvtzu(h, 1, 1);
        sw_store(t, 1, d);
        return T_OK;
    default:
        return T_UNSUPPORTED;
    }
}

static int translate_vfp(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    const uint32_t inst = (hw1 << 16) | hw2;
    const unsigned coproc = (hw2 >> 8) & 0xF;
    if ((coproc & 0xE) != 0xA) return T_UNSUPPORTED;
    const bool doubleRegs = coproc & 1;

    if ((hw1 & 0xFF00) == 0xEE00) {
        if ((hw2 & 0x10) == 0) return translate_vfp_data(t, inst);
        const bool toCore = hw1 & 0x10;
        const unsigned a = (hw1 >> 5) & 7, tt = hw2 >> 12;
        if (doubleRegs || tt == 13) return T_UNSUPPORTED;
        if (a == 0) {                                           // VMOV Rt <-> Sn
            if (tt == 15) return T_UNSUPPORTED;
            const unsigned sn = ((hw1 & 0xF) << 1) | ((hw2 >> 7) & 1);
            if (toCore) sw_load(t, G[tt], sn); else sw_store(t, G[tt], sn);
            return T_OK;
        }
        if (a == 7 && (hw1 & 0xF) == 1) {                       // VMRS, VMSR
            if (toCore) {
                if (tt == 15) {
                    a64_ldr_imm(h, X16, CTX, OFF(cpu.fpscr));
                    a64_and_imm(h, X16, X16, 0xF0000000u);
                    a64_msr_nzcv(h, X16);
                } else {
                    a64_ldr_imm(h, G[tt], CTX, OFF(cpu.fpscr));
                }
            } else {
                if (tt == 15) return T_UNSUPPORTED;
                a64_str_imm(h, G[tt], CTX, OFF(cpu.fpscr));
            }
            return T_OK;
        }
        return T_UNSUPPORTED;
    }

    if ((hw1 & 0xFFE0) == 0xEC40) {                             // VMOV two core registers
        const bool toCore = hw1 & 0x10;
        const unsigned tt = hw2 >> 12, t2 = hw1 & 0xF;
        const unsigned first = doubleRegs ? ((((hw2 >> 5) & 1) << 4) | (hw2 & 0xF)) * 2
                                          : ((hw2 & 0xF) << 1) | ((hw2 >> 5) & 1);
        if (first + 1 > 31 || tt >= 13 || t2 >= 13) return T_UNSUPPORTED;
        if (toCore) { sw_load(t, G[tt], first); sw_load(t, G[t2], first + 1); }
        else { sw_store(t, G[tt], first); sw_store(t, G[t2], first + 1); }
        return T_OK;
    }

    // VLDR, VSTR, VLDM, VSTM, VPUSH, VPOP
    const bool index = hw1 & 0x100, add = hw1 & 0x80, wback = hw1 & 0x20, isLoad = hw1 & 0x10;
    const unsigned n = hw1 & 0xF, vd = (hw2 >> 12) & 0xF, dBit = (hw1 >> 6) & 1;
    const uint32_t imm8 = hw2 & 0xFF;
    const unsigned first = doubleRegs ? ((dBit << 4) | vd) * 2 : (vd << 1) | dBit;

    if (index && !wback) {
        const unsigned words = doubleRegs ? 2 : 1;
        if (first + words > 32) return T_UNSUPPORTED;
        if (n == 15) {
            if (!isLoad) return T_UNSUPPORTED;
            const uint32_t base = (t->pc + 4) & ~3u;
            const uint32_t address = add ? base + imm8 * 4 : base - imm8 * 4;
            for (unsigned i = 0; i < words; i++) {
                bool ok;
                const uint32_t v = flash_word(t, address + 4 * i, &ok);
                if (!ok) return T_UNSUPPORTED;
                a64_mov32(h, 1, v);
                sw_store(t, 1, first + i);
            }
            return T_OK;
        }
        if (add) a64_add_imm(h, 0, G[n], imm8 * 4, false); else a64_sub_imm(h, 0, G[n], imm8 * 4, false);
        if (words == 1) {
            if (isLoad) { access(t, false, 4, false, 0, 1); sw_store(t, 1, first); }
            else { sw_load(t, 1, first); access(t, true, 4, false, 0, 1); }
        } else {
            ram_range(t, 0, 8);
            for (unsigned i = 0; i < 2; i++) {
                if (isLoad) { ram_word(t, false, 1, 4 * i); sw_store(t, 1, first + i); }
                else { sw_load(t, 1, first + i); ram_word(t, true, 1, 4 * i); }
            }
        }
        return T_OK;
    }
    if (index == add || n == 15) return T_UNSUPPORTED;
    const unsigned words = imm8;
    if (first + words > 32 || words == 0) return T_UNSUPPORTED;
    if (add) a64_mov(h, 0, G[n]); else a64_sub_imm(h, 0, G[n], words * 4, false);
    ram_range(t, 0, words * 4);
    for (unsigned i = 0; i < words; i++) {
        if (isLoad) { ram_word(t, false, 1, 4 * i); sw_store(t, 1, first + i); }
        else { sw_load(t, 1, first + i); ram_word(t, true, 1, 4 * i); }
    }
    if (wback) {
        if (add) a64_add_imm(h, G[n], G[n], words * 4, false); else a64_mov(h, G[n], 0);
    }
    return T_OK;
}

// --- 32-bit dispatch

static int translate32(tr_t *t, uint32_t hw1, uint32_t hw2)
{
    a64_buf_t *h = &t->hot;
    switch ((hw1 >> 11) & 3) {
    case 1:
        if (hw1 & 0x0400) return translate_vfp(t, hw1, hw2);
        if (hw1 & 0x0200) {                                     // data processing (shifted register)
            const unsigned op = (hw1 >> 5) & 0xF;
            const bool setflags = hw1 & 0x10;
            const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
            const unsigned imm5 = (((hw2 >> 12) & 7) << 2) | ((hw2 >> 6) & 3);
            if (op == 0x6 || m == 15) return T_UNSUPPORTED;     // PKH
            int type;
            unsigned amount;
            decode_imm_shift((hw2 >> 4) & 3, imm5, &type, &amount);
            if (type == SHIFT_RRX) return T_UNSUPPORTED;
            bool logical = op <= 0x4;
            int carry = -1;
            if (setflags && logical && shift_carry(t, 6, G[m], type, amount)) carry = 2;
            shift_imm(t, 7, G[m], type, amount);
            if (!data_processing(t, op, setflags, d, n, 7, carry)) return T_UNSUPPORTED;
            return T_OK;
        }
        if ((hw1 & 0x0040) == 0) return translate_load_store_multiple(t, hw1, hw2);
        return translate_load_store_dual(t, hw1, hw2);
    case 2:
        if (hw2 & 0x8000) return translate_branch_misc(t, hw1, hw2);
        if ((hw1 & 0x0200) == 0) {                              // data processing (modified immediate)
            const unsigned op = (hw1 >> 5) & 0xF;
            const bool setflags = hw1 & 0x10;
            const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF;
            const uint32_t imm12 = (((hw1 >> 10) & 1) << 11) | (((hw2 >> 12) & 7) << 8) | (hw2 & 0xFF);
            int carry;
            const uint32_t imm = thumb_expand_imm(imm12, &carry);
            a64_mov32(h, 7, imm);
            const bool logical = op <= 0x4;
            if (!data_processing(t, op, setflags, d, n, 7, setflags && logical ? carry : -1)) return T_UNSUPPORTED;
            return T_OK;
        }
        return translate_plain_immediate(t, hw1, hw2);
    case 3:
        if (hw1 & 0x0400) return T_UNSUPPORTED;
        if ((hw1 & 0x0200) == 0) return translate_load_store_single(t, hw1, hw2);
        switch ((hw1 >> 7) & 3) {
        case 0: case 1: return (hw1 & 0x100) ? T_UNSUPPORTED : translate_data_register(t, hw1, hw2);
        case 2: return translate_multiply(t, hw1, hw2);
        default: return translate_long_multiply(t, hw1, hw2);
        }
    default:
        return T_UNSUPPORTED;
    }
}

// --- Blocks

static inline uint16_t fetch(vfc_t *vfc, uint32_t pc)
{
    uint16_t v;
    memcpy(&v, vfc->flash + (pc - VFC_FLASH_BASE), 2);
    return v;
}

static bool condition_is_always(unsigned cond) { return cond == 0xE; }


static void *translate(vfc_t *vfc, uint32_t startPc)
{
    struct vfc_jit *jit = vfc->jit;
    tr_t *t = calloc(1, sizeof(tr_t));
    if (!t) return NULL;
    t->vfc = vfc;
    t->jit = jit;
    t->hot = (a64_buf_t){ jit->hot, 0, HOT_CAP, false };
    t->cold = (a64_buf_t){ jit->cold, 0, COLD_CAP, false };
    t->startPc = startPc;

    // Prologue: take this block's instructions from the budget, or leave
    // before running any when there are too few left.
    a64_ldr_x_imm(&t->hot, X16, CTX, OFF(jitBudget));
    t->lenPatchAt = t->hot.count;
    a64_sub_x_imm(&t->hot, X16, X16, 0);
    t->pc = startPc;
    t->k = 0;
    branch_hot_to_cold(t, a64_tbnz(X16, 63, 0), exit_stub(t, startPc, EXIT_BUDGET, -1, false));
    a64_str_x_imm(&t->hot, X16, CTX, OFF(jitBudget));

    uint32_t pc = startPc;
    int count = 0;
    uint8_t itstate = 0;
    bool ended = false;
    while (count < MAX_BLOCK && !t->failed) {
        const uint32_t offset = pc - VFC_FLASH_BASE;
        if (offset + 4 > vfc->flashUsed) break;
        const uint32_t hw1 = fetch(vfc, pc);
        const bool wide = (hw1 >> 11) >= 0x1D;
        const uint32_t hw2 = wide ? fetch(vfc, pc + 2) : 0;
        const bool isIT = !wide && (hw1 & 0xFF00) == 0xBF00 && (hw1 & 0xF);
        if (isIT && (itstate & 0xF)) break;                     // IT inside IT: interpreter
        if (isIT && count + 5 > MAX_BLOCK) break;              // don't split an IT block

        t->pc = pc;
        t->size = wide ? 4 : 2;
        t->k = count;
        t->itstate = itstate;
        uint32_t skip = UINT32_MAX;
        const uint32_t hotBefore = t->hot.count, coldBefore = t->cold.count;
        const int fixBefore = t->nfixups, addBefore = t->naddbacks;
        if (itstate & 0xF) {
            const unsigned cond = itstate >> 4;
            t->inIT = true;
            if (!condition_is_always(cond)) {
                skip = t->hot.count;
                a64(&t->hot, a64_bcond((int)(cond ^ 1), 0));
            }
        } else {
            t->inIT = false;
        }

        int result;
        if (!wide && hw1 == 0xBF30) {                           // WFI
            branch_hot_to_cold(t, a64_b(0), exit_stub(t, pc + 2, EXIT_WFI, 1, false));
            result = T_END;
        } else if (isIT) {
            result = T_OK;
        } else {
            result = wide ? translate32(t, hw1, hw2) : translate16(t, hw1);
        }

        if (result == T_UNSUPPORTED) {
            // Undo anything half-emitted, then hand this instruction to the
            // interpreter. The IT state it needs is set by the stub.
            t->hot.count = hotBefore;
            t->cold.count = coldBefore;
            t->nfixups = fixBefore;
            t->naddbacks = addBefore;
            if (count == 0) {
                free(t);
                return NULL;
            }
            branch_hot_to_cold(t, a64_b(0), exit_stub(t, pc, EXIT_INTERPRET, 0, (itstate & 0xF) != 0));
            ended = true;
            break;
        }
        if (skip != UINT32_MAX) {
            a64_patch(t->hot.code, skip, t->hot.count);
        }
        count++;
        if (isIT) {
            itstate = hw1 & 0xFF;
        } else if (itstate & 0xF) {
            itstate = (itstate & 7) == 0 ? 0 : (uint8_t)((itstate & 0xE0) | ((itstate << 1) & 0x1F));
        }
        pc += wide ? 4 : 2;
        if (result == T_END) {
            if (itstate & 0xF) {
                // A branch that isn't the last of its IT block: unpredictable;
                // the conditional skip already continues below.
            }
            ended = true;
            if (skip != UINT32_MAX) {
                // Not taken: carry on after the branch.
                goto_pc(t, &t->hot, pc);
            }
            break;
        }
    }
    if (t->failed || t->hot.overflow || t->cold.overflow || count == 0) {
        free(t);
        return NULL;
    }
    if (!ended) {
        if (itstate & 0xF) {
            // Ran out of room inside an IT block: the interpreter continues it.
            t->itstate = itstate;
            t->k = count;
            t->pc = pc;
            branch_hot_to_cold(t, a64_b(0), exit_stub(t, pc, EXIT_INTERPRET, 0, true));
        } else {
            goto_pc(t, &t->hot, pc);
        }
    }

    // Lay out hot then cold, fix up branches and budgets.
    const uint32_t length = (uint32_t)count;
    t->hot.code[t->lenPatchAt] = (t->hot.code[t->lenPatchAt] & ~(0xFFFu << 10)) | (length << 10);
    for (int i = 0; i < t->naddbacks; i++) {
        const addback_t *a = &t->addbacks[i];
        const uint32_t back = length - (uint32_t)a->k - (a->after ? 1u : 0u);
        t->cold.code[a->coldAt] = (t->cold.code[a->coldAt] & ~(0xFFFu << 10)) | (back << 10);
    }
    const uint32_t total = t->hot.count + t->cold.count;
    if (jit->used + total > CACHE_INSNS) {
        free(t);
        return NULL;                                            // full: the interpreter carries on
    }
    const uint32_t base = jit->used;
    uint32_t *out = jit->assembled;
    memcpy(out, t->hot.code, t->hot.count * 4);
    memcpy(out + t->hot.count, t->cold.code, t->cold.count * 4);
    for (int i = 0; i < t->nfixups; i++) {
        const fixup_t *f = &t->fixups[i];
        const uint32_t from = f->fromCold ? t->hot.count + f->at : f->at;
        int64_t to;
        switch (f->toKind) {
        case 0: to = f->to; break;
        case 1: to = t->hot.count + f->to; break;
        default: to = (int64_t)f->to - base; break;
        }
        out[from] = set_offset(out[from], (int32_t)(to - from));
    }
    if (jit->npending + (uint32_t)t->nlinks > jit->pendingCap) {
        const uint32_t cap = (jit->pendingCap ? jit->pendingCap * 2 : 1024) + (uint32_t)t->nlinks;
        void *grown = realloc(jit->pending, cap * sizeof(*jit->pending));
        if (!grown) {
            free(t);
            return NULL;
        }
        jit->pending = grown;
        jit->pendingCap = cap;
    }
    for (int i = 0; i < t->nlinks; i++) {
        const uint32_t from = t->links[i].fromCold ? t->hot.count + t->links[i].at : t->links[i].at;
        jit->pending[jit->npending].at = base + from;
        jit->pending[jit->npending].target = t->links[i].target;
        jit->npending++;
    }
    pthread_jit_write_protect_np(0);
    memcpy(jit->cache + base, out, total * 4);
    // Branches already waiting for this block now go straight to it.
    for (uint32_t i = 0; i < jit->npending;) {
        if (jit->pending[i].target != startPc) {
            i++;
            continue;
        }
        const uint32_t at = jit->pending[i].at;
        jit->cache[at] = set_offset(jit->cache[at], (int32_t)base - (int32_t)at);
        sys_icache_invalidate(jit->cache + at, 4);
        jit->pending[i] = jit->pending[--jit->npending];
    }
    pthread_jit_write_protect_np(1);
    sys_icache_invalidate(jit->cache + base, total * 4);
    jit->used += total;
    free(t);
    void *entry = jit->cache + base;
    jit->table[(startPc - VFC_FLASH_BASE) / 2] = entry;
    return entry;
}

// --- Shared code: entry, exit, dispatch

static void emit_shared(struct vfc_jit *jit)
{
    a64_buf_t b = { jit->cache, 0, 1024, false };

    // enter(vfc, block)
    jit->enter = b.count;
    a64_stp_x_pre(&b, 29, 30, A64_SP, -96);
    a64(&b, 0xA9000000u | ((16 / 8) << 15) | (20u << 10) | (31u << 5) | 19u);   // stp x19, x20, [sp, #16]
    a64(&b, 0xA9000000u | ((32 / 8) << 15) | (22u << 10) | (31u << 5) | 21u);
    a64(&b, 0xA9000000u | ((48 / 8) << 15) | (24u << 10) | (31u << 5) | 23u);
    a64(&b, 0xA9000000u | ((64 / 8) << 15) | (26u << 10) | (31u << 5) | 25u);
    a64(&b, 0xA9000000u | ((80 / 8) << 15) | (28u << 10) | (31u << 5) | 27u);
    a64(&b, 0xAA0003E0u | (0u << 16) | CTX);                                     // mov x28, x0
    a64_ldr_x_imm(&b, RAMB, CTX, OFF(jitRam));
    for (int i = 0; i < 14; i += 2) a64_ldp_w(&b, G[i], G[i + 1], CTX, (int)OFF_R(i));
    a64_ldr_imm(&b, G[14], CTX, OFF_R(14));
    a64_ldr_imm(&b, X16, CTX, OFF(jitNzcv));
    a64_msr_nzcv(&b, X16);
    a64_br(&b, 1);

    // exit: w0 = guest pc, w1 = kind
    jit->exit = b.count;
    for (int i = 0; i < 14; i += 2) a64_stp_w(&b, G[i], G[i + 1], CTX, (int)OFF_R(i));
    a64_str_imm(&b, G[14], CTX, OFF_R(14));
    a64_str_imm(&b, 0, CTX, OFF_R(15));
    a64_mrs_nzcv(&b, X16);
    a64_str_imm(&b, X16, CTX, OFF(jitNzcv));
    a64_str_imm(&b, 1, CTX, OFF(jitExitKind));
    a64(&b, 0xA9400000u | ((16 / 8) << 15) | (20u << 10) | (31u << 5) | 19u);   // ldp x19, x20, [sp, #16]
    a64(&b, 0xA9400000u | ((32 / 8) << 15) | (22u << 10) | (31u << 5) | 21u);
    a64(&b, 0xA9400000u | ((48 / 8) << 15) | (24u << 10) | (31u << 5) | 23u);
    a64(&b, 0xA9400000u | ((64 / 8) << 15) | (26u << 10) | (31u << 5) | 25u);
    a64(&b, 0xA9400000u | ((80 / 8) << 15) | (28u << 10) | (31u << 5) | 27u);
    a64_ldp_x_post(&b, 29, 30, A64_SP, 96);
    a64_ret(&b);

    // dispatch: w0 = guest pc
    jit->dispatch = b.count;
    a64_movz(&b, X16, 0x0800, 1);
    a64_sub(&b, X16, 0, X16, SH_LSL, 0);
    a64_lsr_imm(&b, X17, X16, 21);
    const uint32_t miss1 = b.count;
    a64(&b, a64_cbnz(X17, 0));
    a64_lsr_imm(&b, X16, X16, 1);
    a64_ldr_x_imm(&b, X17, CTX, OFF(jitTable));
    a64_ldr_x_lsl3(&b, X16, X17, X16);
    const uint32_t miss2 = b.count;
    a64(&b, a64_cbz_x(X16, 0));
    a64_br(&b, X16);
    a64_patch(b.code, miss1, b.count);
    a64_patch(b.code, miss2, b.count);
    a64_movz(&b, 1, EXIT_NORMAL, 0);
    a64(&b, a64_b((int32_t)jit->exit - (int32_t)b.count));
    jit->used = b.count;
}

static bool jit_init(vfc_t *vfc)
{
    if (vfc->jit) return true;
    struct vfc_jit *jit = calloc(1, sizeof(*jit));
    if (!jit) return false;
    jit->cache = mmap(NULL, CACHE_INSNS * 4, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
    if (jit->cache == MAP_FAILED) {
        free(jit);
        return false;
    }
    jit->table = calloc(VFC_FLASH_SIZE / 2, sizeof(void *));
    jit->untranslatable = calloc(VFC_FLASH_SIZE / 2, 1);
    if (!jit->table || !jit->untranslatable) {
        free(jit->table);
        free(jit->untranslatable);
        munmap(jit->cache, CACHE_INSNS * 4);
        free(jit);
        return false;
    }
    pthread_jit_write_protect_np(0);
    emit_shared(jit);
    pthread_jit_write_protect_np(1);
    sys_icache_invalidate(jit->cache, jit->used * 4);
    vfc->jit = jit;
    vfc->jitTable = jit->table;
    vfc->jitLoad = (void *)jit_load;
    vfc->jitStore = (void *)jit_store;
    return true;
}

bool vfc_jit_available(void)
{
    return true;
}

#ifdef VFC_JIT_STATS
// Development: where the interpreter still runs, and how often the generated
// code comes back to C. Build with -DVFC_JIT_STATS.
static uint64_t *statInterpreted, statTrips, statInterpretedTotal, statOutside;
static void stat_interpreted(vfc_t *vfc, uint32_t pc, const char *why)
{
    (void)why;
    statInterpretedTotal++;
    const uint32_t offset = pc - VFC_FLASH_BASE;
    if (offset >= vfc->flashUsed) { statOutside++; return; }
    if (!statInterpreted) statInterpreted = calloc(VFC_FLASH_SIZE / 2, sizeof(uint64_t));
    statInterpreted[offset / 2]++;
}
static void stat_report(vfc_t *vfc)
{
    if (!statInterpreted) return;
    fprintf(stderr, "jit: %llu trips, %llu interpreted (%llu outside flash)\n",
            (unsigned long long)statTrips, (unsigned long long)statInterpretedTotal, (unsigned long long)statOutside);
    for (int k = 0; k < 30; k++) {
        uint32_t best = 0;
        for (uint32_t i = 0; i < vfc->flashUsed / 2; i++) if (statInterpreted[i] > statInterpreted[best]) best = i;
        if (!statInterpreted[best]) break;
        uint16_t h0, h1;
        memcpy(&h0, vfc->flash + best * 2, 2);
        memcpy(&h1, vfc->flash + best * 2 + 2, 2);
        fprintf(stderr, "  %08x  %04x %04x  %llu\n", VFC_FLASH_BASE + best * 2, h0, h1, (unsigned long long)statInterpreted[best]);
        statInterpreted[best] = 0;
    }
}
#define STAT_INTERPRETED(pc, why) stat_interpreted(vfc, pc, why)
#define STAT_TRIP() (statTrips++)
#else
#define STAT_INTERPRETED(pc, why) ((void)0)
#define STAT_TRIP() ((void)0)
#endif

void vfc_jit_free(vfc_t *vfc)
{
    if (!vfc->jit) return;
#ifdef VFC_JIT_STATS
    stat_report(vfc);
#endif
    munmap(vfc->jit->cache, CACHE_INSNS * 4);
    free(vfc->jit->table);
    free(vfc->jit->untranslatable);
    free(vfc->jit->pending);
    free(vfc->jit);
    vfc->jit = NULL;
}

void vfc_jit_flush(vfc_t *vfc)
{
    vfc_jit_free(vfc);
}

typedef void (*enter_fn)(vfc_t *, void *);

// One trip: a translated run of blocks from the current PC, or one
// interpreted instruction. False when the core should stop.
static bool run_once(vfc_t *vfc, uint64_t *budget)
{
    struct vfc_jit *jit = vfc->jit;
    vfc_cpu_t *cpu = &vfc->cpu;
    const uint32_t pc = cpu->r[15];
    const uint32_t offset = pc - VFC_FLASH_BASE;
    void *entry = NULL;
    if ((cpu->itstate & 0xF) == 0 && offset < vfc->flashUsed && (offset & 1) == 0
        && !jit->untranslatable[offset / 2]) {
        entry = jit->table[offset / 2];
        if (!entry) {
            entry = translate(vfc, pc);
            if (!entry) jit->untranslatable[offset / 2] = 1;
        }
    }
    if (!entry) {
        STAT_INTERPRETED(pc, "start");
        vfc_cpu_run(vfc, 1);
        (*budget)--;
        return !vfc->stopRequested;
    }
    STAT_TRIP();
    vfc->jitFlash = vfc->flash;
    vfc->jitRam = vfc->ram;
    vfc->jitBudget = (int64_t)(*budget > (uint64_t)INT64_MAX ? INT64_MAX : *budget);
    vfc->jitNzcv = ((uint32_t)cpu->n << 31) | ((uint32_t)cpu->z << 30) | ((uint32_t)cpu->c << 29) | ((uint32_t)cpu->v << 28);
    ((enter_fn)(void *)(jit->cache + jit->enter))(vfc, entry);
    const uint64_t left = (uint64_t)vfc->jitBudget;
    vfc->instructions += *budget - left;
    *budget = left;
    cpu->n = (vfc->jitNzcv >> 31) & 1;
    cpu->z = (vfc->jitNzcv >> 30) & 1;
    cpu->c = (vfc->jitNzcv >> 29) & 1;
    cpu->v = (vfc->jitNzcv >> 28) & 1;
    switch (vfc->jitExitKind) {
    case EXIT_WFI:
        vfc->stopRequested = true;
        vfc->stopReason = VFC_STOP_IDLE;
        return false;
    case EXIT_STOP:
        return false;
    case EXIT_INTERPRET:
    case EXIT_BUDGET:
        // The instruction the generated code couldn't take: the interpreter
        // runs it, or translating from here would find the same one again.
        if (*budget > 0) {
            STAT_INTERPRETED(cpu->r[15], "exit");
            vfc_cpu_run(vfc, 1);
            (*budget)--;
        }
        return !vfc->stopRequested;
    default:
        return true;
    }
}

void vfc_jit_run(vfc_t *vfc, uint64_t budget)
{
    if (!jit_init(vfc)) {
        vfc_cpu_run(vfc, budget);
        return;
    }
    while (budget > 0 && !vfc->stopRequested) {
        if (!run_once(vfc, &budget)) break;
    }
}

uint64_t vfc_jit_step(vfc_t *vfc)
{
    vfc->stopRequested = false;
    if (!jit_init(vfc)) {
        vfc_cpu_run(vfc, 1);
        return 1;
    }
    vfc->jit->noChain = true;
    const uint64_t before = vfc->instructions;
    uint64_t budget = UINT64_C(1) << 40;
    run_once(vfc, &budget);
    return vfc->instructions - before;
}

#else

bool vfc_jit_available(void) { return false; }
void vfc_jit_run(vfc_t *vfc, uint64_t budget) { vfc_cpu_run(vfc, budget); }
uint64_t vfc_jit_step(vfc_t *vfc) { vfc->stopRequested = false; vfc_cpu_run(vfc, 1); return 1; }
void vfc_jit_free(vfc_t *vfc) { (void)vfc; }
void vfc_jit_flush(vfc_t *vfc) { (void)vfc; }

#endif
