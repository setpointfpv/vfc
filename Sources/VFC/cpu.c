// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// An ARMv7E-M (Thumb-2 with the DSP extension) and FPv4-SP interpreter,
// written from the ARMv7-M Architecture Reference Manual. There are no
// exceptions or interrupts: the virtual board has none, and anything that
// would raise one on silicon stops the core with a fault description instead.
//
// This file must be compiled with -ffp-contract=off so that separately
// rounded operations (VMLA, VMUL then VADD) are never fused by the host
// compiler; the fused ones (VFMA and friends) use fmaf explicitly.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "vfc_internal.h"

// --- Memory, with the common cases inline

static inline uint32_t load(vfc_t *vfc, uint32_t address, int size)
{
    uint32_t offset = address - VFC_RAM_BASE;
    if (offset <= VFC_RAM_SIZE - 4) {
        uint32_t value = 0;
        memcpy(&value, vfc->ram + offset, (size_t)size);
        return value;
    }
    offset = address - VFC_FLASH_BASE;
    if (offset <= VFC_FLASH_SIZE - 4) {
        uint32_t value = 0;
        memcpy(&value, vfc->flash + offset, (size_t)size);
        return value;
    }
    return vfc_bus_read(vfc, address, size);
}

static inline void store(vfc_t *vfc, uint32_t address, uint32_t value, int size)
{
    const uint32_t offset = address - VFC_RAM_BASE;
    if (offset <= VFC_RAM_SIZE - 4) {
        memcpy(vfc->ram + offset, &value, (size_t)size);
        return;
    }
    vfc_bus_write(vfc, address, value, size);
}

static inline uint16_t fetch16(vfc_t *vfc, uint32_t address)
{
    const uint32_t offset = address - VFC_FLASH_BASE;
    if (offset <= VFC_FLASH_SIZE - 2) {
        uint16_t value;
        memcpy(&value, vfc->flash + offset, 2);
        return value;
    }
    return (uint16_t)vfc_bus_read(vfc, address, 2);
}

// --- Faults

void vfc_raise_fault(vfc_t *vfc, const char *format, ...)
{
    if (vfc->stopRequested && vfc->stopReason == VFC_STOP_FAULT) {
        return;
    }
    va_list args;
    va_start(args, format);
    vsnprintf(vfc->fault, sizeof(vfc->fault), format, args);
    va_end(args);
    vfc->stopRequested = true;
    vfc->stopReason = VFC_STOP_FAULT;
}

// --- Arithmetic helpers from the ARM pseudocode

typedef struct {
    vfc_t *vfc;
    vfc_cpu_t *cpu;
    uint32_t pc;        // address of this instruction
    bool inIT;
} ctx_t;

enum { SR_LSL = 0, SR_LSR = 1, SR_ASR = 2, SR_ROR = 3, SR_RRX = 4 };

static inline uint32_t reg(const ctx_t *x, unsigned n)
{
    return n == 15 ? x->pc + 4 : x->cpu->r[n];
}

static inline uint32_t align4(uint32_t v)
{
    return v & ~3u;
}

static inline uint32_t ror32(uint32_t v, unsigned n)
{
    n &= 31;
    return n ? (v >> n) | (v << (32 - n)) : v;
}

static inline uint32_t add_with_carry(uint32_t x, uint32_t y, bool carryIn, bool *carryOut, bool *overflow)
{
    const uint64_t unsignedSum = (uint64_t)x + y + (carryIn ? 1 : 0);
    const int64_t signedSum = (int64_t)(int32_t)x + (int32_t)y + (carryIn ? 1 : 0);
    const uint32_t result = (uint32_t)unsignedSum;
    *carryOut = (unsignedSum >> 32) & 1;
    *overflow = (int64_t)(int32_t)result != signedSum;
    return result;
}

static uint32_t shift_c(uint32_t value, int type, unsigned amount, bool carryIn, bool *carryOut)
{
    *carryOut = carryIn;
    if (type == SR_RRX) {
        *carryOut = value & 1;
        return (value >> 1) | ((carryIn ? 1u : 0u) << 31);
    }
    if (amount == 0) {
        return value;
    }
    switch (type) {
    case SR_LSL:
        if (amount < 32) {
            *carryOut = (value >> (32 - amount)) & 1;
            return value << amount;
        }
        *carryOut = amount == 32 ? (value & 1) : 0;
        return 0;
    case SR_LSR:
        if (amount < 32) {
            *carryOut = (value >> (amount - 1)) & 1;
            return value >> amount;
        }
        *carryOut = amount == 32 ? (value >> 31) : 0;
        return 0;
    case SR_ASR:
        if (amount < 32) {
            *carryOut = (value >> (amount - 1)) & 1;
            return (uint32_t)((int32_t)value >> amount);
        }
        *carryOut = value >> 31;
        return (value >> 31) ? 0xFFFFFFFFu : 0;
    case SR_ROR: {
        const uint32_t result = ror32(value, amount);
        *carryOut = result >> 31;
        return result;
    }
    }
    return value;
}

static inline void decode_imm_shift(unsigned type, unsigned imm5, int *outType, unsigned *outAmount)
{
    switch (type) {
    case 0: *outType = SR_LSL; *outAmount = imm5; break;
    case 1: *outType = SR_LSR; *outAmount = imm5 ? imm5 : 32; break;
    case 2: *outType = SR_ASR; *outAmount = imm5 ? imm5 : 32; break;
    default:
        if (imm5 == 0) {
            *outType = SR_RRX;
            *outAmount = 1;
        } else {
            *outType = SR_ROR;
            *outAmount = imm5;
        }
        break;
    }
}

static uint32_t thumb_expand_imm_c(uint32_t imm12, bool carryIn, bool *carryOut)
{
    *carryOut = carryIn;
    if ((imm12 >> 10) == 0) {
        const uint32_t imm8 = imm12 & 0xFF;
        switch ((imm12 >> 8) & 3) {
        case 0: return imm8;
        case 1: return (imm8 << 16) | imm8;
        case 2: return (imm8 << 24) | (imm8 << 8);
        default: return imm8 * 0x01010101u;
        }
    }
    const uint32_t result = ror32(0x80 | (imm12 & 0x7F), (imm12 >> 7) & 0x1F);
    *carryOut = result >> 31;
    return result;
}

static inline bool condition_passed(const vfc_cpu_t *cpu, unsigned cond)
{
    bool result;
    switch (cond >> 1) {
    case 0: result = cpu->z; break;
    case 1: result = cpu->c; break;
    case 2: result = cpu->n; break;
    case 3: result = cpu->v; break;
    case 4: result = cpu->c && !cpu->z; break;
    case 5: result = cpu->n == cpu->v; break;
    case 6: result = cpu->n == cpu->v && !cpu->z; break;
    default: return true;
    }
    return (cond & 1) ? !result : result;
}

static inline void set_nz(vfc_cpu_t *cpu, uint32_t result)
{
    cpu->n = result >> 31;
    cpu->z = result == 0;
}

static inline int64_t signed_sat(int64_t value, unsigned bits, bool *saturated)
{
    const int64_t max = ((int64_t)1 << (bits - 1)) - 1;
    const int64_t min = -((int64_t)1 << (bits - 1));
    *saturated = value > max || value < min;
    return value > max ? max : value < min ? min : value;
}

static inline int64_t unsigned_sat(int64_t value, unsigned bits, bool *saturated)
{
    const int64_t max = ((int64_t)1 << bits) - 1;
    *saturated = value > max || value < 0;
    return value > max ? max : value < 0 ? 0 : value;
}

// --- PC writes

static inline void branch_write_pc(ctx_t *x, uint32_t address)
{
    x->cpu->r[15] = address & ~1u;
}

static inline void bx_write_pc(ctx_t *x, uint32_t address)
{
    if ((address & 1) == 0) {
        vfc_raise_fault(x->vfc, "INVSTATE: branch to 0x%08x (ARM state) from 0x%08x", address, x->pc);
        return;
    }
    x->cpu->r[15] = address & ~1u;
}

static inline void undefined(ctx_t *x, uint32_t inst, bool wide)
{
    if (wide) {
        vfc_raise_fault(x->vfc, "UNDEFINSTR 0x%04x %04x at 0x%08x", inst >> 16, inst & 0xFFFF, x->pc);
    } else {
        vfc_raise_fault(x->vfc, "UNDEFINSTR 0x%04x at 0x%08x", inst, x->pc);
    }
}

// --- Data processing shared by the 32-bit immediate and register forms

// op is inst[24:21]; returns false when the op is undefined.
static bool data_processing(ctx_t *x, unsigned op, bool setflags, unsigned d, unsigned n,
                            uint32_t operand, bool shifterCarry)
{
    vfc_cpu_t *cpu = x->cpu;
    const uint32_t rn = reg(x, n);
    uint32_t result;
    bool carry = shifterCarry, overflow = cpu->v;
    bool writeResult = true;

    switch (op) {
    case 0x0:   // AND / TST
        result = rn & operand;
        if (d == 15) {
            writeResult = false;
        }
        break;
    case 0x1:   // BIC
        result = rn & ~operand;
        break;
    case 0x2:   // ORR / MOV
        result = n == 15 ? operand : rn | operand;
        break;
    case 0x3:   // ORN / MVN
        result = n == 15 ? ~operand : rn | ~operand;
        break;
    case 0x4:   // EOR / TEQ
        result = rn ^ operand;
        if (d == 15) {
            writeResult = false;
        }
        break;
    case 0x8:   // ADD / CMN
        result = add_with_carry(rn, operand, false, &carry, &overflow);
        if (d == 15) {
            writeResult = false;
        }
        break;
    case 0xA:   // ADC
        result = add_with_carry(rn, operand, cpu->c, &carry, &overflow);
        break;
    case 0xB:   // SBC
        result = add_with_carry(rn, ~operand, cpu->c, &carry, &overflow);
        break;
    case 0xD:   // SUB / CMP
        result = add_with_carry(rn, ~operand, true, &carry, &overflow);
        if (d == 15) {
            writeResult = false;
        }
        break;
    case 0xE:   // RSB
        result = add_with_carry(~rn, operand, true, &carry, &overflow);
        break;
    default:
        return false;
    }

    if (writeResult) {
        if (d == 15) {
            // Only reachable through encodings that are UNPREDICTABLE here.
            branch_write_pc(x, result);
        } else {
            cpu->r[d] = result;
        }
    }
    if (setflags) {
        set_nz(cpu, result);
        cpu->c = carry;
        cpu->v = overflow;
    }
    return true;
}

// --- Load and store multiple

static void load_multiple(ctx_t *x, unsigned n, uint32_t list, uint32_t address, bool writeback, uint32_t newBase)
{
    vfc_cpu_t *cpu = x->cpu;
    uint32_t pcValue = 0;
    bool loadsPc = false;
    for (unsigned i = 0; i < 16; i++) {
        if (list & (1u << i)) {
            const uint32_t value = load(x->vfc, address, 4);
            if (i == 15) {
                pcValue = value;
                loadsPc = true;
            } else {
                cpu->r[i] = value;
            }
            address += 4;
        }
    }
    if (writeback && !(list & (1u << n))) {
        cpu->r[n] = newBase;
    }
    if (loadsPc) {
        bx_write_pc(x, pcValue);
    }
}

static void store_multiple(ctx_t *x, unsigned n, uint32_t list, uint32_t address, bool writeback, uint32_t newBase)
{
    for (unsigned i = 0; i < 16; i++) {
        if (list & (1u << i)) {
            store(x->vfc, address, reg(x, i), 4);
            address += 4;
        }
    }
    if (writeback) {
        x->cpu->r[n] = newBase;
    }
}

static inline int popcount16(uint32_t v)
{
    return __builtin_popcount(v & 0xFFFF);
}

// --- Special registers

static uint32_t read_special(ctx_t *x, unsigned sysm)
{
    vfc_cpu_t *cpu = x->cpu;
    switch (sysm) {
    case 0: case 1: case 2: case 3: case 5: case 6: case 7: {
        uint32_t value = 0;
        if (sysm < 4) {
            value = ((uint32_t)cpu->n << 31) | ((uint32_t)cpu->z << 30) | ((uint32_t)cpu->c << 29)
                  | ((uint32_t)cpu->v << 28) | ((uint32_t)cpu->q << 27) | ((uint32_t)cpu->ge << 16);
        }
        return value;   // IPSR is 0: always thread mode
    }
    case 8: return cpu->r[13];
    case 9: return cpu->psp;
    case 16: return cpu->primask & 1;
    case 17: case 18: return cpu->basepri & 0xFF;
    case 19: return cpu->faultmask & 1;
    case 20: return cpu->control & 7;
    default: return 0;
    }
}

static void write_special(ctx_t *x, unsigned sysm, unsigned mask, uint32_t value)
{
    vfc_cpu_t *cpu = x->cpu;
    switch (sysm) {
    case 0: case 1: case 2: case 3:
        if (mask & 2) {
            cpu->n = value >> 31;
            cpu->z = (value >> 30) & 1;
            cpu->c = (value >> 29) & 1;
            cpu->v = (value >> 28) & 1;
            cpu->q = (value >> 27) & 1;
        }
        if (mask & 1) {
            cpu->ge = (value >> 16) & 0xF;
        }
        break;
    case 8: cpu->r[13] = value & ~3u; break;
    case 9: cpu->psp = value & ~3u; break;
    case 16: cpu->primask = value & 1; break;
    case 17: cpu->basepri = value & 0xFF; break;
    case 18:
        value &= 0xFF;
        if (value != 0 && (value < cpu->basepri || cpu->basepri == 0)) {
            cpu->basepri = value;
        }
        break;
    case 19: cpu->faultmask = value & 1; break;
    case 20: cpu->control = value & 7; break;
    default: break;
    }
}

// --- Sleep

static void wait_for_interrupt(ctx_t *x)
{
    x->vfc->stopRequested = true;
    x->vfc->stopReason = VFC_STOP_IDLE;
}

// --- 16-bit instructions

static void exec16(ctx_t *x, uint32_t hw)
{
    vfc_cpu_t *cpu = x->cpu;
    vfc_t *vfc = x->vfc;
    const bool setflags = !x->inIT;
    bool carry, overflow;

    switch (hw >> 11) {
    case 0x00: case 0x01: case 0x02: {     // LSL, LSR, ASR (immediate)
        const unsigned d = hw & 7, m = (hw >> 3) & 7, imm5 = (hw >> 6) & 0x1F;
        int type;
        unsigned amount;
        decode_imm_shift(hw >> 11, imm5, &type, &amount);
        const uint32_t result = shift_c(cpu->r[m], type, amount, cpu->c, &carry);
        cpu->r[d] = result;
        if (setflags) {
            set_nz(cpu, result);
            cpu->c = carry;
        }
        return;
    }
    case 0x03: {                            // ADD/SUB register or 3-bit immediate
        const unsigned d = hw & 7, n = (hw >> 3) & 7, rmOrImm = (hw >> 6) & 7;
        const uint32_t operand = (hw & 0x400) ? rmOrImm : cpu->r[rmOrImm];
        uint32_t result;
        if (hw & 0x200) {
            result = add_with_carry(cpu->r[n], ~operand, true, &carry, &overflow);
        } else {
            result = add_with_carry(cpu->r[n], operand, false, &carry, &overflow);
        }
        cpu->r[d] = result;
        if (setflags) {
            set_nz(cpu, result);
            cpu->c = carry;
            cpu->v = overflow;
        }
        return;
    }
    case 0x04: {                            // MOV immediate
        const unsigned d = (hw >> 8) & 7;
        cpu->r[d] = hw & 0xFF;
        if (setflags) {
            set_nz(cpu, cpu->r[d]);
        }
        return;
    }
    case 0x05: {                            // CMP immediate
        const uint32_t result = add_with_carry(cpu->r[(hw >> 8) & 7], ~(hw & 0xFFu), true, &carry, &overflow);
        set_nz(cpu, result);
        cpu->c = carry;
        cpu->v = overflow;
        return;
    }
    case 0x06: case 0x07: {                 // ADD / SUB 8-bit immediate
        const unsigned dn = (hw >> 8) & 7;
        const uint32_t imm = hw & 0xFF;
        const uint32_t result = (hw & 0x800)
            ? add_with_carry(cpu->r[dn], ~imm, true, &carry, &overflow)
            : add_with_carry(cpu->r[dn], imm, false, &carry, &overflow);
        cpu->r[dn] = result;
        if (setflags) {
            set_nz(cpu, result);
            cpu->c = carry;
            cpu->v = overflow;
        }
        return;
    }
    case 0x08: {
        if ((hw & 0x400) == 0) {            // data processing (register)
            const unsigned dn = hw & 7, m = (hw >> 3) & 7;
            const uint32_t a = cpu->r[dn], b = cpu->r[m];
            uint32_t result = 0;
            bool write = true;
            carry = cpu->c;
            overflow = cpu->v;
            switch ((hw >> 6) & 0xF) {
            case 0x0: result = a & b; break;                                           // AND
            case 0x1: result = a ^ b; break;                                           // EOR
            case 0x2: result = shift_c(a, SR_LSL, b & 0xFF, cpu->c, &carry); break;    // LSL
            case 0x3: result = shift_c(a, SR_LSR, b & 0xFF, cpu->c, &carry); break;    // LSR
            case 0x4: result = shift_c(a, SR_ASR, b & 0xFF, cpu->c, &carry); break;    // ASR
            case 0x5: result = add_with_carry(a, b, cpu->c, &carry, &overflow); break; // ADC
            case 0x6: result = add_with_carry(a, ~b, cpu->c, &carry, &overflow); break;// SBC
            case 0x7: result = shift_c(a, SR_ROR, b & 0xFF, cpu->c, &carry); break;    // ROR
            case 0x8: result = a & b; write = false;                                    // TST
                set_nz(cpu, result);
                return;
            case 0x9: result = add_with_carry(~b, 0, true, &carry, &overflow); break;  // RSB #0
            case 0xA: result = add_with_carry(a, ~b, true, &carry, &overflow);         // CMP
                set_nz(cpu, result);
                cpu->c = carry;
                cpu->v = overflow;
                return;
            case 0xB: result = add_with_carry(a, b, false, &carry, &overflow);         // CMN
                set_nz(cpu, result);
                cpu->c = carry;
                cpu->v = overflow;
                return;
            case 0xC: result = a | b; break;                                           // ORR
            case 0xD: result = a * b; break;                                           // MUL
            case 0xE: result = a & ~b; break;                                          // BIC
            case 0xF: result = ~b; break;                                              // MVN
            }
            if (((hw >> 6) & 0xF) == 0x9) {
                cpu->r[dn] = result;    // RSB writes Rd = hw[2:0]
            } else if (write) {
                cpu->r[dn] = result;
            }
            if (setflags) {
                set_nz(cpu, result);
                cpu->c = carry;
                cpu->v = overflow;
            }
            return;
        }
        // Special data processing and branch/exchange.
        const unsigned op = (hw >> 8) & 3;
        const unsigned m = (hw >> 3) & 0xF;
        const unsigned dn = (hw & 7) | ((hw >> 4) & 8);
        switch (op) {
        case 0: {                           // ADD (register), high registers
            const uint32_t result = reg(x, dn) + reg(x, m);
            if (dn == 15) {
                branch_write_pc(x, result);
            } else {
                cpu->r[dn] = result;
            }
            return;
        }
        case 1: {                           // CMP (register), high registers
            const uint32_t result = add_with_carry(reg(x, dn), ~reg(x, m), true, &carry, &overflow);
            set_nz(cpu, result);
            cpu->c = carry;
            cpu->v = overflow;
            return;
        }
        case 2: {                           // MOV (register)
            const uint32_t value = reg(x, m);
            if (dn == 15) {
                branch_write_pc(x, value);
            } else {
                cpu->r[dn] = value;
            }
            return;
        }
        default:
            if (hw & 0x80) {                // BLX (register)
                const uint32_t target = reg(x, m);
                cpu->r[14] = (x->pc + 2) | 1;
                bx_write_pc(x, target);
            } else {                        // BX
                bx_write_pc(x, reg(x, m));
            }
            return;
        }
    }
    case 0x09: {                            // LDR (literal)
        const unsigned t = (hw >> 8) & 7;
        cpu->r[t] = load(vfc, align4(x->pc + 4) + ((hw & 0xFF) << 2), 4);
        return;
    }
    case 0x0A: case 0x0B: {                 // load/store register offset
        const unsigned t = hw & 7, n = (hw >> 3) & 7, m = (hw >> 6) & 7;
        const uint32_t address = cpu->r[n] + cpu->r[m];
        switch ((hw >> 9) & 7) {
        case 0: store(vfc, address, cpu->r[t], 4); break;                             // STR
        case 1: store(vfc, address, cpu->r[t], 2); break;                             // STRH
        case 2: store(vfc, address, cpu->r[t], 1); break;                             // STRB
        case 3: cpu->r[t] = (uint32_t)(int32_t)(int8_t)load(vfc, address, 1); break;  // LDRSB
        case 4: cpu->r[t] = load(vfc, address, 4); break;                             // LDR
        case 5: cpu->r[t] = load(vfc, address, 2); break;                             // LDRH
        case 6: cpu->r[t] = load(vfc, address, 1); break;                             // LDRB
        case 7: cpu->r[t] = (uint32_t)(int32_t)(int16_t)load(vfc, address, 2); break; // LDRSH
        }
        return;
    }
    case 0x0C: case 0x0D: {                 // STR/LDR immediate (word)
        const unsigned t = hw & 7, n = (hw >> 3) & 7;
        const uint32_t address = cpu->r[n] + (((hw >> 6) & 0x1F) << 2);
        if (hw & 0x800) {
            cpu->r[t] = load(vfc, address, 4);
        } else {
            store(vfc, address, cpu->r[t], 4);
        }
        return;
    }
    case 0x0E: case 0x0F: {                 // STRB/LDRB immediate
        const unsigned t = hw & 7, n = (hw >> 3) & 7;
        const uint32_t address = cpu->r[n] + ((hw >> 6) & 0x1F);
        if (hw & 0x800) {
            cpu->r[t] = load(vfc, address, 1);
        } else {
            store(vfc, address, cpu->r[t], 1);
        }
        return;
    }
    case 0x10: case 0x11: {                 // STRH/LDRH immediate
        const unsigned t = hw & 7, n = (hw >> 3) & 7;
        const uint32_t address = cpu->r[n] + (((hw >> 6) & 0x1F) << 1);
        if (hw & 0x800) {
            cpu->r[t] = load(vfc, address, 2);
        } else {
            store(vfc, address, cpu->r[t], 2);
        }
        return;
    }
    case 0x12: case 0x13: {                 // STR/LDR SP-relative
        const unsigned t = (hw >> 8) & 7;
        const uint32_t address = cpu->r[13] + ((hw & 0xFF) << 2);
        if (hw & 0x800) {
            cpu->r[t] = load(vfc, address, 4);
        } else {
            store(vfc, address, cpu->r[t], 4);
        }
        return;
    }
    case 0x14:                              // ADR
        cpu->r[(hw >> 8) & 7] = align4(x->pc + 4) + ((hw & 0xFF) << 2);
        return;
    case 0x15:                              // ADD Rd, SP, #imm
        cpu->r[(hw >> 8) & 7] = cpu->r[13] + ((hw & 0xFF) << 2);
        return;
    case 0x16: case 0x17:                   // miscellaneous
        if ((hw & 0xFF00) == 0xB000) {      // ADD/SUB SP, SP, #imm
            const uint32_t imm = (hw & 0x7F) << 2;
            cpu->r[13] = (hw & 0x80) ? cpu->r[13] - imm : cpu->r[13] + imm;
            return;
        }
        if ((hw & 0xF500) == 0xB100) {      // CBZ, CBNZ
            const unsigned n = hw & 7;
            const uint32_t imm = (((hw >> 9) & 1) << 6) | (((hw >> 3) & 0x1F) << 1);
            const bool nonzero = hw & 0x800;
            if ((cpu->r[n] != 0) == nonzero) {
                branch_write_pc(x, x->pc + 4 + imm);
            }
            return;
        }
        if ((hw & 0xFF00) == 0xB200) {      // SXTH, SXTB, UXTH, UXTB
            const unsigned d = hw & 7, m = (hw >> 3) & 7;
            const uint32_t v = cpu->r[m];
            switch ((hw >> 6) & 3) {
            case 0: cpu->r[d] = (uint32_t)(int32_t)(int16_t)v; break;
            case 1: cpu->r[d] = (uint32_t)(int32_t)(int8_t)v; break;
            case 2: cpu->r[d] = v & 0xFFFF; break;
            case 3: cpu->r[d] = v & 0xFF; break;
            }
            return;
        }
        if ((hw & 0xFE00) == 0xB400) {      // PUSH
            const uint32_t list = (hw & 0xFF) | ((hw & 0x100) ? (1u << 14) : 0);
            const uint32_t address = cpu->r[13] - 4 * (uint32_t)popcount16(list);
            store_multiple(x, 13, list, address, true, address);
            return;
        }
        if ((hw & 0xFFE8) == 0xB660) {      // CPS
            if (hw & 1) {
                cpu->faultmask = (hw & 0x10) ? 1 : 0;
            }
            if (hw & 2) {
                cpu->primask = (hw & 0x10) ? 1 : 0;
            }
            return;
        }
        if ((hw & 0xFF00) == 0xBA00) {      // REV, REV16, REVSH
            const unsigned d = hw & 7, m = (hw >> 3) & 7;
            const uint32_t v = cpu->r[m];
            switch ((hw >> 6) & 3) {
            case 0: cpu->r[d] = __builtin_bswap32(v); return;
            case 1: cpu->r[d] = ((v & 0x00FF00FFu) << 8) | ((v & 0xFF00FF00u) >> 8); return;
            case 3: cpu->r[d] = (uint32_t)(int32_t)(int16_t)(((v & 0xFF) << 8) | ((v >> 8) & 0xFF)); return;
            default: break;
            }
            undefined(x, hw, false);
            return;
        }
        if ((hw & 0xFE00) == 0xBC00) {      // POP
            const uint32_t list = (hw & 0xFF) | ((hw & 0x100) ? (1u << 15) : 0);
            const uint32_t address = cpu->r[13];
            load_multiple(x, 13, list, address, true, address + 4 * (uint32_t)popcount16(list));
            return;
        }
        if ((hw & 0xFF00) == 0xBE00) {      // BKPT
            vfc_raise_fault(vfc, "BKPT #%u at 0x%08x", hw & 0xFF, x->pc);
            return;
        }
        if ((hw & 0xFF00) == 0xBF00) {
            if (hw & 0xF) {                 // IT
                cpu->itstate = hw & 0xFF;
                return;
            }
            switch ((hw >> 4) & 0xF) {      // hints
            case 3: wait_for_interrupt(x); return;
            default: return;                // NOP, YIELD, WFE, SEV
            }
        }
        undefined(x, hw, false);
        return;
    case 0x18: {                            // STM (STMIA Rn!)
        const unsigned n = (hw >> 8) & 7;
        const uint32_t list = hw & 0xFF;
        store_multiple(x, n, list, cpu->r[n], true, cpu->r[n] + 4 * (uint32_t)popcount16(list));
        return;
    }
    case 0x19: {                            // LDM (LDMIA Rn{!})
        const unsigned n = (hw >> 8) & 7;
        const uint32_t list = hw & 0xFF;
        load_multiple(x, n, list, cpu->r[n], !(list & (1u << n)), cpu->r[n] + 4 * (uint32_t)popcount16(list));
        return;
    }
    case 0x1A: case 0x1B: {                 // B<c>, UDF, SVC
        const unsigned cond = (hw >> 8) & 0xF;
        if (cond == 0xE) {
            vfc_raise_fault(vfc, "UDF #%u at 0x%08x", hw & 0xFF, x->pc);
            return;
        }
        if (cond == 0xF) {
            vfc_raise_fault(vfc, "SVC #%u at 0x%08x", hw & 0xFF, x->pc);
            return;
        }
        if (condition_passed(cpu, cond)) {
            branch_write_pc(x, x->pc + 4 + (uint32_t)((int32_t)(int8_t)(hw & 0xFF) * 2));
        }
        return;
    }
    case 0x1C: {                            // B (unconditional)
        const int32_t imm = ((int32_t)((hw & 0x7FF) << 21)) >> 20;
        branch_write_pc(x, x->pc + 4 + (uint32_t)imm);
        return;
    }
    default:
        undefined(x, hw, false);
        return;
    }
}

// --- 32-bit instructions: load/store single

static void load_store_single(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    vfc_t *vfc = x->vfc;
    const unsigned size = (hw1 >> 5) & 3;       // 0 byte, 1 halfword, 2 word
    const bool isLoad = hw1 & 0x10;
    const bool signExtend = hw1 & 0x100;
    const unsigned n = hw1 & 0xF, t = hw2 >> 12;
    uint32_t address;
    bool writeback = false;
    uint32_t newBase = 0;

    if (size == 3) {
        undefined(x, (hw1 << 16) | hw2, true);
        return;
    }
    if (n == 15) {
        if (!isLoad) {
            undefined(x, (hw1 << 16) | hw2, true);
            return;
        }
        const uint32_t imm12 = hw2 & 0xFFF;
        const uint32_t base = align4(x->pc + 4);
        address = (hw1 & 0x80) ? base + imm12 : base - imm12;
    } else if (hw1 & 0x80) {
        address = cpu->r[n] + (hw2 & 0xFFF);
    } else if (hw2 & 0x800) {
        const uint32_t imm8 = hw2 & 0xFF;
        const bool index = hw2 & 0x400, add = hw2 & 0x200, wback = hw2 & 0x100;
        const uint32_t offsetAddress = add ? cpu->r[n] + imm8 : cpu->r[n] - imm8;
        if ((hw2 & 0xF00) == 0xE00) {
            address = offsetAddress;        // LDRT/STRT: unprivileged, the same here
        } else {
            if (!index && !wback) {
                undefined(x, (hw1 << 16) | hw2, true);
                return;
            }
            address = index ? offsetAddress : cpu->r[n];
            writeback = wback;
            newBase = offsetAddress;
        }
    } else if ((hw2 & 0xFC0) == 0) {
        address = cpu->r[n] + (cpu->r[hw2 & 0xF] << ((hw2 >> 4) & 3));
    } else {
        undefined(x, (hw1 << 16) | hw2, true);
        return;
    }

    if (isLoad) {
        if (t == 15 && size != 2) {
            return;                         // PLD, PLI: hints
        }
        uint32_t value;
        switch (size) {
        case 0: value = load(vfc, address, 1); if (signExtend) value = (uint32_t)(int32_t)(int8_t)value; break;
        case 1: value = load(vfc, address, 2); if (signExtend) value = (uint32_t)(int32_t)(int16_t)value; break;
        default: value = load(vfc, address, 4); break;
        }
        if (writeback) {
            cpu->r[n] = newBase;
        }
        if (t == 15) {
            bx_write_pc(x, value);
        } else {
            cpu->r[t] = value;
        }
    } else {
        store(vfc, address, reg(x, t), size == 0 ? 1 : size == 1 ? 2 : 4);
        if (writeback) {
            cpu->r[n] = newBase;
        }
    }
}

// --- 32-bit: load/store dual, exclusive, table branch

static void load_store_dual(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    vfc_t *vfc = x->vfc;
    const unsigned op1 = (hw1 >> 7) & 3, op2 = (hw1 >> 4) & 3, op3 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, t = hw2 >> 12, t2 = (hw2 >> 8) & 0xF;

    if (op1 == 0 && op2 == 0) {             // STREX: always succeeds
        store(vfc, cpu->r[n] + ((hw2 & 0xFF) << 2), cpu->r[t], 4);
        cpu->r[t2] = 0;
        return;
    }
    if (op1 == 0 && op2 == 1) {             // LDREX
        cpu->r[t] = load(vfc, cpu->r[n] + ((hw2 & 0xFF) << 2), 4);
        return;
    }
    if (op1 == 1 && op2 == 0) {             // STREXB, STREXH
        const unsigned d = hw2 & 0xF;
        if (op3 == 4) {
            store(vfc, cpu->r[n], cpu->r[t], 1);
        } else if (op3 == 5) {
            store(vfc, cpu->r[n], cpu->r[t], 2);
        } else {
            undefined(x, (hw1 << 16) | hw2, true);
            return;
        }
        cpu->r[d] = 0;
        return;
    }
    if (op1 == 1 && op2 == 1) {
        const unsigned m = hw2 & 0xF;
        switch (op3) {
        case 0: {                           // TBB
            const uint32_t offset = load(vfc, reg(x, n) + cpu->r[m], 1);
            branch_write_pc(x, x->pc + 4 + 2 * offset);
            return;
        }
        case 1: {                           // TBH
            const uint32_t offset = load(vfc, reg(x, n) + (cpu->r[m] << 1), 2);
            branch_write_pc(x, x->pc + 4 + 2 * offset);
            return;
        }
        case 4: cpu->r[t] = load(vfc, cpu->r[n], 1); return;   // LDREXB
        case 5: cpu->r[t] = load(vfc, cpu->r[n], 2); return;   // LDREXH
        default:
            undefined(x, (hw1 << 16) | hw2, true);
            return;
        }
    }

    // LDRD / STRD (immediate, or literal for loads).
    const bool index = hw1 & 0x100, add = hw1 & 0x80, wback = hw1 & 0x20, isLoad = hw1 & 0x10;
    const uint32_t imm = (hw2 & 0xFF) << 2;
    const uint32_t base = n == 15 ? align4(x->pc + 4) : cpu->r[n];
    const uint32_t offsetAddress = add ? base + imm : base - imm;
    const uint32_t address = index ? offsetAddress : base;
    if (isLoad) {
        const uint32_t lo = load(vfc, address, 4), hi = load(vfc, address + 4, 4);
        cpu->r[t] = lo;
        cpu->r[t2] = hi;
    } else {
        store(vfc, address, cpu->r[t], 4);
        store(vfc, address + 4, cpu->r[t2], 4);
    }
    if (wback) {
        cpu->r[n] = offsetAddress;
    }
}

// --- 32-bit: data processing (plain binary immediate)

static void plain_immediate(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    const unsigned op = (hw1 >> 4) & 0x1F;
    const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF;
    const uint32_t imm12 = (((hw1 >> 10) & 1) << 11) | (((hw2 >> 12) & 7) << 8) | (hw2 & 0xFF);
    const unsigned imm5 = (((hw2 >> 12) & 7) << 2) | ((hw2 >> 6) & 3);
    const unsigned field = hw2 & 0x1F;

    switch (op) {
    case 0x00:                              // ADDW, ADR
        cpu->r[d] = n == 15 ? align4(x->pc + 4) + imm12 : cpu->r[n] + imm12;
        return;
    case 0x0A:                              // SUBW, ADR (subtract)
        cpu->r[d] = n == 15 ? align4(x->pc + 4) - imm12 : cpu->r[n] - imm12;
        return;
    case 0x04:                              // MOVW
        cpu->r[d] = ((hw1 & 0xF) << 12) | imm12;
        return;
    case 0x0C:                              // MOVT
        cpu->r[d] = (cpu->r[d] & 0xFFFF) | ((((hw1 & 0xF) << 12) | imm12) << 16);
        return;
    case 0x10: case 0x12: {                 // SSAT
        if (op == 0x12 && imm5 == 0) {
            break;                          // SSAT16: not implemented
        }
        const int32_t operand = op == 0x12 ? (int32_t)cpu->r[n] >> (imm5 ? imm5 : 31)
                                           : (int32_t)(cpu->r[n] << imm5);
        bool saturated;
        cpu->r[d] = (uint32_t)(int32_t)signed_sat(operand, field + 1, &saturated);
        if (saturated) {
            cpu->q = true;
        }
        return;
    }
    case 0x18: case 0x1A: {                 // USAT
        if (op == 0x1A && imm5 == 0) {
            break;                          // USAT16: not implemented
        }
        const int32_t operand = op == 0x1A ? (int32_t)cpu->r[n] >> (imm5 ? imm5 : 31)
                                           : (int32_t)(cpu->r[n] << imm5);
        bool saturated;
        cpu->r[d] = (uint32_t)unsigned_sat(operand, field, &saturated);
        if (saturated) {
            cpu->q = true;
        }
        return;
    }
    case 0x14: {                            // SBFX
        const unsigned width = field + 1;
        if (imm5 + width > 32) {
            break;
        }
        cpu->r[d] = (uint32_t)((int32_t)(cpu->r[n] << (32 - imm5 - width)) >> (32 - width));
        return;
    }
    case 0x1C: {                            // UBFX
        const unsigned width = field + 1;
        if (imm5 + width > 32) {
            break;
        }
        cpu->r[d] = (cpu->r[n] >> imm5) & (width == 32 ? 0xFFFFFFFFu : ((1u << width) - 1));
        return;
    }
    case 0x16: {                            // BFI, BFC
        const unsigned lsb = imm5, msb = field;
        if (msb < lsb) {
            break;
        }
        const unsigned width = msb - lsb + 1;
        const uint32_t mask = (width == 32 ? 0xFFFFFFFFu : ((1u << width) - 1)) << lsb;
        const uint32_t source = n == 15 ? 0 : cpu->r[n] << lsb;
        cpu->r[d] = (cpu->r[d] & ~mask) | (source & mask);
        return;
    }
    default:
        break;
    }
    undefined(x, (hw1 << 16) | hw2, true);
}

// --- 32-bit: data processing (register)

static void data_register(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    const unsigned op1 = (hw1 >> 4) & 0xF, op2 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;

    if (op2 == 0 && (op1 & 8) == 0) {       // LSL, LSR, ASR, ROR (register)
        bool carry;
        const uint32_t result = shift_c(cpu->r[n], (op1 >> 1) & 3, cpu->r[m] & 0xFF, cpu->c, &carry);
        cpu->r[d] = result;
        if (op1 & 1) {
            set_nz(cpu, result);
            cpu->c = carry;
        }
        return;
    }
    if ((op1 & 8) == 0 && (op2 & 8)) {      // extend (and add)
        const uint32_t rotated = ror32(cpu->r[m], ((hw2 >> 4) & 3) * 8);
        const uint32_t addend = n == 15 ? 0 : cpu->r[n];
        switch (op1) {
        case 0: cpu->r[d] = addend + (uint32_t)(int32_t)(int16_t)rotated; return;   // SXTAH, SXTH
        case 1: cpu->r[d] = addend + (rotated & 0xFFFF); return;                    // UXTAH, UXTH
        case 4: cpu->r[d] = addend + (uint32_t)(int32_t)(int8_t)rotated; return;    // SXTAB, SXTB
        case 5: cpu->r[d] = addend + (rotated & 0xFF); return;                      // UXTAB, UXTB
        case 3: {                                                                   // UXTAB16, UXTB16
            const uint32_t lo = ((n == 15 ? 0 : cpu->r[n]) & 0xFFFF) + (rotated & 0xFF);
            const uint32_t hi = ((n == 15 ? 0 : cpu->r[n]) >> 16) + ((rotated >> 16) & 0xFF);
            cpu->r[d] = (lo & 0xFFFF) | (hi << 16);
            return;
        }
        case 2: {                                                                   // SXTAB16, SXTB16
            const uint32_t base = n == 15 ? 0 : cpu->r[n];
            const uint32_t lo = (base & 0xFFFF) + (uint32_t)(int32_t)(int8_t)rotated;
            const uint32_t hi = (base >> 16) + (uint32_t)(int32_t)(int8_t)(rotated >> 16);
            cpu->r[d] = (lo & 0xFFFF) | (hi << 16);
            return;
        }
        default:
            break;
        }
    } else if ((op1 & 8) && (op2 & 0xC) == 0x4) {   // parallel add/sub, unsigned
        const uint32_t a = cpu->r[n], b = cpu->r[m];
        const unsigned op = op1 & 7, kind = op2 & 3;
        if (kind == 0 && op == 0) {         // UADD8
            uint32_t result = 0;
            uint8_t ge = 0;
            for (int i = 0; i < 4; i++) {
                const uint32_t sum = ((a >> (8 * i)) & 0xFF) + ((b >> (8 * i)) & 0xFF);
                result |= (sum & 0xFF) << (8 * i);
                if (sum >= 0x100) {
                    ge |= 1 << i;
                }
            }
            cpu->r[d] = result;
            cpu->ge = ge;
            return;
        }
        if (kind == 0 && op == 4) {         // USUB8
            uint32_t result = 0;
            uint8_t ge = 0;
            for (int i = 0; i < 4; i++) {
                const int32_t diff = (int32_t)((a >> (8 * i)) & 0xFF) - (int32_t)((b >> (8 * i)) & 0xFF);
                result |= ((uint32_t)diff & 0xFF) << (8 * i);
                if (diff >= 0) {
                    ge |= 1 << i;
                }
            }
            cpu->r[d] = result;
            cpu->ge = ge;
            return;
        }
        if (kind == 0 && op == 1) {         // UADD16
            const uint32_t lo = (a & 0xFFFF) + (b & 0xFFFF), hi = (a >> 16) + (b >> 16);
            cpu->r[d] = (lo & 0xFFFF) | (hi << 16);
            cpu->ge = (uint8_t)((lo >= 0x10000 ? 3 : 0) | (hi >= 0x10000 ? 0xC : 0));
            return;
        }
        if (kind == 0 && op == 5) {         // USUB16
            const int32_t lo = (int32_t)(a & 0xFFFF) - (int32_t)(b & 0xFFFF);
            const int32_t hi = (int32_t)(a >> 16) - (int32_t)(b >> 16);
            cpu->r[d] = ((uint32_t)lo & 0xFFFF) | ((uint32_t)hi << 16);
            cpu->ge = (uint8_t)((lo >= 0 ? 3 : 0) | (hi >= 0 ? 0xC : 0));
            return;
        }
    } else if ((op1 & 0xC) == 0x8 && (op2 & 0xC) == 0x8) {  // miscellaneous
        const uint32_t v = cpu->r[m];
        switch (((op1 & 3) << 2) | (op2 & 3)) {
        case 0x4: cpu->r[d] = __builtin_bswap32(v); return;                                  // REV
        case 0x5: cpu->r[d] = ((v & 0x00FF00FFu) << 8) | ((v & 0xFF00FF00u) >> 8); return;   // REV16
        case 0x6: {                                                                          // RBIT
            uint32_t r = 0;
            for (int i = 0; i < 32; i++) {
                r |= ((v >> i) & 1) << (31 - i);
            }
            cpu->r[d] = r;
            return;
        }
        case 0x7: cpu->r[d] = (uint32_t)(int32_t)(int16_t)(((v & 0xFF) << 8) | ((v >> 8) & 0xFF)); return; // REVSH
        case 0x8: {                                                                          // SEL
            const uint32_t a = cpu->r[n];
            uint32_t r = 0;
            for (int i = 0; i < 4; i++) {
                const uint32_t lane = 0xFFu << (8 * i);
                r |= ((cpu->ge >> i) & 1) ? (a & lane) : (v & lane);
            }
            cpu->r[d] = r;
            return;
        }
        case 0xC: cpu->r[d] = v ? (uint32_t)__builtin_clz(v) : 32; return;                   // CLZ
        case 0x0: case 0x2: {                                                                // QADD, QSUB
            bool saturated;
            const int64_t a = (int32_t)v, b = (int32_t)cpu->r[n];
            cpu->r[d] = (uint32_t)(int32_t)signed_sat(((op2 & 3) == 0) ? a + b : a - b, 32, &saturated);
            if (saturated) {
                cpu->q = true;
            }
            return;
        }
        default:
            break;
        }
    }
    undefined(x, (hw1 << 16) | hw2, true);
}

// --- 32-bit: multiply, long multiply, divide

static inline int32_t half(uint32_t v, bool top)
{
    return top ? (int32_t)(int16_t)(v >> 16) : (int32_t)(int16_t)v;
}

static void multiply(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    const unsigned op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 3;
    const unsigned n = hw1 & 0xF, a = hw2 >> 12, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
    const uint32_t rn = cpu->r[n], rm = cpu->r[m];

    switch (op1) {
    case 0:
        if (op2 == 0) {                     // MLA, MUL
            cpu->r[d] = rn * rm + (a == 15 ? 0 : cpu->r[a]);
            return;
        }
        if (op2 == 1) {                     // MLS
            cpu->r[d] = cpu->r[a] - rn * rm;
            return;
        }
        break;
    case 1: {                               // SMLAxy, SMULxy
        const int32_t product = half(rn, hw2 & 0x20) * half(rm, hw2 & 0x10);
        if (a == 15) {
            cpu->r[d] = (uint32_t)product;
        } else {
            const int64_t sum = (int64_t)product + (int32_t)cpu->r[a];
            cpu->r[d] = (uint32_t)sum;
            if (sum != (int32_t)sum) {
                cpu->q = true;
            }
        }
        return;
    }
    case 2: {                               // SMLAD, SMUAD
        const bool swap = hw2 & 0x10;
        const uint32_t operand = swap ? ror32(rm, 16) : rm;
        const int64_t sum = (int64_t)half(rn, false) * half(operand, false)
                          + (int64_t)half(rn, true) * half(operand, true)
                          + (a == 15 ? 0 : (int32_t)cpu->r[a]);
        cpu->r[d] = (uint32_t)sum;
        if (sum != (int32_t)sum) {
            cpu->q = true;
        }
        return;
    }
    case 3: {                               // SMLAWy, SMULWy
        const int64_t product = (int64_t)(int32_t)rn * half(rm, hw2 & 0x10);
        const int64_t result = (product >> 16) + (a == 15 ? 0 : (int32_t)cpu->r[a]);
        cpu->r[d] = (uint32_t)result;
        if (a != 15 && result != (int32_t)result) {
            cpu->q = true;
        }
        return;
    }
    case 5: {                               // SMMLA, SMMUL
        int64_t result = (int64_t)(int32_t)rn * (int32_t)rm;
        if (a != 15) {
            result += (int64_t)((uint64_t)cpu->r[a] << 32);
        }
        if (hw2 & 0x10) {
            result += 0x80000000LL;
        }
        cpu->r[d] = (uint32_t)(result >> 32);
        return;
    }
    case 6: {                               // SMMLS
        int64_t result = (int64_t)((uint64_t)cpu->r[a] << 32) - (int64_t)(int32_t)rn * (int32_t)rm;
        if (hw2 & 0x10) {
            result += 0x80000000LL;
        }
        cpu->r[d] = (uint32_t)(result >> 32);
        return;
    }
    default:
        break;
    }
    undefined(x, (hw1 << 16) | hw2, true);
}

static void long_multiply(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    const unsigned op1 = (hw1 >> 4) & 7, op2 = (hw2 >> 4) & 0xF;
    const unsigned n = hw1 & 0xF, lo = hw2 >> 12, hi = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
    const uint32_t rn = cpu->r[n], rm = cpu->r[m];

    if (op1 == 0 && op2 == 0) {             // SMULL
        const int64_t r = (int64_t)(int32_t)rn * (int32_t)rm;
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)((uint64_t)r >> 32);
        return;
    }
    if (op1 == 1 && op2 == 0xF) {           // SDIV
        const int32_t a = (int32_t)rn, b = (int32_t)rm;
        cpu->r[hi] = b == 0 ? 0 : (a == INT32_MIN && b == -1) ? (uint32_t)INT32_MIN : (uint32_t)(a / b);
        return;
    }
    if (op1 == 2 && op2 == 0) {             // UMULL
        const uint64_t r = (uint64_t)rn * rm;
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)(r >> 32);
        return;
    }
    if (op1 == 3 && op2 == 0xF) {           // UDIV
        cpu->r[hi] = rm == 0 ? 0 : rn / rm;
        return;
    }
    if (op1 == 4 && op2 == 0) {             // SMLAL
        const uint64_t acc = ((uint64_t)cpu->r[hi] << 32) | cpu->r[lo];
        const uint64_t r = acc + (uint64_t)((int64_t)(int32_t)rn * (int32_t)rm);
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)(r >> 32);
        return;
    }
    if (op1 == 4 && (op2 & 0xC) == 0x8) {   // SMLALxy
        const uint64_t acc = ((uint64_t)cpu->r[hi] << 32) | cpu->r[lo];
        const uint64_t r = acc + (uint64_t)(int64_t)(half(rn, op2 & 2) * half(rm, op2 & 1));
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)(r >> 32);
        return;
    }
    if (op1 == 6 && op2 == 0) {             // UMLAL
        const uint64_t acc = ((uint64_t)cpu->r[hi] << 32) | cpu->r[lo];
        const uint64_t r = acc + (uint64_t)rn * rm;
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)(r >> 32);
        return;
    }
    if (op1 == 6 && op2 == 6) {             // UMAAL
        const uint64_t r = (uint64_t)rn * rm + cpu->r[hi] + cpu->r[lo];
        cpu->r[lo] = (uint32_t)r;
        cpu->r[hi] = (uint32_t)(r >> 32);
        return;
    }
    undefined(x, (hw1 << 16) | hw2, true);
}

// --- 32-bit: branches and miscellaneous control

static void branch_misc(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    const uint32_t s = (hw1 >> 10) & 1, j1 = (hw2 >> 13) & 1, j2 = (hw2 >> 11) & 1;

    switch (hw2 & 0x5000) {
    case 0x0000: {
        const unsigned op = (hw1 >> 4) & 0x7F;
        if ((op & 0x38) != 0x38) {          // B<c>.W
            const uint32_t imm = (s << 20) | (j2 << 19) | (j1 << 18) | ((hw1 & 0x3F) << 12) | ((hw2 & 0x7FF) << 1);
            const int32_t offset = (int32_t)(imm << 11) >> 11;
            if (condition_passed(cpu, (hw1 >> 6) & 0xF)) {
                branch_write_pc(x, x->pc + 4 + (uint32_t)offset);
            }
            return;
        }
        switch (op) {
        case 0x38: case 0x39:               // MSR
            write_special(x, hw2 & 0xFF, (hw2 >> 10) & 3, cpu->r[hw1 & 0xF]);
            return;
        case 0x3A:                          // hints
            if ((hw2 & 0xFF) == 3) {
                wait_for_interrupt(x);
            }
            return;
        case 0x3B:                          // DSB, DMB, ISB, CLREX
            return;
        case 0x3E: case 0x3F:               // MRS
            cpu->r[(hw2 >> 8) & 0xF] = read_special(x, hw2 & 0xFF);
            return;
        case 0x7F:
            break;                          // UDF.W
        default:
            break;
        }
        break;
    }
    case 0x1000: case 0x5000: {             // B.W, BL
        const uint32_t i1 = !(j1 ^ s), i2 = !(j2 ^ s);
        const uint32_t imm = (s << 24) | (i1 << 23) | (i2 << 22) | ((hw1 & 0x3FF) << 12) | ((hw2 & 0x7FF) << 1);
        const int32_t offset = (int32_t)(imm << 7) >> 7;
        if (hw2 & 0x4000) {
            cpu->r[14] = (x->pc + 4) | 1;
        }
        branch_write_pc(x, x->pc + 4 + (uint32_t)offset);
        return;
    }
    default:
        break;
    }
    undefined(x, (hw1 << 16) | hw2, true);
}

// --- FPv4-SP

static inline float sreg(const vfc_cpu_t *cpu, unsigned i)
{
    return cpu->s.f[i];
}

static inline void set_sreg(vfc_cpu_t *cpu, unsigned i, float v)
{
    cpu->s.f[i] = v;
}

static uint32_t vfp_expand_imm(uint32_t imm8)
{
    const uint32_t sign = (imm8 >> 7) & 1;
    const uint32_t b6 = (imm8 >> 6) & 1;
    const uint32_t exponent = ((b6 ^ 1) << 7) | (b6 ? 0x7C : 0) | ((imm8 >> 4) & 3);
    return (sign << 31) | (exponent << 23) | ((imm8 & 0xF) << 19);
}

static void vfp_compare(vfc_cpu_t *cpu, float a, float b)
{
    uint32_t flags;
    if (isnan(a) || isnan(b)) {
        flags = 0x3;                        // C, V
    } else if (a == b) {
        flags = 0x6;                        // Z, C
    } else if (a < b) {
        flags = 0x8;                        // N
    } else {
        flags = 0x2;                        // C
    }
    cpu->fpscr = (cpu->fpscr & 0x0FFFFFFFu) | (flags << 28);
}

static uint32_t float_to_int(float value, bool isSigned, int rounding)
{
    if (isnan(value)) {
        return 0;
    }
    float rounded;
    switch (rounding) {
    case 0: rounded = nearbyintf(value); break;    // to nearest, ties to even
    case 1: rounded = ceilf(value); break;
    case 2: rounded = floorf(value); break;
    default: rounded = truncf(value); break;
    }
    if (isSigned) {
        if (rounded >= 2147483648.0f) {
            return 0x7FFFFFFFu;
        }
        if (rounded < -2147483648.0f) {
            return 0x80000000u;
        }
        return (uint32_t)(int32_t)rounded;
    }
    if (rounded >= 4294967296.0f) {
        return 0xFFFFFFFFu;
    }
    if (rounded < 0) {
        return 0;
    }
    return (uint32_t)rounded;
}

static void vfp_data_processing(ctx_t *x, uint32_t inst)
{
    vfc_cpu_t *cpu = x->cpu;
    if (inst & 0x100) {                     // double precision: not in FPv4-SP
        undefined(x, inst, true);
        return;
    }
    const unsigned d = (((inst >> 12) & 0xF) << 1) | ((inst >> 22) & 1);
    const unsigned n = (((inst >> 16) & 0xF) << 1) | ((inst >> 7) & 1);
    const unsigned m = ((inst & 0xF) << 1) | ((inst >> 5) & 1);
    const bool op = (inst >> 6) & 1;
    const unsigned opc1 = (((inst >> 23) & 1) << 2) | ((inst >> 20) & 3);

    switch (opc1) {
    case 0: {                               // VMLA, VMLS
        const float product = sreg(cpu, n) * sreg(cpu, m);
        set_sreg(cpu, d, sreg(cpu, d) + (op ? -product : product));
        return;
    }
    case 1: {                               // VNMLS, VNMLA
        const float product = sreg(cpu, n) * sreg(cpu, m);
        set_sreg(cpu, d, op ? -sreg(cpu, d) - product : -sreg(cpu, d) + product);
        return;
    }
    case 2: {                               // VMUL, VNMUL
        const float product = sreg(cpu, n) * sreg(cpu, m);
        set_sreg(cpu, d, op ? -product : product);
        return;
    }
    case 3:                                 // VADD, VSUB
        set_sreg(cpu, d, op ? sreg(cpu, n) - sreg(cpu, m) : sreg(cpu, n) + sreg(cpu, m));
        return;
    case 4:                                 // VDIV
        if (op) {
            break;
        }
        set_sreg(cpu, d, sreg(cpu, n) / sreg(cpu, m));
        return;
    case 5:                                 // VFNMS, VFNMA
        set_sreg(cpu, d, op ? fmaf(-sreg(cpu, n), sreg(cpu, m), -sreg(cpu, d))
                            : fmaf(sreg(cpu, n), sreg(cpu, m), -sreg(cpu, d)));
        return;
    case 6:                                 // VFMA, VFMS
        set_sreg(cpu, d, op ? fmaf(-sreg(cpu, n), sreg(cpu, m), sreg(cpu, d))
                            : fmaf(sreg(cpu, n), sreg(cpu, m), sreg(cpu, d)));
        return;
    case 7: {
        const unsigned opc2 = (inst >> 16) & 0xF, opc3 = (inst >> 6) & 3;
        if ((opc3 & 1) == 0) {              // VMOV immediate
            cpu->s.u[d] = vfp_expand_imm((opc2 << 4) | (inst & 0xF));
            return;
        }
        switch (opc2) {
        case 0x0:                           // VMOV register, VABS
            if (opc3 == 1) {
                cpu->s.u[d] = cpu->s.u[m];
            } else {
                cpu->s.u[d] = cpu->s.u[m] & 0x7FFFFFFFu;
            }
            return;
        case 0x1:                           // VNEG, VSQRT
            if (opc3 == 1) {
                cpu->s.u[d] = cpu->s.u[m] ^ 0x80000000u;
            } else {
                set_sreg(cpu, d, sqrtf(sreg(cpu, m)));
            }
            return;
        case 0x4:                           // VCMP{E} register
            vfp_compare(cpu, sreg(cpu, d), sreg(cpu, m));
            return;
        case 0x5:                           // VCMP{E} with zero
            vfp_compare(cpu, sreg(cpu, d), 0.0f);
            return;
        case 0x8:                           // VCVT.F32.{S32,U32}
            if (inst & 0x80) {
                set_sreg(cpu, d, (float)(int32_t)cpu->s.u[m]);
            } else {
                set_sreg(cpu, d, (float)cpu->s.u[m]);
            }
            return;
        case 0xC: case 0xD: {               // VCVT{R}.{U32,S32}.F32
            const int rounding = (inst & 0x80) ? 3 : (int)((cpu->fpscr >> 22) & 3);
            cpu->s.u[d] = float_to_int(sreg(cpu, m), opc2 & 1, rounding);
            return;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
    undefined(x, inst, true);
}

static void vfp(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;
    vfc_t *vfc = x->vfc;
    const uint32_t inst = (hw1 << 16) | hw2;
    const unsigned coproc = (hw2 >> 8) & 0xF;
    if ((coproc & 0xE) != 0xA) {
        undefined(x, inst, true);
        return;
    }
    const bool doubleRegs = coproc & 1;

    if ((hw1 & 0xFF00) == 0xEE00) {
        if ((hw2 & 0x10) == 0) {
            vfp_data_processing(x, inst);
            return;
        }
        // Transfers between a core register and the floating point unit.
        const bool toCore = hw1 & 0x10;
        const unsigned a = (hw1 >> 5) & 7, t = hw2 >> 12;
        if (doubleRegs) {
            undefined(x, inst, true);
            return;
        }
        if (a == 0) {                       // VMOV between Rt and Sn
            const unsigned sn = ((hw1 & 0xF) << 1) | ((hw2 >> 7) & 1);
            if (toCore) {
                cpu->r[t] = cpu->s.u[sn];
            } else {
                cpu->s.u[sn] = cpu->r[t];
            }
            return;
        }
        if (a == 7 && (hw1 & 0xF) == 1) {   // VMRS, VMSR (FPSCR)
            if (toCore) {
                if (t == 15) {
                    cpu->n = cpu->fpscr >> 31;
                    cpu->z = (cpu->fpscr >> 30) & 1;
                    cpu->c = (cpu->fpscr >> 29) & 1;
                    cpu->v = (cpu->fpscr >> 28) & 1;
                } else {
                    cpu->r[t] = cpu->fpscr;
                }
            } else {
                cpu->fpscr = cpu->r[t];
            }
            return;
        }
        undefined(x, inst, true);
        return;
    }

    if ((hw1 & 0xFFE0) == 0xEC40) {         // VMOV two core registers <-> two singles or a double
        const bool toCore = hw1 & 0x10;
        const unsigned t = hw2 >> 12, t2 = hw1 & 0xF;
        const unsigned first = doubleRegs ? ((((hw2 >> 5) & 1) << 4) | (hw2 & 0xF)) * 2
                                          : ((hw2 & 0xF) << 1) | ((hw2 >> 5) & 1);
        if (first + 1 > 31) {
            undefined(x, inst, true);
            return;
        }
        if (toCore) {
            cpu->r[t] = cpu->s.u[first];
            cpu->r[t2] = cpu->s.u[first + 1];
        } else {
            cpu->s.u[first] = cpu->r[t];
            cpu->s.u[first + 1] = cpu->r[t2];
        }
        return;
    }

    // Extension register loads and stores.
    const bool index = hw1 & 0x100, add = hw1 & 0x80, wback = hw1 & 0x20, isLoad = hw1 & 0x10;
    const unsigned n = hw1 & 0xF, vd = (hw2 >> 12) & 0xF, dBit = (hw1 >> 6) & 1;
    const uint32_t imm8 = hw2 & 0xFF;
    const unsigned first = doubleRegs ? ((dBit << 4) | vd) * 2 : (vd << 1) | dBit;

    if (index && !wback) {                  // VLDR, VSTR
        const uint32_t base = n == 15 ? align4(x->pc + 4) : cpu->r[n];
        const uint32_t address = add ? base + imm8 * 4 : base - imm8 * 4;
        const unsigned words = doubleRegs ? 2 : 1;
        for (unsigned i = 0; i < words; i++) {
            if (isLoad) {
                cpu->s.u[first + i] = load(vfc, address + 4 * i, 4);
            } else {
                store(vfc, address + 4 * i, cpu->s.u[first + i], 4);
            }
        }
        return;
    }
    if (index == add) {                     // P == U: undefined here
        undefined(x, inst, true);
        return;
    }
    // VLDM, VSTM (IA or DB), VPUSH, VPOP.
    const unsigned words = imm8;
    if (first + words > 32 || words == 0) {
        undefined(x, inst, true);
        return;
    }
    const uint32_t base = cpu->r[n];
    uint32_t address = add ? base : base - words * 4;
    for (unsigned i = 0; i < words; i++, address += 4) {
        if (isLoad) {
            cpu->s.u[first + i] = load(vfc, address, 4);
        } else {
            store(vfc, address, cpu->s.u[first + i], 4);
        }
    }
    if (wback) {
        cpu->r[n] = add ? base + words * 4 : base - words * 4;
    }
}

// --- 32-bit dispatch

static void exec32(ctx_t *x, uint32_t hw1, uint32_t hw2)
{
    vfc_cpu_t *cpu = x->cpu;

    switch ((hw1 >> 11) & 3) {
    case 1:
        if ((hw1 & 0x0400) != 0) {          // coprocessor
            vfp(x, hw1, hw2);
            return;
        }
        if ((hw1 & 0x0200) != 0) {          // data processing (shifted register)
            const unsigned op = (hw1 >> 5) & 0xF;
            const bool setflags = hw1 & 0x10;
            const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF, m = hw2 & 0xF;
            const unsigned imm5 = (((hw2 >> 12) & 7) << 2) | ((hw2 >> 6) & 3);
            int type;
            unsigned amount;
            decode_imm_shift((hw2 >> 4) & 3, imm5, &type, &amount);
            if (op == 0x6) {                // PKHBT, PKHTB
                const uint32_t shifted = type == SR_LSL ? cpu->r[m] << amount
                                                        : (uint32_t)((int32_t)cpu->r[m] >> (amount > 31 ? 31 : amount));
                cpu->r[d] = type == SR_LSL ? (cpu->r[n] & 0xFFFF) | (shifted & 0xFFFF0000u)
                                           : (shifted & 0xFFFF) | (cpu->r[n] & 0xFFFF0000u);
                return;
            }
            bool carry;
            const uint32_t operand = shift_c(cpu->r[m], type, amount, cpu->c, &carry);
            if (!data_processing(x, op, setflags, d, n, operand, carry)) {
                undefined(x, (hw1 << 16) | hw2, true);
            }
            return;
        }
        if ((hw1 & 0x0040) == 0) {          // load/store multiple
            const unsigned n = hw1 & 0xF;
            const uint32_t list = hw2;
            const bool wback = hw1 & 0x20, isLoad = hw1 & 0x10;
            const uint32_t count = (uint32_t)popcount16(list);
            switch ((hw1 >> 7) & 3) {
            case 1:                         // IA
                if (isLoad) {
                    load_multiple(x, n, list, cpu->r[n], wback, cpu->r[n] + 4 * count);
                } else {
                    store_multiple(x, n, list, cpu->r[n], wback, cpu->r[n] + 4 * count);
                }
                return;
            case 2: {                       // DB
                const uint32_t address = cpu->r[n] - 4 * count;
                if (isLoad) {
                    load_multiple(x, n, list, address, wback, address);
                } else {
                    store_multiple(x, n, list, address, wback, address);
                }
                return;
            }
            default:
                undefined(x, (hw1 << 16) | hw2, true);
                return;
            }
        }
        load_store_dual(x, hw1, hw2);
        return;

    case 2:
        if (hw2 & 0x8000) {
            branch_misc(x, hw1, hw2);
            return;
        }
        if ((hw1 & 0x0200) == 0) {          // data processing (modified immediate)
            const unsigned op = (hw1 >> 5) & 0xF;
            const bool setflags = hw1 & 0x10;
            const unsigned n = hw1 & 0xF, d = (hw2 >> 8) & 0xF;
            const uint32_t imm12 = (((hw1 >> 10) & 1) << 11) | (((hw2 >> 12) & 7) << 8) | (hw2 & 0xFF);
            bool carry;
            const uint32_t imm = thumb_expand_imm_c(imm12, cpu->c, &carry);
            if (!data_processing(x, op, setflags, d, n, imm, carry)) {
                undefined(x, (hw1 << 16) | hw2, true);
            }
            return;
        }
        plain_immediate(x, hw1, hw2);
        return;

    case 3:
        if ((hw1 & 0x0400) != 0) {          // coprocessor T2 space: not the FPU
            undefined(x, (hw1 << 16) | hw2, true);
            return;
        }
        if ((hw1 & 0x0200) == 0) {          // load/store single
            load_store_single(x, hw1, hw2);
            return;
        }
        switch ((hw1 >> 7) & 3) {
        case 0: case 1:
            if ((hw1 & 0x0100) == 0) {
                data_register(x, hw1, hw2);
                return;
            }
            break;
        case 2:
            multiply(x, hw1, hw2);
            return;
        case 3:
            long_multiply(x, hw1, hw2);
            return;
        }
        undefined(x, (hw1 << 16) | hw2, true);
        return;

    default:
        undefined(x, (hw1 << 16) | hw2, true);
        return;
    }
}

// --- The loop

void vfc_cpu_reset(vfc_t *vfc)
{
    vfc_cpu_t *cpu = &vfc->cpu;
    memset(cpu, 0, sizeof(*cpu));
    cpu->r[13] = load(vfc, VFC_FLASH_BASE, 4) & ~3u;
    cpu->r[15] = load(vfc, VFC_FLASH_BASE + 4, 4) & ~1u;
    cpu->r[14] = 0xFFFFFFFFu;
}

void vfc_cpu_run(vfc_t *vfc, uint64_t budget)
{
    vfc_cpu_t *cpu = &vfc->cpu;
    ctx_t x = { .vfc = vfc, .cpu = cpu };

    while (budget-- > 0 && !vfc->stopRequested) {
        const uint32_t pc = cpu->r[15];
        const uint32_t hw1 = fetch16(vfc, pc);
        const bool wide = (hw1 >> 11) >= 0x1D;
        const uint32_t hw2 = wide ? fetch16(vfc, pc + 2) : 0;
        cpu->r[15] = pc + (wide ? 4 : 2);
        x.pc = pc;
        vfc->instructions++;

        if (cpu->itstate & 0xF) {
            const unsigned cond = cpu->itstate >> 4;
            // Advance the IT state before executing, so a nested IT (illegal)
            // or a branch sees the block as finished when it is.
            if ((cpu->itstate & 7) == 0) {
                cpu->itstate = 0;
            } else {
                cpu->itstate = (uint8_t)((cpu->itstate & 0xE0) | ((cpu->itstate << 1) & 0x1F));
            }
            if (!condition_passed(cpu, cond)) {
                continue;
            }
            x.inIT = true;
        } else {
            x.inIT = false;
        }

        if (wide) {
            exec32(&x, hw1, hw2);
        } else {
            exec16(&x, hw1);
        }
    }
    if (vfc->stopRequested && vfc->stopReason == VFC_STOP_FAULT) {
        // Leave the PC on the faulting instruction for diagnostics.
        cpu->r[15] = x.pc;
    }
}
