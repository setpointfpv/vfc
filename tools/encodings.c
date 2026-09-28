// SPDX-License-Identifier: MIT
// Development check: the JIT's AArch64 encoder against the system assembler.
//   cc -ISources/VFC tools/encodings.c -o /tmp/encodings && /tmp/encodings
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jit_emit.h"

static uint32_t buf[4096];
static a64_buf_t b = { buf, 0, 4096, false };
static char text[65536];
static size_t textLength;

#define T(asmtext, call) do { call; textLength += (size_t)snprintf(text + textLength, sizeof(text) - textLength, "%s\n", asmtext); } while (0)

int main(void)
{
    T("add w1, w2, w3", a64_add(&b, 1, 2, 3, SH_LSL, 0));
    T("add w1, w2, w3, lsl #2", a64_add(&b, 1, 2, 3, SH_LSL, 2));
    T("adds w19, w20, w21", a64_adds(&b, 19, 20, 21, SH_LSL, 0));
    T("sub w9, w10, w11, lsr #5", a64_sub(&b, 9, 10, 11, SH_LSR, 5));
    T("subs wzr, w19, w9", a64_subs(&b, 31, 19, 9, SH_LSL, 0));
    T("and w1, w2, w3", a64_and(&b, 1, 2, 3, SH_LSL, 0));
    T("bic w1, w2, w3, asr #3", a64_bic(&b, 1, 2, 3, SH_ASR, 3));
    T("orr w1, w2, w3, ror #7", a64_orr(&b, 1, 2, 3, SH_ROR, 7));
    T("orn w1, wzr, w3", a64_orn(&b, 1, 31, 3, SH_LSL, 0));
    T("eor w1, w2, w3", a64_eor(&b, 1, 2, 3, SH_LSL, 0));
    T("ands wzr, w5, w5", a64_ands(&b, 31, 5, 5, SH_LSL, 0));
    T("adc w1, w2, w3", a64_adc(&b, 1, 2, 3));
    T("adcs w1, w2, w3", a64_adcs(&b, 1, 2, 3));
    T("sbc w1, w2, w3", a64_sbc(&b, 1, 2, 3));
    T("sbcs w1, w2, w3", a64_sbcs(&b, 1, 2, 3));
    T("mov w4, w5", a64_mov(&b, 4, 5));
    T("lsl w1, w2, w3", a64_lslv(&b, 1, 2, 3));
    T("lsl x1, x2, x3", a64_lslv_x(&b, 1, 2, 3));
    T("lsr x1, x2, x3", a64_lsrv_x(&b, 1, 2, 3));
    T("asr x1, x2, x3", a64_asrv_x(&b, 1, 2, 3));
    T("lsl x2, x19, #32", a64_ubfm_x(&b, 2, 19, 32, 31));
    T("lsr x2, x2, #32", a64_ubfm_x(&b, 2, 2, 32, 63));
    T("ubfx x6, x2, #32, #1", a64_ubfm_x(&b, 6, 2, 32, 32));
    T("lsr w1, w2, w3", a64_lsrv(&b, 1, 2, 3));
    T("asr w1, w2, w3", a64_asrv(&b, 1, 2, 3));
    T("ror w1, w2, w3", a64_rorv(&b, 1, 2, 3));
    T("udiv w1, w2, w3", a64_udiv(&b, 1, 2, 3));
    T("sdiv w1, w2, w3", a64_sdiv(&b, 1, 2, 3));
    T("madd w1, w2, w3, w4", a64_madd(&b, 1, 2, 3, 4));
    T("mul w1, w2, w3", a64_madd(&b, 1, 2, 3, 31));
    T("msub w1, w2, w3, w4", a64_msub(&b, 1, 2, 3, 4));
    T("smaddl x1, w2, w3, x4", a64_smaddl(&b, 1, 2, 3, 4));
    T("umaddl x1, w2, w3, xzr", a64_umaddl(&b, 1, 2, 3, 31));
    T("clz w1, w2", a64_clz(&b, 1, 2));
    T("rbit w1, w2", a64_rbit(&b, 1, 2));
    T("rev w1, w2", a64_rev(&b, 1, 2));
    T("rev16 w1, w2", a64_rev16(&b, 1, 2));
    T("csel w1, w2, w3, ne", a64_csel(&b, 1, 2, 3, COND_NE));
    T("lsl w1, w2, #5", a64_lsl_imm(&b, 1, 2, 5));
    T("lsr w1, w2, #19", a64_lsr_imm(&b, 1, 2, 19));
    T("asr w1, w2, #31", a64_asr_imm(&b, 1, 2, 31));
    T("ror w1, w2, #16", a64_ror_imm(&b, 1, 2, 16));
    T("ubfx w1, w2, #29, #1", a64_ubfx(&b, 1, 2, 29, 1));
    T("sbfx w1, w2, #3, #9", a64_sbfx(&b, 1, 2, 3, 9));
    T("bfi w17, w6, #29, #1", a64_bfi(&b, 17, 6, 29, 1));
    T("bfi w1, wzr, #4, #8", a64_bfi(&b, 1, 31, 4, 8));
    T("uxtb w1, w2", a64_uxtb(&b, 1, 2));
    T("uxth w1, w2", a64_uxth(&b, 1, 2));
    T("sxtb w1, w2", a64_sxtb(&b, 1, 2));
    T("sxth w1, w2", a64_sxth(&b, 1, 2));
    T("add w1, w2, #4095", a64_add_imm(&b, 1, 2, 4095, false));
    T("adds w1, w2, #7", a64_adds_imm(&b, 1, 2, 7));
    T("sub w1, w2, #255", a64_sub_imm(&b, 1, 2, 255, false));
    T("subs wzr, w2, #200", a64_subs_imm(&b, 31, 2, 200));
    T("add x16, x16, #12", a64_add_x_imm(&b, 16, 16, 12));
    T("sub x16, x16, #48", a64_sub_x_imm(&b, 16, 16, 48));
    T("movz w1, #0x2000, lsl #16", a64_movz(&b, 1, 0x2000, 1));
    T("movk w1, #0x1234, lsl #16", a64_movk(&b, 1, 0x1234, 1));
    T("movn w1, #5", a64_movn(&b, 1, 5, 0));
    T("and w16, w16, #0x30000000", a64_and_imm(&b, 16, 16, 0x30000000u));
    T("and w16, w16, #0x10000000", a64_and_imm(&b, 16, 16, 0x10000000u));
    T("and w0, w0, #0xfffffffe", a64_and_imm(&b, 0, 0, 0xFFFFFFFEu));
    T("and w1, w2, #0xff", a64_and_imm(&b, 1, 2, 0xFF));
    T("and w1, w2, #0x1", a64_and_imm(&b, 1, 2, 1));
    T("and w1, w1, #0x7fffffff", a64_and_imm(&b, 1, 1, 0x7FFFFFFFu));
    T("and w4, w4, #0x0fffffff", a64_and_imm(&b, 4, 4, 0x0FFFFFFFu));
    T("and w16, w16, #0xf0000000", a64_and_imm(&b, 16, 16, 0xF0000000u));
    T("and w1, w1, #0x55555555", a64_and_imm(&b, 1, 1, 0x55555555u));
    T("and w1, w1, #0x00ff00ff", a64_and_imm(&b, 1, 1, 0x00FF00FFu));
    T("mrs x16, nzcv", a64_mrs_nzcv(&b, 16));
    T("msr nzcv, x17", a64_msr_nzcv(&b, 17));
    T("br x16", a64_br(&b, 16));
    T("blr x16", a64_blr(&b, 16));
    T("ret", a64_ret(&b));
    T("ldr w1, [x27, w16, uxtw]", a64_ldr_uxtw(&b, 1, 27, 16));
    T("str w1, [x27, w16, uxtw]", a64_str_uxtw(&b, 1, 27, 16));
    T("ldrb w1, [x27, w16, uxtw]", a64_ldrb_uxtw(&b, 1, 27, 16));
    T("strb w1, [x27, w16, uxtw]", a64_strb_uxtw(&b, 1, 27, 16));
    T("ldrh w1, [x27, w16, uxtw]", a64_ldrh_uxtw(&b, 1, 27, 16));
    T("strh w1, [x27, w16, uxtw]", a64_strh_uxtw(&b, 1, 27, 16));
    T("ldrsb w1, [x27, w16, uxtw]", a64_ldrsb_uxtw(&b, 1, 27, 16));
    T("ldrsh w1, [x27, w16, uxtw]", a64_ldrsh_uxtw(&b, 1, 27, 16));
    T("ldr s1, [x27, w16, uxtw]", a64_ldr_s_uxtw(&b, 1, 27, 16));
    T("str s1, [x27, w16, uxtw]", a64_str_s_uxtw(&b, 1, 27, 16));
    T("ldr w1, [x28, #60]", a64_ldr_imm(&b, 1, 28, 60));
    T("str w1, [x28, #252]", a64_str_imm(&b, 1, 28, 252));
    T("ldr x16, [x28, #256]", a64_ldr_x_imm(&b, 16, 28, 256));
    T("str x16, [x28, #256]", a64_str_x_imm(&b, 16, 28, 256));
    T("ldrb w16, [x28, #333]", a64_ldrb_imm(&b, 16, 28, 333));
    T("strb w16, [x28, #97]", a64_strb_imm(&b, 16, 28, 97));
    T("ldr s1, [x28, #100]", a64_ldr_s_imm(&b, 1, 28, 100));
    T("str s0, [x28, #188]", a64_str_s_imm(&b, 0, 28, 188));
    T("ldr x16, [x17, x16, lsl #3]", a64_ldr_x_lsl3(&b, 16, 17, 16));
    T("stp w9, w10, [x28, #32]", a64_stp_w(&b, 9, 10, 28, 32));
    T("ldp w13, w14, [x28, #48]", a64_ldp_w(&b, 13, 14, 28, 48));
    T("stp x29, x30, [sp, #-96]!", a64_stp_x_pre(&b, 29, 30, 31, -96));
    T("ldp x29, x30, [sp], #96", a64_ldp_x_post(&b, 29, 30, 31, 96));
    T("fadd s0, s1, s2", a64_fadd(&b, 0, 1, 2));
    T("fsub s0, s1, s2", a64_fsub(&b, 0, 1, 2));
    T("fmul s0, s1, s2", a64_fmul(&b, 0, 1, 2));
    T("fdiv s0, s1, s2", a64_fdiv(&b, 0, 1, 2));
    T("fnmul s0, s1, s2", a64_fnmul(&b, 0, 1, 2));
    T("fmadd s0, s1, s2, s3", a64_fmadd(&b, 0, 1, 2, 3));
    T("fmsub s0, s1, s2, s3", a64_fmsub(&b, 0, 1, 2, 3));
    T("fnmadd s0, s1, s2, s3", a64_fnmadd(&b, 0, 1, 2, 3));
    T("fnmsub s0, s1, s2, s3", a64_fnmsub(&b, 0, 1, 2, 3));
    T("fabs s0, s1", a64_fabs(&b, 0, 1));
    T("fneg s0, s1", a64_fneg(&b, 0, 1));
    T("fsqrt s0, s1", a64_fsqrt(&b, 0, 1));
    T("fcmp s0, s1", a64_fcmp(&b, 0, 1));
    T("fcmp s0, #0.0", a64_fcmp0(&b, 0));
    T("fcvtzs w1, s1", a64_fcvtzs(&b, 1, 1));
    T("fcvtzu w1, s1", a64_fcvtzu(&b, 1, 1));
    T("scvtf s0, w1", a64_scvtf(&b, 0, 1));
    T("ucvtf s0, w1", a64_ucvtf(&b, 0, 1));
    T("fmov w1, s2", a64_fmov_ws(&b, 1, 2));
    T("fmov s2, w1", a64_fmov_sw(&b, 2, 1));

    FILE *f = fopen("/tmp/vfc-enc.s", "w");
    fputs(".text\n", f);
    fputs(text, f);
    fclose(f);
    if (system("clang -c /tmp/vfc-enc.s -o /tmp/vfc-enc.o && otool -tvX /tmp/vfc-enc.o > /dev/null && "
               "objcopy -O binary --only-section=__TEXT,__text /tmp/vfc-enc.o /tmp/vfc-enc.bin 2>/dev/null || "
               "segedit /tmp/vfc-enc.o -extract __TEXT __text /tmp/vfc-enc.bin") != 0) {
        fprintf(stderr, "assembling failed\n");
    }
    FILE *bin = fopen("/tmp/vfc-enc.bin", "rb");
    if (!bin) { fprintf(stderr, "no assembled output\n"); return 1; }
    uint32_t ref[4096];
    const size_t n = fread(ref, 4, 4096, bin);
    fclose(bin);
    int bad = 0;
    char *line = strtok(text, "\n");
    for (uint32_t i = 0; i < b.count && line; i++, line = strtok(NULL, "\n")) {
        if (i >= n || ref[i] != buf[i]) {
            printf("MISMATCH %-34s ours %08x  assembler %08x\n", line, buf[i], i < n ? ref[i] : 0);
            bad++;
        }
    }
    printf("%u encodings checked, %d wrong\n", b.count, bad);
    return bad != 0;
}
