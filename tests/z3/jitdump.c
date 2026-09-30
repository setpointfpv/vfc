// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// For tests/z3/prove.py: runs the real translator on single instructions and
// prints the AArch64 it emits, as JSON, one line per instruction. Nothing is
// executed, so it builds and runs anywhere (with -DVFC_JIT_EMULATED, which
// only lets jit.c compile off Apple silicon).
//
// Each instruction is translated at PC, in flash that is otherwise erased,
// as a block of its own: flashUsed stops the block after it, and chaining is
// off, so that every way out of the block goes through the shared exit stub
// with the guest PC in w0 and the exit kind in w1.
//
//   jitdump [narrow|all]      every 16-bit encoding, or those and more
//   jitdump hw1 [hw2]         one instruction

#define VFC_JIT_EMULATED
#include "../../Sources/VFC/jit.c"

bool vfc_jit_emulated_enter(vfc_t *vfc, void *code, size_t codeBytes, void *enter, void *block)
{
    (void)vfc, (void)code, (void)codeBytes, (void)enter, (void)block;
    return false;
}
void *vfc_jit_emulated_helper(int which, void *function)
{
    (void)function;
    return (void *)(uintptr_t)(which ? 0xFEED0004u : 0xFEED0000u);     // recognisable call targets
}
void vfc_jit_emulated_flush(void *code, size_t length)
{
    (void)code, (void)length;
}

#define PC (VFC_FLASH_BASE + 0x1000)

static vfc_t *board;

static void layout(void)
{
    printf("{\"layout\": {\"G\": [");
    for (int i = 0; i < 15; i++) printf("%s%d", i ? ", " : "", G[i]);
    printf("], \"ctx\": %d, \"ramBase\": %d, \"pc\": %u, \"exitKinds\": {\"normal\": %d, \"interpret\": %d, "
           "\"budget\": %d, \"wfi\": %d, \"stop\": %d}, \"loadHelper\": %u, \"storeHelper\": %u, \"offsets\": {",
           CTX, RAMB, PC, EXIT_NORMAL, EXIT_INTERPRET, EXIT_BUDGET, EXIT_WFI, EXIT_STOP, 0xFEED0000u, 0xFEED0004u);
    printf("\"r\": %u, \"s\": %u, \"fpscr\": %u, \"itstate\": %u, \"primask\": %u, \"basepri\": %u, \"faultmask\": %u, "
           "\"jitBudget\": %u, \"jitNzcv\": %u, \"jitSavedNzcv\": %u, \"jitTable\": %u, \"jitFlash\": %u, \"jitRam\": %u, "
           "\"jitLoad\": %u, \"jitStore\": %u, \"stopRequested\": %u, \"stopReason\": %u, \"instructionPc\": %u, "
           "\"timeRegs\": %u, \"size\": %zu}}}\n",
           OFF(cpu.r), OFF(cpu.s), OFF(cpu.fpscr), OFF(cpu.itstate), OFF(cpu.primask), OFF(cpu.basepri), OFF(cpu.faultmask),
           OFF(jitBudget), OFF(jitNzcv), OFF(jitSavedNzcv), OFF(jitTable), OFF(jitFlash), OFF(jitRam),
           OFF(jitLoad), OFF(jitStore), OFF(stopRequested), OFF(stopReason), OFF(instructionPc), OFF(timeRegs), sizeof(vfc_t));
}

static void dump(uint16_t hw1, uint16_t hw2)
{
    struct vfc_jit *jit = board->jit;
    const uint32_t offset = PC - VFC_FLASH_BASE;
    const bool wide = (hw1 >> 11) >= 0x1D;
    memset(board->flash + offset - 64, 0xFF, 256);
    memcpy(board->flash + offset, &hw1, 2);
    memcpy(board->flash + offset + 2, &hw2, 2);
    board->flashUsed = offset + 4;
    memset(jit->table, 0, VFC_FLASH_SIZE / 2 * sizeof(void *));
    jit->npending = 0;
    const uint32_t start = jit->used;
    void *entry = translate(board, PC);
    printf("{\"hw1\": %u, \"hw2\": %u, \"wide\": %s", hw1, hw2, wide ? "true" : "false");
    if (!entry) {
        printf(", \"translated\": false}\n");
    } else {
        printf(", \"translated\": true, \"exit\": %d, \"dispatch\": %d, \"code\": [",
               (int)jit->exit - (int)start, (int)jit->dispatch - (int)start);
        for (uint32_t i = start; i < jit->used; i++) printf("%s%u", i > start ? ", " : "", jit->cache[i]);
        printf("]}\n");
    }
    jit->used = start;                                  // the next one goes in the same place
}

int main(int argc, char **argv)
{
    board = vfc_create();
    if (!board || !jit_init(board)) {
        fprintf(stderr, "jitdump: no JIT\n");
        return 1;
    }
    board->jit->noChain = true;
    layout();
    if (argc > 1 && (!strcmp(argv[1], "narrow") || !strcmp(argv[1], "all"))) {
        for (uint32_t hw = 0; hw < 0xE800; hw++) {
            dump((uint16_t)hw, 0xBF00);
        }
        return 0;
    }
    if (argc > 1) {
        dump((uint16_t)strtoul(argv[1], NULL, 16), argc > 2 ? (uint16_t)strtoul(argv[2], NULL, 16) : 0xBF00);
    }
    return 0;
}
