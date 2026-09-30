// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// CBMC: vfc_load on any image of up to 96 bytes, whatever its contents. No
// access out of bounds (the image included), no undefined behaviour, and an
// image it accepts has its config region inside RAM.

#include <stdlib.h>

#include "vfc.h"
#include "vfc_internal.h"

size_t nondet_size(void);

int main(void)
{
    vfc_t *v = vfc_create();
    __CPROVER_assume(v != NULL);
    const size_t length = nondet_size();
    __CPROVER_assume(length >= 1 && length <= 96);
    uint8_t *image = malloc(length);            // contents unconstrained: any image
    __CPROVER_assume(image != NULL);
    if (vfc_load(v, image, length, VFC_FLASH_BASE) == VFC_OK) {
        __CPROVER_assert(v->flashUsed == length, "the whole image is in flash");
        __CPROVER_assert(v->eepromSize > 0 && v->eepromAddress - VFC_RAM_BASE < VFC_RAM_SIZE
                         && v->eepromSize <= VFC_RAM_SIZE - (v->eepromAddress - VFC_RAM_BASE),
                         "the config region is inside RAM");
        __CPROVER_assert(vfc_pc(v) - VFC_FLASH_BASE < length, "reset lands in the image");
    }
    return 0;
}
