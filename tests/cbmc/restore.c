// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// CBMC: vfc_restore on any snapshot whatever its contents, then every call
// that indexes with what it restored: the serial FIFOs, the console, the
// strings, the mailbox. No access out of bounds, no undefined behaviour.

#include <stdlib.h>
#include <string.h>

#include "vfc.h"
#include "vfc_internal.h"

uint8_t nondet_u8(void);
uint32_t nondet_u32(void);
int nondet_int(void);

int main(void)
{
    vfc_t *v = vfc_create();
    __CPROVER_assume(v != NULL);
    const size_t n = vfc_snapshot_size(v);
    uint8_t *snapshot = malloc(n);              // contents unconstrained: any file
    __CPROVER_assume(snapshot != NULL);
    if (vfc_restore(v, snapshot, n) != VFC_OK) {
        return 0;
    }
    const uint8_t byte = nondet_u8();
    vfc_serial_write(v, &byte, 1);
    uint8_t out[4];
    vfc_serial_read(v, out, sizeof(out));
    char text[VFC_CONSOLE_CAPACITY + 1];
    vfc_console_read(v, text, sizeof(text));
    const char *fault = vfc_fault(v);
    if (fault) {
        __CPROVER_assert(strlen(fault) < sizeof(v->fault), "the fault message is a string");
    }
    __CPROVER_assert(strlen(vfc_firmware_version(v)) < sizeof(v->firmwareVersion), "the version is a string");
    (void)vfc_motor(v, nondet_int());
    (void)vfc_bus_read(v, VFC_MAILBOX_BASE + (nondet_u32() & 0xFFC), 4);
    vfc_bus_write(v, VFC_MAILBOX_BASE + (nondet_u32() & 0xFFC), nondet_u32(), 4);
    vfc_serial_read(v, out, sizeof(out));
    vfc_console_read(v, text, sizeof(text));
    return 0;
}
