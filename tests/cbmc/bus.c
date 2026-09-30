// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// CBMC: the bus, inductively. From any board state within its invariants
// (which vfc_create establishes and vfc_restore checks), any read or write,
// at any address, of any size, with any value, makes no access out of bounds,
// has no undefined behaviour, and leaves the invariants holding: so no
// sequence of accesses does either. A read of RAM returns what's there.

#include <stdlib.h>
#include <string.h>

#include "vfc.h"
#include "vfc_internal.h"

uint32_t nondet_u32(void);
size_t nondet_size_t(void);
int nondet_int(void);
_Bool nondet_bool(void);

static bool invariants(const vfc_t *v)
{
    return v->toGuest.head < VFC_SERIAL_CAPACITY && v->toGuest.tail < VFC_SERIAL_CAPACITY
        && v->fromGuest.head < VFC_SERIAL_CAPACITY && v->fromGuest.tail < VFC_SERIAL_CAPACITY
        && v->consoleLength <= VFC_CONSOLE_CAPACITY
        && v->blackboxLength <= v->blackboxCapacity && (v->blackbox != NULL || v->blackboxCapacity == 0);
}

int main(void)
{
    vfc_t *v = vfc_create();
    __CPROVER_assume(v != NULL);

    // Any state within the invariants.
    v->toGuest.head = nondet_u32();
    v->toGuest.tail = nondet_u32();
    v->fromGuest.head = nondet_u32();
    v->fromGuest.tail = nondet_u32();
    v->consoleLength = nondet_u32();
    v->blackboxCapacity = nondet_size_t();
    v->blackboxLength = nondet_size_t();
    __CPROVER_assume(v->blackboxCapacity <= 2 * VFC_BLACKBOX_CHUNK);
    v->blackbox = v->blackboxCapacity ? malloc(v->blackboxCapacity) : NULL;
    __CPROVER_assume(invariants(v));
    __CPROVER_havoc_slice(v->ram, VFC_RAM_SIZE);

    const uint32_t address = nondet_u32();
    const int size = nondet_int();
    __CPROVER_assume(size == 1 || size == 2 || size == 4);
    if (nondet_bool()) {
        vfc_bus_write(v, address, nondet_u32(), size);
    } else {
        const uint32_t value = vfc_bus_read(v, address, size);
        if (address - VFC_RAM_BASE <= VFC_RAM_SIZE - (uint32_t)size) {
            uint32_t there = 0;
            memcpy(&there, v->ram + (address - VFC_RAM_BASE), (size_t)size);
            __CPROVER_assert(value == there, "a RAM read returns RAM");
        }
    }
    __CPROVER_assert(invariants(v), "the invariants still hold");
    return 0;
}
