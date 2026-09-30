// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
//
// CBMC: library functions it has no body for, modelled by their contracts in
// the C standard, with whatever the standard leaves open left nondeterministic.
// Without these it would treat their results as arbitrary, which can hide a
// bug as easily as it can invent one.

#include <stdarg.h>
#include <stddef.h>

size_t nondet_size_t(void);

void *memchr(const void *s, int c, size_t n)
{
    const unsigned char *p = s;
    for (size_t i = 0; i < n; i++) {
        if (p[i] == (unsigned char)c) {
            return (void *)(p + i);
        }
    }
    return NULL;
}

// Writes some string of fewer than `size` characters, and its terminator.
int vsnprintf(char *out, size_t size, const char *format, va_list args)
{
    (void)format;
    (void)args;
    const size_t length = nondet_size_t();
    if (size == 0) {
        return (int)length;
    }
    __CPROVER_assume(length < size);
    __CPROVER_havoc_slice(out, length);
    out[length] = 0;
    return (int)length;
}
