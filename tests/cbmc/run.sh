#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
#
# Runs one CBMC proof: tests/cbmc/run.sh <harness> [cbmc options...]. Prints
# one line when it holds, and the properties that don't when it doesn't. The
# full log is left in .build/check/cbmc-<harness>.log.

set -u
harness=$1
shift
root=$(cd "$(dirname "$0")/../.." && pwd)
log=$root/.build/check/cbmc-$harness.log
mkdir -p "$(dirname "$log")"
cd "$root" || exit 2

# The library's sources, unless the harness includes what it needs itself.
case $harness in
logical_imm) sources= ;;
*) sources="Sources/VFC/board.c Sources/VFC/cpu.c Sources/VFC/jit.c" ;;
esac
file=tests/cbmc/$harness.c

start=$(date +%s)
# shellcheck disable=SC2086
cbmc "$file" tests/cbmc/models.c $sources -DVFC_CBMC -ISources/VFC/include -ISources/VFC \
    --bounds-check --pointer-check --signed-overflow-check --undefined-shift-check \
    --div-by-zero-check --pointer-overflow-check --unwinding-assertions "$@" > "$log" 2>&1
status=$?
seconds=$(($(date +%s) - start))
summary=$(grep -E '^\*\* [0-9]+ of [0-9]+ failed' "$log" | tail -1)
if [ $status -eq 0 ] && grep -q 'VERIFICATION SUCCESSFUL' "$log"; then
    total=$(echo "$summary" | sed -E 's/.* of ([0-9]+) failed.*/\1/')
    echo "$harness: holds ($total properties, ${seconds}s)"
    exit 0
fi
echo "$harness: FAILED after ${seconds}s (cbmc exit $status) ${summary:+- $summary}"
grep -E '^\[.*\]: FAILURE$|FAILURE$' "$log" | head -20
[ $status -eq 0 ] && exit 1
exit $status
