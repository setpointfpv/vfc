# Development checks for vfc: tests, fuzzing and proofs. The library itself
# is built by SwiftPM (Package.swift); nothing here is part of it. What each
# check establishes, and what it relies on, is in docs/verification.md.
#
#   make check       all of the below
#   make test        regression tests on the interpreter and the JIT, under ASan and UBSan
#   make fpu         the interpreter's floating point against Unicorn's Cortex-M4
#   make fuzz        the JIT against the interpreter over the instruction space (FUZZ_ARGS=...)
#   make encodings   the JIT's AArch64 encoder against an assembler
#   make cbmc        memory safety and no undefined behaviour, proved with CBMC
#   make prove       the JIT's translations proved against a Thumb spec with Z3
#
# On Apple silicon the JIT runs natively. Elsewhere it runs in Unicorn's
# AArch64 emulator (tools/jitemu.c), which needs libunicorn, as does `make fpu`
# everywhere (see UNICORN below).

CC ?= cc
BUILD := .build/check
SOURCES := Sources/VFC/cpu.c Sources/VFC/board.c Sources/VFC/jit.c
CFLAGS := -O1 -g -Wall -Wextra -ffp-contract=off -ISources/VFC/include -ISources/VFC
SANITIZE := -fsanitize=address,undefined -fno-sanitize-recover=undefined
PYTHON ?= python3

# libunicorn. On macOS, Homebrew's if it's installed, since the linker doesn't
# look in Homebrew's prefix; and as unicorn's own CMake build names its dylib
# @rpath/libunicorn.2.dylib, each directory the linker may find it in (-L,
# LIBRARY_PATH, and /usr/local/lib, where that build installs) goes into the
# binaries as an rpath. Elsewhere, the compiler's own paths.
# UNICORN="-I<prefix>/include -L<prefix>/lib -lunicorn" chooses another.
comma := ,
ifeq ($(shell uname -s),Darwin)
UNICORN_HOME := $(firstword $(wildcard /opt/homebrew/opt/unicorn /usr/local/opt/unicorn))
UNICORN ?= $(if $(UNICORN_HOME),-I$(UNICORN_HOME)/include -L$(UNICORN_HOME)/lib) -lunicorn
UNICORN_DIRS = $(patsubst -L%,%,$(filter -L%,$(UNICORN))) $(subst :, ,$(LIBRARY_PATH))
UNICORN_LIBS = $(UNICORN) $(foreach dir,$(UNICORN_DIRS) $(filter-out $(UNICORN_DIRS),/usr/local/lib),-Wl$(comma)-rpath$(comma)$(dir))
else
UNICORN ?= -lunicorn
UNICORN_LIBS = $(UNICORN)
endif

ifeq ($(shell uname -s)-$(shell uname -m),Darwin-arm64)
JIT_CFLAGS :=
JIT_SOURCES :=
JIT_LIBS :=
else
JIT_CFLAGS := -DVFC_JIT_EMULATED
JIT_SOURCES := tools/jitemu.c
JIT_LIBS := $(UNICORN_LIBS)
endif

CBMC_HARNESSES := load restore bus logical_imm
CBMC_UNWIND_load := 100
CBMC_UNWIND_restore := 170
CBMC_UNWIND_bus := 170
CBMC_UNWIND_logical_imm := 40

.PHONY: check test fpu fuzz encodings cbmc $(addprefix cbmc-,$(CBMC_HARNESSES)) prove clean

check: test fpu encodings fuzz cbmc prove

$(BUILD):
	mkdir -p $@

test: $(BUILD)/regress
	ASAN_OPTIONS=detect_leaks=0 $(BUILD)/regress

$(BUILD)/regress: $(SOURCES) Sources/VFC/*.h Sources/VFC/include/vfc.h tests/regress.c $(JIT_SOURCES) | $(BUILD)
	$(CC) $(CFLAGS) $(SANITIZE) $(JIT_CFLAGS) $(SOURCES) $(JIT_SOURCES) tests/regress.c $(JIT_LIBS) -lm -o $@

fpu: $(BUILD)/fpu
	ASAN_OPTIONS=detect_leaks=0 $(BUILD)/fpu

$(BUILD)/fpu: $(SOURCES) Sources/VFC/*.h Sources/VFC/include/vfc.h tests/fpu.c | $(BUILD)
	$(CC) $(CFLAGS) $(SANITIZE) $(SOURCES) tests/fpu.c $(UNICORN_LIBS) -lm -o $@

fuzz: $(BUILD)/jitfuzz
	$(BUILD)/jitfuzz $(FUZZ_ARGS)

$(BUILD)/jitfuzz: $(SOURCES) Sources/VFC/*.h Sources/VFC/include/vfc.h tools/jitfuzz.c $(JIT_SOURCES) | $(BUILD)
	$(CC) $(CFLAGS) -O2 $(JIT_CFLAGS) $(SOURCES) $(JIT_SOURCES) tools/jitfuzz.c $(JIT_LIBS) -lm -o $@

encodings: $(BUILD)/encodings
	$(BUILD)/encodings

$(BUILD)/encodings: Sources/VFC/jit_emit.h tools/encodings.c | $(BUILD)
	$(CC) $(CFLAGS) tools/encodings.c -o $@

cbmc: $(addprefix cbmc-,$(CBMC_HARNESSES))

$(addprefix cbmc-,$(CBMC_HARNESSES)): cbmc-%:
	sh tests/cbmc/run.sh $* --unwind $(CBMC_UNWIND_$*)

prove: $(BUILD)/jitdump
	$(BUILD)/jitdump narrow > $(BUILD)/jitdump.jsonl
	$(PYTHON) tests/z3/check_a64.py $(BUILD)/jitdump.jsonl
	$(PYTHON) tests/z3/prove.py $(BUILD)/jitdump.jsonl

$(BUILD)/jitdump: $(SOURCES) Sources/VFC/*.h tests/z3/jitdump.c | $(BUILD)
	$(CC) $(CFLAGS) tests/z3/jitdump.c Sources/VFC/board.c Sources/VFC/cpu.c -lm -o $@

clean:
	rm -rf $(BUILD)
