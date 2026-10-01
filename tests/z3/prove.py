# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Ross Bamford. See vfc.h for the licence.
#
# Proves, with Z3, that the code the JIT emits for each 16-bit Thumb
# instruction does what thumb.py says the instruction does, from every
# machine state: every register value, every flag, every byte of memory.
#
# The code is the real translator's (tests/z3/jitdump), run on the AArch64
# model in a64.py (which tests/z3/check_a64.py holds to Unicorn). Every path
# through the block, up to the exit stub, is proved separately. A path either
#
# - hands the instruction to the interpreter (exits at its own PC): then it
#   must have changed nothing at all, the budget included, so that the
#   interpreter starts from the state the block did; or
# - completes it: then the exit PC, the fifteen registers, the flags and all
#   of memory must be exactly what the instruction makes them, with the
#   instruction taken off the budget.
#
# Memory is compared whole, so a stray store anywhere fails a proof. The top
# halves of the host registers are left unknown, so code that relies on them
# being zero fails too.
#
#   python3 tests/z3/prove.py <jitdump output> [--jobs N] [--only HW]

import json
import multiprocessing
import sys
import time

import z3

import a64
import thumb

CTX = 0x0000100000000000                # where the board is, to the generated code
RAM = 0x0000200000000000
FLASH = 0x0000300000000000


def bv(value, bits):
    return z3.BitVecVal(value & ((1 << bits) - 1), bits)


def load(mem, address, size):
    parts = [z3.Select(mem, bv(address + i, 64)) for i in range(size)]
    return z3.Concat(*reversed(parts)) if size > 1 else parts[0]


def store(mem, address, value, size):
    for i in range(size):
        mem = z3.Store(mem, bv(address + i, 64), z3.Extract(8 * i + 7, 8 * i, value))
    return mem


class Failed(Exception):
    pass


def explore(machine, code, exit_index, pre):
    """Every path from the block's start to the exit stub: (condition, machine)."""
    solver = z3.Solver()
    solver.add(pre)
    work, finished = [(z3.BoolVal(True), machine, 0)], []
    while work:
        cond, m, i = work.pop()
        for _ in range(4 * len(code) + 16):
            if i == exit_index:
                finished.append((cond, m))
                break
            if not 0 <= i < len(code):
                raise Failed('control left the block at %d' % i)
            ways = a64.execute(m, code[i], i)
            live = []
            for c, target in ways:
                c = z3.simplify(z3.And(cond, c))
                if z3.is_false(c):
                    continue
                if len(ways) > 1 and not z3.is_true(c):
                    solver.push()
                    solver.add(c)
                    feasible = solver.check() != z3.unsat
                    solver.pop()
                    if not feasible:
                        continue
                live.append((c, target))
            if not live:
                break
            for c, target in live[1:]:
                if isinstance(target, tuple):
                    raise Failed('%s out of the block' % target[0])
                work.append((c, m.copy(), target))
            cond, target = live[0]
            if isinstance(target, tuple):
                raise thumb.NotCovered('%s out of the block' % target[0])
            i = target
        else:
            raise Failed('no exit within bound')
    return finished


def prove(layout, entry):
    """'proved', 'not covered: ...', or raises Failed with a counterexample."""
    hw, pc, code, exit_index = entry['hw1'], layout['pc'], entry['code'], entry['exit']
    off, G, kinds = layout['offsets'], layout['G'], layout['exitKinds']
    m = a64.Machine('j')
    mem0 = m.mem
    m.x[layout['ctx']] = bv(CTX, 64)
    m.x[layout['ramBase']] = bv(RAM, 64)
    budget0 = load(mem0, CTX + off['jitBudget'], 8)
    itstate0 = load(mem0, CTX + off['itstate'], 1)
    pre = z3.And(budget0 >= 1, itstate0 == 0, load(mem0, CTX + off['jitFlash'], 8) == FLASH)

    initial = thumb.State([z3.Extract(31, 0, m.x[G[i]]) for i in range(15)], m.n, m.z, m.c, m.v, itstate0,
                          load(mem0, CTX + off['primask'], 4), load(mem0, CTX + off['faultmask'], 4))
    try:
        outcomes = thumb.outcomes(initial, hw, pc)
    except thumb.NotCovered as e:
        return 'not covered: %s' % e
    try:
        paths = explore(m, code, exit_index, pre)
    except thumb.NotCovered as e:
        return 'not covered: %s' % e
    except a64.Unsupported as e:
        raise Failed('the model has no %s' % e)

    def unchanged(fin):
        same = [z3.Extract(31, 0, fin.x[G[i]]) == initial.r[i] for i in range(15)]
        same += [fin.n == initial.n, fin.z == initial.z, fin.c == initial.c, fin.v == initial.v, fin.mem == mem0]
        return z3.And(same)

    def matches(fin, out, exit_kind):
        s = out.state
        expected = store(mem0, CTX + off['jitBudget'], budget0 - 1, 8)
        expected = store(expected, CTX + off['itstate'], s.itstate, 1)
        expected = store(expected, CTX + off['primask'], s.primask, 4)
        expected = store(expected, CTX + off['faultmask'], s.faultmask, 4)
        want = {'next': (kinds['normal'], kinds['interpret']), 'wfi': (kinds['wfi'],), 'fault': ()}[out.kind]
        if exit_kind not in want:
            return z3.BoolVal(False)
        same = [z3.Extract(31, 0, fin.x[0]) == out.pc]
        same += [z3.Extract(31, 0, fin.x[G[i]]) == s.r[i] for i in range(15)]
        same += [fin.n == s.n, fin.z == s.z, fin.c == s.c, fin.v == s.v, fin.mem == expected]
        return z3.And(same)

    for cond, fin in paths:
        exit_kind = z3.simplify(z3.Extract(31, 0, fin.x[1]))
        if not z3.is_bv_value(exit_kind):
            raise Failed('exit kind not known')
        exit_kind = exit_kind.as_long()
        exit_pc = z3.simplify(z3.Extract(31, 0, fin.x[0]))
        punted = exit_kind in (kinds['interpret'], kinds['budget']) and z3.is_bv_value(exit_pc) and exit_pc.as_long() == pc
        obligations = [(z3.BoolVal(True), unchanged(fin), 'hands over with nothing changed')] if punted else \
                      [(c, matches(fin, out, exit_kind), 'does what the instruction does (%s)' % out.kind) for c, out in outcomes]
        for c, obligation, what in obligations:
            solver = z3.Solver()
            solver.set('timeout', 120000)
            solver.add(pre, cond, c, z3.Not(obligation))
            result = solver.check()
            if result == z3.sat:
                model = solver.model()
                regs = ' '.join('r%d=%08x' % (i, model.eval(initial.r[i], True).as_long()) for i in range(15))
                flags = ''.join(f for f, b in zip('NZCV', (initial.n, initial.z, initial.c, initial.v))
                                if z3.is_true(model.eval(b, True)))
                raise Failed('a path that %s does not, from %s flags %s' % (what.split(' (')[0], regs, flags or '-'))
            if result != z3.unsat:
                raise Failed('undecided (%s)' % solver.reason_unknown())
    return 'proved'


def work(args):
    layout, line = args
    entry = json.loads(line)
    try:
        return entry['hw1'], prove(layout, entry)
    except Failed as e:
        return entry['hw1'], 'FAILED: %s' % e


def main():
    args = sys.argv[1:]
    jobs, only = multiprocessing.cpu_count(), None
    if '--jobs' in args:
        jobs = int(args[args.index('--jobs') + 1])
    if '--only' in args:
        only = int(args[args.index('--only') + 1], 16)
    with open(args[0]) as f:
        layout = json.loads(f.readline())['layout']
        lines = [line for line in f if json.loads(line)['translated']
                 and (only is None or json.loads(line)['hw1'] == only)
                 and not json.loads(line)['wide']]
    start = time.time()
    counts = {'proved': 0, 'not covered': 0, 'FAILED': 0}
    with multiprocessing.Pool(jobs) as pool:
        for hw, result in pool.imap_unordered(work, [(layout, line) for line in lines], chunksize=16):
            key = result.split(':')[0]
            counts[key] += 1
            if key == 'FAILED' or only is not None:
                print('%04x: %s' % (hw, result))
    print('%d 16-bit encodings the JIT translates: %d proved, %d not covered (loads and stores, left to make fuzz), %d failed, in %.0fs'
          % (len(lines), counts['proved'], counts['not covered'], counts['FAILED'], time.time() - start))
    return 1 if counts['FAILED'] else 0


if __name__ == '__main__':
    sys.exit(main())
