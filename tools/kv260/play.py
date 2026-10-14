#!/usr/bin/env python3
"""Replay a register transcript (tb/axil.py format, written by tb/test_dec_host.py with TS_TRANSCRIPT) against the
decoder's AXI-Lite register file on the KV260 through /dev/mem, so a board run needs no model: the transcript holds
every write, the polls, and the expected value of every checked read (the picture, sample by sample).
    sudo ./play.py host-seed1.txt [--base 0xA0000000] [--quiet]
Lines: W <addr> <data> | R <addr> <expect> | P <addr> <mask> <value> (hex). The pushes into the tile-byte FIFO
(DATA1 / DATA4) wait for a free word themselves (the transcript does not record the simulation's flow control).
Exit status 0 when every R matched and every P completed; the first mismatches are printed.
"""
import argparse
import ctypes
import mmap
import os
import struct
import sys
import time

R_STATUS, R_DATA1, R_DATA4, R_HADDR = 0x010, 0x01C, 0x020, 0x024


class Regs:
    """32-bit register access through /dev/mem. Every access must be ONE 32-bit bus transaction: on the first
    board run (2026-10-14) struct.pack_into turned out to issue three stores per word and a slice assignment
    two, so each DATA4 write pushed several junk words into the tile FIFO; ctypes' c_uint32 does a single
    aligned store/load."""
    def __init__(self, base, span=0x1000):
        self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, span, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=base)
        self.regs = {}

    def reg(self, a):
        r = self.regs.get(a)
        if r is None:
            r = self.regs[a] = ctypes.c_uint32.from_buffer(self.m, a)
        return r

    def rd(self, a):
        return self.reg(a).value

    def wr(self, a, v):
        self.reg(a).value = v & 0xFFFFFFFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("transcript")
    ap.add_argument("--base", type=lambda x: int(x, 0), default=0xA0000000)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--max-bad", type=int, default=16)
    ap.add_argument("--poll-timeout", type=float, default=30.0, help="seconds per poll")
    ap.add_argument("--dump", help="write every checked read as '<addr> <value> <expected>' to this file")
    args = ap.parse_args()
    r = Regs(args.base)
    ops = [ln.split() for ln in open(args.transcript) if ln.strip()]
    bad = []
    dump = open(args.dump, "w") if args.dump else None
    n_r = n_w = n_p = 0
    t0 = time.time()
    for i, op in enumerate(ops):
        kind = op[0]
        a = int(op[1], 16)
        if kind == "W":
            v = int(op[2], 16)
            if a in (R_DATA1, R_DATA4):
                t1 = time.time()
                while (r.rd(R_STATUS) >> 16) == 0:
                    if time.time() - t1 > args.poll_timeout:
                        print(f"op {i}: FIFO never freed a word (STATUS 0x{r.rd(R_STATUS):08x})")
                        sys.exit(2)
            r.wr(a, v)
            n_w += 1
            if a == R_HADDR:
                # the AXI write and the following HDATA read travel on independent channels and the read can
                # overtake the posted write (first board run: every pixel came back as its predecessor), so wait
                # until the address reads back before the data read is issued
                for _ in range(1000):
                    if r.rd(R_HADDR) == v:
                        break
                else:
                    print(f"op {i}: HADDR never read back 0x{v:08x}")
                    sys.exit(2)
        elif kind == "R":
            exp = int(op[2], 16)
            v = r.rd(a)
            n_r += 1
            if dump:
                dump.write(f"{a:x} {v:x} {exp:x}\n")
            if v != exp:
                bad.append((i, a, v, exp))
                if len(bad) <= args.max_bad:
                    print(f"op {i}: read 0x{a:03x} = 0x{v:08x}, expected 0x{exp:08x}")
        elif kind == "P":
            mask, val = int(op[2], 16), int(op[3], 16)
            t1 = time.time()
            while (r.rd(a) & mask) != val:
                if time.time() - t1 > args.poll_timeout:
                    print(f"op {i}: poll 0x{a:03x} & 0x{mask:x} != 0x{val:x} after {args.poll_timeout}s (reads 0x{r.rd(a):08x})")
                    sys.exit(2)
            n_p += 1
        else:
            print(f"op {i}: unknown {op}")
            sys.exit(2)
        if not args.quiet and i % 20000 == 0 and i:
            print(f"  {i}/{len(ops)} ops, {len(bad)} bad, {time.time() - t0:.1f}s")
    print(f"{len(ops)} ops ({n_w} writes, {n_r} checked reads, {n_p} polls) in {time.time() - t0:.1f}s: {len(bad)} mismatches")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
