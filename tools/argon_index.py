#!/usr/bin/env python3
"""Index AV1 .obu streams (Section 5 or Annex B): sequence-header facts + frame-type census.

Usage: argon_index.py <dir-or-files...> [-o index.csv]
Columns: path, container, profile, bitdepth, mono, ss_x, ss_y, max_w, max_h, sb128,
         still, reduced_hdr, filter_intra, intra_edge, superres, cdef, lr, film_grain,
         n_key, n_intra_only, n_inter, n_switch, n_show_existing, n_frames, intra_only_stream
"""
import csv, os, sys


class Bits:
    def __init__(self, data, pos=0):
        self.d = data; self.p = pos * 8

    def f(self, n):
        v = 0
        for _ in range(n):
            byte = self.d[self.p >> 3]
            v = (v << 1) | ((byte >> (7 - (self.p & 7))) & 1)
            self.p += 1
        return v

    def uvlc(self):
        lz = 0
        while self.f(1) == 0:
            lz += 1
            if lz >= 32: return (1 << 32) - 1
        return self.f(lz) + (1 << lz) - 1


def leb128(d, p):
    v = 0
    for i in range(8):
        b = d[p]; p += 1
        v |= (b & 0x7f) << (7 * i)
        if not b & 0x80: break
    return v, p


def parse_seq(b):
    s = {}
    s['profile'] = b.f(3); s['still'] = b.f(1); red = b.f(1); s['reduced_hdr'] = red
    dm_present = 0; buf_delay_len = 0
    if red:
        b.f(5)  # seq_level_idx
    else:
        if b.f(1):  # timing_info_present
            b.f(32); b.f(32)
            if b.f(1): b.uvlc()
            dm_present = b.f(1)
            if dm_present:
                buf_delay_len = b.f(5) + 1; b.f(32); b.f(5); b.f(5)
        idd_present = b.f(1)
        cnt = b.f(5) + 1
        for _ in range(cnt):
            b.f(12); lvl = b.f(5)
            if lvl > 7: b.f(1)
            if dm_present and b.f(1):
                b.f(buf_delay_len); b.f(buf_delay_len); b.f(1)
            if idd_present and b.f(1):
                b.f(4)
    wb = b.f(4) + 1; hb = b.f(4) + 1
    s['max_w'] = b.f(wb) + 1; s['max_h'] = b.f(hb) + 1
    fid = 0 if red else b.f(1)
    if fid: b.f(4); b.f(3)
    s['sb128'] = b.f(1); s['filter_intra'] = b.f(1); s['intra_edge'] = b.f(1)
    if not red:
        b.f(4)  # interintra, masked, warped, dual_filter
        order_hint = b.f(1)
        if order_hint: b.f(2)
        if b.f(1) == 0: sfsct = b.f(1)
        else: sfsct = 2
        if sfsct > 0:
            if b.f(1) == 0: b.f(1)
        if order_hint: b.f(3)
    s['superres'] = b.f(1); s['cdef'] = b.f(1); s['lr'] = b.f(1)
    # color_config
    hbd = b.f(1)
    if s['profile'] == 2 and hbd: s['bitdepth'] = 12 if b.f(1) else 10
    else: s['bitdepth'] = 10 if hbd else 8
    mono = 0 if s['profile'] == 1 else b.f(1)
    s['mono'] = mono
    cp = tc = mc = None
    if b.f(1): cp = b.f(8); tc = b.f(8); mc = b.f(8)
    if mono:
        b.f(1); s['ss_x'] = s['ss_y'] = 1
    elif cp == 1 and tc == 13 and mc == 0:
        s['ss_x'] = s['ss_y'] = 0
    else:
        b.f(1)
        if s['profile'] == 0: s['ss_x'] = s['ss_y'] = 1
        elif s['profile'] == 1: s['ss_x'] = s['ss_y'] = 0
        else:
            if s['bitdepth'] == 12:
                s['ss_x'] = b.f(1); s['ss_y'] = b.f(1) if s['ss_x'] else 0
            else: s['ss_x'], s['ss_y'] = 1, 0
        if s['ss_x'] and s['ss_y']: b.f(2)
    if not mono: b.f(1)  # separate_uv_delta_q
    s['film_grain'] = b.f(1)
    return s


def obus_section5(d):
    p = 0
    while p < len(d):
        h = d[p]
        if h & 0x80: raise ValueError('forbidden bit')
        typ = (h >> 3) & 0xf; ext = (h >> 2) & 1; has_size = (h >> 1) & 1
        q = p + 1 + ext
        if not has_size: raise ValueError('no size field')
        size, q = leb128(d, q)
        yield typ, q, size
        p = q + size


def obus_annexb(d):
    p = 0
    while p < len(d):
        tu, p = leb128(d, p); tu_end = p + tu
        while p < tu_end:
            fu, p = leb128(d, p); fu_end = p + fu
            while p < fu_end:
                ol, p = leb128(d, p); o_end = p + ol
                h = d[p]
                if h & 0x80: raise ValueError('forbidden bit')
                typ = (h >> 3) & 0xf; ext = (h >> 2) & 1; has_size = (h >> 1) & 1
                q = p + 1 + ext
                if has_size:
                    size, q = leb128(d, q)
                else:
                    size = o_end - q
                yield typ, q, size
                p = o_end


def index_stream(path):
    d = open(path, 'rb').read()
    row = {'path': path}
    for name, gen in (('section5', obus_section5), ('annexb', obus_annexb)):
        try:
            seq = None; cnt = dict(n_key=0, n_intra_only=0, n_inter=0, n_switch=0, n_show_existing=0)
            for typ, off, size in gen(d):
                if typ == 1:
                    seq = parse_seq(Bits(d, off))
                elif typ in (3, 6) and seq is not None:
                    b = Bits(d, off)
                    if seq['reduced_hdr']:
                        cnt['n_key'] += 1; continue
                    if b.f(1): cnt['n_show_existing'] += 1; continue
                    ft = b.f(2)
                    cnt[('n_key', 'n_inter', 'n_intra_only', 'n_switch')[ft]] += 1
            if seq is None: raise ValueError('no sequence header')
            row.pop('error', None); row['container'] = name; row.update(seq); row.update(cnt)
            row['n_frames'] = cnt['n_key'] + cnt['n_intra_only'] + cnt['n_inter'] + cnt['n_switch']
            row['intra_only_stream'] = int(cnt['n_inter'] == 0 and cnt['n_switch'] == 0 and row['n_frames'] > 0)
            return row
        except (ValueError, IndexError) as e:
            row['error'] = f'{name}: {e}'
    return row


COLS = ['path', 'container', 'profile', 'bitdepth', 'mono', 'ss_x', 'ss_y', 'max_w', 'max_h', 'sb128',
        'still', 'reduced_hdr', 'filter_intra', 'intra_edge', 'superres', 'cdef', 'lr', 'film_grain',
        'n_key', 'n_intra_only', 'n_inter', 'n_switch', 'n_show_existing', 'n_frames', 'intra_only_stream', 'error']


def main(argv):
    out = None
    if '-o' in argv:
        i = argv.index('-o'); out = argv[i + 1]; argv = argv[:i] + argv[i + 2:]
    files = []
    for a in argv:
        if os.path.isdir(a):
            for r, _, fs in os.walk(a):
                files += [os.path.join(r, f) for f in fs if f.endswith('.obu')]
        else: files.append(a)
    files.sort()
    w = csv.DictWriter(open(out, 'w', newline='') if out else sys.stdout, fieldnames=COLS, extrasaction='ignore')
    w.writeheader()
    for f in files:
        w.writerow(index_stream(f))


if __name__ == '__main__':
    main(sys.argv[1:])
