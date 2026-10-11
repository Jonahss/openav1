#!/usr/bin/env python3
"""Report module-scope identifiers used before their declaration line (Vivado turns such uses into implicit
1-bit nets, with only a warning: [Synth 8-6901] / [Synth 8-8895]; Verilator accepts them silently).
Usage: tools/check_decl_order.py rtl/*.sv   (exit 1 if anything is reported)"""
import re, sys
decl_re = re.compile(r'^\s*(?:\(\*[^)]*\*\)\s*)?(?:logic|wire|reg|bit|int|integer|[A-Za-z_]\w*_t)\b(?:\s+(?:signed|unsigned))?\s*((?:\[[^\]]*\]\s*)*)([^;=]*?)(?:=[^;]*)?;')
typedef_re = re.compile(r'\}\s*(\w+)\s*;')     # end of a typedef enum/struct: "} name_t;"
enum_start = re.compile(r'^\s*typedef\s+enum')
bad = 0
for path in sys.argv[1:]:
    text = open(path).read()
    lines = text.split('\n')
    # strip comments and strings for the use scan
    code = []
    in_block = False
    for l in lines:
        s = l
        if in_block:
            if '*/' in s: s = s[s.index('*/') + 2:]; in_block = False
            else: s = ''
        s = re.sub(r'"[^"]*"', '""', s)
        if '/*' in s and not in_block:
            if '*/' in s: s = s[:s.index('/*')] + s[s.index('*/') + 2:]
            else: s = s[:s.index('/*')]; in_block = True
        s = s.split('//')[0]
        code.append(s)
    # find module-scope declarations (indent of 4 spaces = module scope in this repo) and typedef names/enum members
    decl = {}
    i = 0
    in_enum = False; enum_members = []
    while i < len(code):
        s = code[i]
        if enum_start.match(s) and s.startswith('    typedef'):
            in_enum = True; enum_members = []; enum_line = i + 1
        if in_enum:
            for m in re.finditer(r'\b([A-Z][A-Z0-9_]+)\b', s):
                if not s.strip().startswith('typedef'): enum_members.append(m.group(1))
            t = typedef_re.search(s)
            if t:
                for e in enum_members + [t.group(1)]: decl.setdefault(e, enum_line)
                in_enum = False
            i += 1; continue
        if s.startswith('    ') and not s.startswith('     '):
            # may hold several declarations on one line
            for stmt in s.split(';'):
                stmt = stmt.strip()
                m = decl_re.match('    ' + stmt + ';')
                if not m: continue
                names = m.group(2)
                for part in names.split(','):
                    part = part.strip()
                    nm = re.match(r'([A-Za-z_]\w*)', part)
                    if nm and nm.group(1) not in ('input', 'output', 'inout', 'parameter', 'localparam'):
                        decl.setdefault(nm.group(1), i + 1)
        i += 1
    args = set(re.findall(r'\b(?:input|output|inout)\s+(?:[\w\[\]:\s-]*?\s)?(\w+)\s*[,)]', text))
    args |= set(re.findall(r'for\s*\(\s*int\s+(\w+)', text))
    # ports / params are declared in the header: anything before "    " first body line counts as declared at 0
    hdr_end = next((k for k, l in enumerate(lines) if re.match(r'^\);', l)), 0)
    for name, d in decl.items():
        if d <= hdr_end + 1 or name in args: continue
        pat = re.compile(r'(?<![\w.])' + re.escape(name) + r'(?!\w)')
        for k in range(hdr_end, d - 1):
            if pat.search(code[k]):
                print(f"{path}:{k + 1}: '{name}' used before its declaration at line {d}")
                bad += 1
                break
sys.exit(1 if bad else 0)
