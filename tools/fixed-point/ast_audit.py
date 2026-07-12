#!/usr/bin/env python3
"""
Post-conversion audit: find and fix every remaining raw multiply/divide between
b3Fixed values in the converted tree. These compile fine as int64 ops but are
semantically wrong (missing the Q48.16 rescale). Macro-operand expressions such
as `B3_FIX( 0.25f ) * B3_PI` resolve their endpoints to the macro invocation
tokens, which are real editable code.

Also reports (without fixing): int/b3Fixed divisions, remaining float/double
binops outside allowed files, compound assigns, and unresolvable macro-body ops.
"""
import json, os, subprocess, sys
from collections import defaultdict

# Repo root derived from this script's location (tools/fixed-point/../..)
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BUILD = os.path.join(ROOT, "build-ast")
OUR_DIRS = [os.path.join(ROOT, d) for d in ("src", "include/box3d", "test", "shared", "benchmark")]
EXCLUDE_FILES = {os.path.join(ROOT, "src/verstable.h"),
                 os.path.join(ROOT, "include/box3d/fixed.h")}
FLOAT_OK_FILES = {os.path.join(ROOT, p) for p in ("src/timer.c", "test/test_math.c", "test/test_bitset.c", "test/test_shape.c")}

FIXED = ("b3Fixed", "const b3Fixed", "volatile b3Fixed")

def is_ours(path):
    if path is None:
        return False
    path = os.path.normpath(path)
    if path in EXCLUDE_FILES:
        return False
    return any(path.startswith(d + os.sep) or os.path.dirname(path) == d for d in OUR_DIRS)

def is_loc_dict(d):
    return isinstance(d, dict) and ("offset" in d or "spellingLoc" in d or "expansionLoc" in d)

class Annotator:
    def __init__(self):
        self.file = None
    def run(self, n):
        if isinstance(n, list):
            for c in n:
                self.run(c)
        elif isinstance(n, dict):
            for k, v in n.items():
                if is_loc_dict(v):
                    self.resolve(v)
                elif isinstance(v, (dict, list)):
                    self.run(v)
    def resolve(self, loc):
        if "spellingLoc" in loc or "expansionLoc" in loc:
            sp = loc.get("spellingLoc", {})
            ex = loc.get("expansionLoc", {})
            spr = self.track(sp)
            exr = self.track(ex)
            if ex.get("isMacroArgExpansion", False):
                loc["__res"] = spr        # macro argument: spelling is real code
            else:
                loc["__res"] = exr        # macro name token at the use site
        else:
            loc["__res"] = self.track(loc)
    def track(self, loc):
        if "file" in loc:
            self.file = loc["file"]
        return (self.file, loc.get("offset"), loc.get("tokLen", 0))

def res(loc):
    if loc is None or "__res" not in loc:
        return (None, None, None)
    return loc["__res"]

_text_cache = {}
def file_text(path):
    t = _text_cache.get(path)
    if t is None:
        try:
            t = open(path, "rb").read()
        except OSError:
            t = b""
        _text_cache[path] = t
    return t

def text_at(path, off, length):
    return file_text(path)[off:off + length].decode("utf-8", "replace")

class Edits:
    def __init__(self):
        self.by_file = defaultdict(list)
        self.seen = set()
    def add(self, path, pos, prio, tie, text, dellen=0, tag=None):
        key = (path, pos, prio, tie, text, dellen, tag)
        if key in self.seen:
            return
        self.seen.add(key)
        self.by_file[path].append((pos, prio, tie, text, dellen))

edits = Edits()
report = []
stats = defaultdict(int)

def qual(n):
    return n.get("type", {}).get("qualType", "")

def strip_wrap(n):
    while n.get("kind") in ("ParenExpr", "ImplicitCastExpr", "ConstantExpr") and n.get("inner"):
        n = n["inner"][0]
    return n

def is_int_literal(n):
    return strip_wrap(n).get("kind") == "IntegerLiteral"

def node_span(n):
    r = n.get("range")
    if not r:
        return (None, None, None)
    bf, bo, bt = res(r.get("begin"))
    ef, eo, et = res(r.get("end"))
    if bo is None or eo is None or bf != ef:
        return (bf, None, None)
    return (bf, bo, eo + (et or 0))

def find_op(gap, op):
    i = 0
    while i < len(gap):
        c = gap[i]
        if c == "/" and i + 1 < len(gap) and gap[i + 1] == "*":
            j = gap.find("*/", i + 2)
            if j < 0:
                return None
            i = j + 2
            continue
        if c == "/" and i + 1 < len(gap) and gap[i + 1] == "/":
            j = gap.find("\n", i)
            if j < 0:
                return None
            i = j + 1
            continue
        if c == op and (op != "/" or (i + 1 >= len(gap)) or gap[i + 1] not in "*/"):
            return i
        i += 1
    return None

def lineno(path, off):
    return file_text(path).count(b"\n", 0, off or 0) + 1

def collect(n):
    if isinstance(n, list):
        for c in n:
            collect(c)
        return
    if not isinstance(n, dict):
        return
    kind = n.get("kind")

    if kind == "BinaryOperator" and n.get("opcode") in ("*", "/"):
        inner = n.get("inner", [])
        if len(inner) == 2:
            lt, rt = qual(inner[0]), qual(inner[1])
            t = qual(n)
            f, so, end = node_span(n)
            if is_ours(f):
                op = n["opcode"]
                if lt in FIXED and rt in FIXED and not is_int_literal(inner[0]) and not is_int_literal(inner[1]):
                    lf, lso, lend = node_span(inner[0])
                    rf, rso, rend = node_span(inner[1])
                    if so is None or None in (lso, rso) or lf != f or rf != f or not (so <= lso < rso <= end):
                        report.append((f, lineno(f, so), f"UNFIXABLE fixed {op} (macro body?)"))
                    else:
                        gap = text_at(f, lend, rso - lend)
                        idx = find_op(gap, op)
                        if idx is None:
                            report.append((f, lineno(f, so), f"cannot find '{op}' token"))
                        else:
                            fn = "b3FixMul( " if op == "*" else "b3FixDiv( "
                            edits.add(f, so, 1, -end, fn)
                            edits.add(f, lend + idx, 2, 0, ",", 1)
                            edits.add(f, end, 0, -so, " )", 0, tag=fn)
                            stats["fixed_" + ("mul" if op == "*" else "div")] += 1
                elif op == "/" and rt in FIXED and lt not in FIXED and lt in ("int", "long long", "int64_t", "unsigned int"):
                    report.append((f, lineno(f, so), f"int / b3Fixed division needs review: {lt} / {rt}"))
                elif t in ("float", "double") and os.path.normpath(f) not in FLOAT_OK_FILES:
                    report.append((f, lineno(f, so), f"remaining {t} {op}"))
        collect(inner)
        return

    if kind == "CompoundAssignOperator" and n.get("opcode") in ("*=", "/=") :
        t = qual(n)
        if t in FIXED:
            f, so, end = node_span(n)
            if is_ours(f):
                report.append((f, lineno(f, so), f"fixed compound assign {n.get('opcode')}"))

    for k, v in n.items():
        if k in ("loc", "range"):
            continue
        if isinstance(v, (dict, list)):
            collect(v)

def main():
    cc = json.load(open(os.path.join(BUILD, "compile_commands.json")))
    tus = [e for e in cc if e["file"].endswith(".c")
           and any(f"/{d}/" in e["file"] for d in ("src", "test", "shared", "benchmark"))
           and "extern" not in e["file"]]
    for i, entry in enumerate(tus):
        args = entry["command"].split()
        out, skip = [], False
        for a in args[1:]:
            if skip:
                skip = False
                continue
            if a in ("-o", "-MT", "-MF"):
                skip = True
                continue
            if a in ("-c", "-MD"):
                continue
            out.append(a)
        cmd = [args[0]] + out + ["-Xclang", "-ast-dump=json", "-fsyntax-only"]
        r = subprocess.run(cmd, capture_output=True, text=True, cwd=entry["directory"])
        if not r.stdout:
            print("AST FAIL", entry["file"], r.stderr[:300])
            continue
        ast = json.loads(r.stdout)
        Annotator().run(ast)
        collect(ast)
        sys.stdout.write(f"\r[{i+1}/{len(tus)}] {os.path.basename(entry['file']):28s}")
        sys.stdout.flush()
    print()
    print("stats:", dict(stats))

    apply_edits = "--fix" in sys.argv
    if apply_edits:
        total = 0
        for path, ops in sorted(edits.by_file.items()):
            src = file_text(path)
            ops.sort(key=lambda e: (e[0], e[1], e[2]))
            outparts, cur = [], 0
            ok = True
            for pos, prio, tie, text, dellen in ops:
                if pos < cur:
                    print(f"OVERLAP {path}:{pos}")
                    ok = False
                    break
                outparts.append(src[cur:pos])
                outparts.append(text.encode())
                cur = pos + dellen
            if not ok:
                continue
            outparts.append(src[cur:])
            open(path, "wb").write(b"".join(outparts))
            total += len(ops)
            print(f"fixed {len(ops):4d} edits -> {os.path.relpath(path, ROOT)}")
        print("total:", total)

    seen = set()
    for f, ln, msg in report:
        key = (f, ln, msg)
        if key in seen:
            continue
        seen.add(key)
        print(f"REVIEW {os.path.relpath(f, ROOT)}:{ln}: {msg}")

if __name__ == "__main__":
    main()
