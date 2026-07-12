#!/usr/bin/env python3
"""
Box3D float -> Q48.16 fixed-point source rewriter. See conversion notes.

Pass 1 (annotate): walk the clang JSON AST in exact document order resolving
sparse location info ("file" appears only when it changes vs the previously
printed location).
Pass 2 (collect): find float-typed operations and emit text edits.
Apply: insertion-only edits (plus tiny token replacements), verified against
the actual source text where possible.

Modes:
  --verify   only measure literal-location accuracy, change nothing
  (default)  collect and apply edits, write manual_review.txt
"""
import json, os, re, subprocess, sys
from collections import defaultdict

ROOT = "/Users/glenn/box3d"
BUILD = os.path.join(ROOT, "build-ast")
OUR_DIRS = [os.path.join(ROOT, d) for d in ("src", "include/box3d", "test", "shared", "benchmark")]
EXCLUDE_FILES = {os.path.join(ROOT, "src/verstable.h"),
                 os.path.join(ROOT, "include/box3d/fixed.h")}

CALL_RENAMES = {
    "sqrtf": "b3FixSqrt",
    "fabsf": "b3FixAbs",
    "floorf": "b3FixFloor",
    "ceilf": "b3FixCeil",
    "sinf": "b3Sin",
    "cosf": "b3Cos",
}
MANUAL_CALLS = {"remainderf", "fmodf", "atan2f", "tanf", "acosf", "asinf",
                "powf", "expf", "logf", "nextafterf", "roundf", "truncf", "copysignf"}

FLOAT_LIT_RE = re.compile(r"^(\d+\.?\d*([eE][+-]?\d+)?[fF]?|\.\d+([eE][+-]?\d+)?[fF]?|0[xX][0-9a-fA-F.pP+-]+[fF]?)$")

def is_ours(path):
    if path is None:
        return False
    path = os.path.normpath(path)
    if path in EXCLUDE_FILES:
        return False
    return any(path.startswith(d + os.sep) or os.path.dirname(path) == d for d in OUR_DIRS)

# ---------------------------------------------------------------------------
# Pass 1: annotate locations in document order
# ---------------------------------------------------------------------------
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
                loc["__res"] = (spr[0], spr[1], spr[2], False)
            else:
                loc["__res"] = (exr[0], exr[1], exr[2], True)  # macro body
        else:
            f, o, t = self.track(loc)
            loc["__res"] = (f, o, t, False)

    def track(self, loc):
        if "file" in loc:
            self.file = loc["file"]
        return (self.file, loc.get("offset"), loc.get("tokLen", 0))

def res(loc):
    if loc is None or "__res" not in loc:
        return (None, None, None, False)
    return loc["__res"]

# ---------------------------------------------------------------------------
# Edits
# ---------------------------------------------------------------------------
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
manual = []
stats = defaultdict(int)

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

def add_wrap(path, s_off, end, open_text):
    edits.add(path, s_off, 1, -end, open_text)
    # tag the close with the open text so two wraps sharing the exact same span
    # (e.g. b3FixToDouble around exactly a b3FixMul) don't dedupe to one close
    edits.add(path, end, 0, -s_off, " )", 0, tag=open_text)

# ---------------------------------------------------------------------------
# Pass 2: semantic walk
# ---------------------------------------------------------------------------
def qual(n):
    return n.get("type", {}).get("qualType", "")

def strip_wrap(n):
    while n.get("kind") in ("ParenExpr", "ImplicitCastExpr", "ConstantExpr") and n.get("inner"):
        n = n["inner"][0]
    return n

def node_span(n):
    """(file, s_off, end_off_exclusive, macro_body)"""
    r = n.get("range")
    if not r:
        return (None, None, None, False)
    bf, bo, bt, bmac = res(r.get("begin"))
    ef, eo, et, emac = res(r.get("end"))
    if bo is None or eo is None or bf != ef:
        return (bf, None, None, bmac or emac)
    return (bf, bo, eo + (et or 0), bmac or emac)

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

def collect(n, verify_only=False):
    if isinstance(n, list):
        for c in n:
            collect(c, verify_only)
        return
    if not isinstance(n, dict):
        return
    kind = n.get("kind")

    if kind == "FloatingLiteral" and qual(n) == "float":
        f, so, end, mac = node_span(n)
        if is_ours(f):
            if mac:
                manual.append((f, so, "float literal in macro body"))
            elif so is not None:
                txt = text_at(f, so, end - so)
                if FLOAT_LIT_RE.match(txt):
                    stats["lit_ok"] += 1
                    if not verify_only:
                        add_wrap(f, so, end, "B3_FIX( ")
                else:
                    stats["lit_bad"] += 1
                    manual.append((f, so, f"literal text mismatch {txt!r}"))
        return

    if kind == "BinaryOperator" and n.get("opcode") in ("*", "/") and qual(n) == "float":
        f, so, end, mac = node_span(n)
        inner = n.get("inner", [])
        if is_ours(f) and len(inner) == 2 and not verify_only:
            if mac or so is None:
                manual.append((f, so, f"float '{n.get('opcode')}' in macro body"))
            else:
                lf, lso, lend, lmac = node_span(inner[0])
                rf, rso, rend, rmac = node_span(inner[1])
                if None in (lso, rso) or lmac or rmac or lf != f or rf != f:
                    manual.append((f, so, "float binop with macro operand"))
                else:
                    gap = text_at(f, lend, rso - lend)
                    op = n["opcode"]
                    idx = find_op(gap, op)
                    if idx is None:
                        manual.append((f, so, f"cannot find '{op}' token"))
                    else:
                        fn = "b3FixMul( " if op == "*" else "b3FixDiv( "
                        edits.add(f, so, 1, -end, fn)
                        edits.add(f, lend + idx, 2, 0, ",", 1)
                        edits.add(f, end, 0, -so, " )", 0, tag=fn)
                        stats["binop"] += 1
        collect(inner, verify_only)
        return

    if kind == "CompoundAssignOperator" and n.get("opcode") in ("*=", "/=") and qual(n) == "float":
        f, so, end, mac = node_span(n)
        inner = n.get("inner", [])
        if is_ours(f) and len(inner) == 2 and not verify_only:
            lf, lso, lend, lmac = node_span(inner[0])
            rf, rso, rend, rmac = node_span(inner[1])
            lhs_text = text_at(f, lso, lend - lso) if lso is not None else ""
            if mac or lmac or rmac or None in (lso, rso):
                manual.append((f, so, "float compound assign in macro"))
            elif inner[0].get("kind") not in ("DeclRefExpr", "MemberExpr", "ArraySubscriptExpr") \
                    or len(lhs_text) > 60 or "(" in lhs_text:
                manual.append((f, so, f"complex compound-assign lhs {lhs_text!r}"))
            else:
                gap = text_at(f, lend, rso - lend)
                op = n["opcode"]
                idx = gap.find(op)
                if idx < 0:
                    manual.append((f, so, f"cannot find '{op}'"))
                else:
                    fn = "b3FixMul" if op == "*=" else "b3FixDiv"
                    edits.add(f, lend + idx, 2, 0, f"= {fn}( {lhs_text},", 2)
                    edits.add(f, end, 0, -so, " )", 0, tag=fn + "=")
                    stats["compound"] += 1
        collect(inner, verify_only)
        return

    if kind in ("ImplicitCastExpr", "CStyleCastExpr"):
        ck = n.get("castKind")
        inner = n.get("inner", [])
        if len(inner) == 1 and not verify_only:
            sub = inner[0]
            sf, sso, send, smac = node_span(sub)
            to_type = qual(n)
            from_type = qual(sub)
            wrap = None
            if ck == "IntegralToFloating" and to_type == "float":
                wrap = "b3FixFromInt( "
            elif ck == "FloatingToIntegral" and from_type == "float":
                wrap = "b3FixTruncToInt( "
            elif ck == "FloatingCast" and from_type == "float" and to_type == "double":
                wrap = "b3FixToDouble( "
            elif ck == "FloatingCast" and from_type == "double" and to_type == "float":
                wrap = "B3_FIX( " if strip_wrap(sub).get("kind") == "FloatingLiteral" else "b3FixFromDouble( "
            if wrap:
                if not is_ours(sf):
                    pass
                elif smac or sso is None:
                    manual.append((sf, sso, f"cast ({ck}) inside macro body"))
                else:
                    add_wrap(sf, sso, send, wrap)
                    stats["cast"] += 1
        collect(inner, verify_only)
        return

    if kind == "DeclRefExpr":
        name = n.get("referencedDecl", {}).get("name")
        if name in CALL_RENAMES or name in MANUAL_CALLS:
            f, so, end, mac = node_span(n)
            if is_ours(f) and not mac and so is not None and text_at(f, so, len(name)) == name:
                if name in CALL_RENAMES:
                    if not verify_only:
                        edits.add(f, so, 2, 0, CALL_RENAMES[name], len(name))
                    stats["call"] += 1
                else:
                    manual.append((f, so, f"manual libm call {name}"))
        return

    if kind == "UnaryOperator" and n.get("opcode") in ("++", "--") and qual(n) == "float":
        f, so, end, mac = node_span(n)
        manual.append((f, so, f"float {n.get('opcode')}"))

    for k, v in n.items():
        if k in ("loc", "range"):
            continue
        if isinstance(v, (dict, list)):
            collect(v, verify_only)

# ---------------------------------------------------------------------------
def main():
    verify_only = "--verify" in sys.argv
    cc = json.load(open(os.path.join(BUILD, "compile_commands.json")))
    tus = [e for e in cc if e["file"].endswith(".c")
           and any(f"/{d}/" in e["file"] for d in ("src", "test", "shared", "benchmark"))
           and "extern" not in e["file"]]
    print(f"{len(tus)} TUs")
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
        collect(ast, verify_only)
        sys.stdout.write(f"\r[{i+1}/{len(tus)}] {os.path.basename(entry['file']):28s}")
        sys.stdout.flush()
    print()
    print("stats:", dict(stats))

    if verify_only:
        return

    total = 0
    for path, ops in sorted(edits.by_file.items()):
        if not is_ours(path):
            continue
        src = file_text(path)
        ops.sort(key=lambda e: (e[0], e[1], e[2]))
        outparts, cur, bad = [], 0, False
        for pos, prio, tie, text, dellen in ops:
            if pos < cur:
                print(f"OVERLAP in {path} at {pos}: {text!r}")
                bad = True
                continue
            outparts.append(src[cur:pos])
            outparts.append(text.encode("utf-8"))
            cur = pos + dellen
        outparts.append(src[cur:])
        open(path, "wb").write(b"".join(outparts))
        total += len(ops)
        print(f"applied {len(ops):5d} edits -> {os.path.relpath(path, ROOT)}")
    print("total edits:", total)

    report = os.path.join(os.path.dirname(os.path.abspath(__file__)), "manual_review.txt")
    with open(report, "w") as f:
        seen = set()
        for path, off, reason in manual:
            if not is_ours(path):
                continue
            key = (path, off, reason)
            if key in seen:
                continue
            seen.add(key)
            line = file_text(path).count(b"\n", 0, off or 0) + 1
            f.write(f"{os.path.relpath(path, ROOT)}:{line}: {reason}\n")
    print("manual review ->", report)

if __name__ == "__main__":
    main()
