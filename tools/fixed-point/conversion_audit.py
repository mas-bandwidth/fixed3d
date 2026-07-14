#!/usr/bin/env python3
"""
Type-aware fixed/float boundary audit.

Finds every implicit conversion between b3Fixed and float/double in the AST,
in both directions, plus explicit casts between them outside the sanctioned
converters. This closes the two blind spots of the warning-based recipe
(-Wfloat-conversion -Wimplicit-int-float-conversion):

  * value-preserving float literals never warn: b3CreateCylinder( 2.0f, ... )
    converts 2.0f to raw int64 2 (30 micrometers) silently — this exact class
    shipped a NULL-hull crash (micro torus in the mover samples);
  * small b3Fixed values into float never warn: raw values under 2^24 convert
    to float exactly, so `float x = someFixed` is silent while still reading
    raw bits (65536x the intended value).

Warnings are value-based; the convention is type-based: b3Fixed must cross to
float via b3FixToFloat/b3FixToDouble and back via b3FixFromFloat/B3_FIX, so
EVERY implicit conversion between the two type families is a finding.

Usage:
  conversion_audit.py [--db <compile_commands.json>] [--all]

Defaults to build-samples/compile_commands.json (covers src, shared, samples,
gfx, host). Report-only: each finding needs a judgment call (value fix vs
restructure to fixed math), so there is no --fix. Expected output on a clean
tree: nothing but the per-TU progress line.
"""
import json, os, subprocess, sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUR_DIRS = [os.path.join(ROOT, d) for d in ("src", "include/box3d", "test", "shared", "benchmark", "samples")]
# fixed.h implements the sanctioned converters; verstable keeps internal floats
# by design; the dump inl is generated data audited separately.
EXCLUDE_FILES = {os.path.join(ROOT, "src/verstable.h"),
                 os.path.join(ROOT, "include/box3d/fixed.h")}
EXCLUDE_DIR_PARTS = (os.sep + "extern" + os.sep, ".fetchcontent-cache", os.sep + "shaders" + os.sep)

def is_ours(path):
    if path is None:
        return False
    path = os.path.normpath(path)
    if path in EXCLUDE_FILES or any(p in path for p in EXCLUDE_DIR_PARTS):
        return False
    return any(path.startswith(d + os.sep) or os.path.dirname(path) == d for d in OUR_DIRS)

def is_loc_dict(d):
    return isinstance(d, dict) and ("offset" in d or "spellingLoc" in d or "expansionLoc" in d)

class Annotator:
    """Resolve clang's sparse file/offset location encoding (see ast_audit.py)."""
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
            loc["__res"] = spr if ex.get("isMacroArgExpansion", False) else exr
            # Where the token is literally spelled. A cast spelled inside
            # fixed.h's converter macros (B3_FIX and friends) is the sanctioned
            # conversion machinery, not a finding at the expansion site.
            loc["__spell"] = spr
        else:
            loc["__res"] = self.track(loc)
            loc["__spell"] = loc["__res"]
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

def lineno(path, off):
    return file_text(path).count(b"\n", 0, off or 0) + 1

def node_span(n):
    r = n.get("range")
    if not r:
        return (None, None, None)
    bf, bo, bt = res(r.get("begin"))
    ef, eo, et = res(r.get("end"))
    if bo is None or eo is None or bf != ef:
        return (bf, bo, None)
    return (bf, bo, eo + (et or 0))

def snippet(path, so, end, limit=90):
    if so is None:
        return "?"
    raw = file_text(path)[so:(end if end else so + limit)]
    s = raw.decode("utf-8", "replace").split("\n")[0].strip()
    return s[:limit]

def qual(n):
    return n.get("type", {}).get("qualType", "")

def is_fixed_type(q):
    # b3Fixed with any cv qualifiers; the typedef survives in qualType wherever
    # the declaration used it, which is the convention everywhere in this tree.
    return "b3Fixed" in q

def is_floating_type(q):
    base = q.replace("const", "").replace("volatile", "").strip()
    return base in ("float", "double", "long double")

report = []
stats = defaultdict(int)

def first_expr_child(n):
    for c in n.get("inner", []):
        if isinstance(c, dict) and c.get("kind", "").endswith(("Expr", "Literal", "Operator", "CallExpr")):
            return c
    inner = n.get("inner", [])
    return inner[0] if inner and isinstance(inner[0], dict) else {}

def spelling_file(n):
    r = n.get("range")
    if not r:
        return None
    sp = (r.get("begin") or {}).get("__spell")
    return sp[0] if sp else None

def add_finding(n, category):
    f, so, end = node_span(n)
    if not is_ours(f):
        return
    # Skip casts whose tokens are spelled inside excluded files (the B3_FIX /
    # b3FixFromFloat machinery in fixed.h expands an explicit cast at every
    # legitimate use site).
    sf = spelling_file(n)
    if sf is not None and not is_ours(sf):
        return
    report.append((f, lineno(f, so), category, snippet(f, so, end)))
    stats[category] += 1

def collect(n):
    if isinstance(n, list):
        for c in n:
            collect(c)
        return
    if not isinstance(n, dict):
        return
    kind = n.get("kind")

    if kind == "ImplicitCastExpr":
        ck = n.get("castKind")
        if ck == "FloatingToIntegral" and is_fixed_type(qual(n)):
            add_finding(n, "float->b3Fixed (implicit, raw truncation)")
        elif ck == "IntegralToFloating" and is_fixed_type(qual(first_expr_child(n))):
            add_finding(n, "b3Fixed->float (implicit, raw read)")
    elif kind in ("CStyleCastExpr", "CXXStaticCastExpr", "CXXFunctionalCastExpr"):
        ck = n.get("castKind")
        if ck == "FloatingToIntegral" and is_fixed_type(qual(n)):
            add_finding(n, "float->b3Fixed (explicit cast, review)")
        elif ck == "IntegralToFloating" and is_fixed_type(qual(first_expr_child(n))):
            add_finding(n, "b3Fixed->float (explicit cast, review)")

    for k, v in n.items():
        if k in ("loc", "range"):
            continue
        if isinstance(v, (dict, list)):
            collect(v)

def main():
    db = os.path.join(ROOT, "build-samples", "compile_commands.json")
    if "--db" in sys.argv:
        db = sys.argv[sys.argv.index("--db") + 1]
    cc = json.load(open(db))
    tus = [e for e in cc
           if e["file"].endswith((".c", ".cpp"))
           and is_ours(os.path.normpath(e["file"]))]
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
        sys.stdout.write(f"\r[{i+1}/{len(tus)}] {os.path.basename(entry['file']):32s}")
        sys.stdout.flush()
        if not r.stdout:
            print("\nAST FAIL", entry["file"], r.stderr[:300])
            continue
        ast = json.loads(r.stdout)
        Annotator().run(ast)
        collect(ast)
    print()
    print("stats:", dict(stats) if stats else "clean")

    seen = set()
    for f, ln, category, snip in sorted(report):
        key = (f, ln, category)
        if key in seen:
            continue
        seen.add(key)
        print(f"REVIEW {os.path.relpath(f, ROOT)}:{ln}: {category}: {snip}")
    if report:
        sys.exit(1)

if __name__ == "__main__":
    main()
