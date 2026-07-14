#!/usr/bin/env python3
"""Pair fixed-tree and float-tree sample captures by category/name, compute
difference metrics, and emit a self-contained HTML contact sheet (thumbnails
inlined as data URIs), sorted so the most-different pairs surface first."""
import base64, io, os, sys
from PIL import Image, ImageChops, ImageStat

FIXED_DIR, FLOAT_DIR, OUT_HTML = sys.argv[1], sys.argv[2], sys.argv[3]

def load_index(d):
    idx = {}
    for line in open(os.path.join(d, "samples.tsv")):
        parts = line.rstrip("\n").split("\t")
        if len(parts) != 3:
            continue
        i, cat, name = parts
        safe = "".join(c for c in f"{cat}__{name}".replace(" ", "_").replace("/", "_")
                       if c.isalnum() or c in "_.-")
        png = os.path.join(d, safe + ".png")
        idx[(cat, name)] = png if os.path.exists(png) else None
    return idx

fixed = load_index(FIXED_DIR)
flt = load_index(FLOAT_DIR)

def thumb_datauri(path, size=(360, 202)):
    im = Image.open(path).convert("RGB")
    im.thumbnail(size)
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=62)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()

def gray_small(path):
    return Image.open(path).convert("L").resize((64, 36))

def metrics(fp, gp):
    a, b = gray_small(fp), gray_small(gp)
    diff = ImageStat.Stat(ImageChops.difference(a, b)).mean[0]
    return diff

def uniformity(path):
    return ImageStat.Stat(gray_small(path)).stddev[0]

rows, fixed_only, missing = [], [], []
for key, fpng in sorted(fixed.items()):
    cat, name = key
    gpng = flt.get(key)
    if fpng is None:
        missing.append((cat, name, "fixed capture missing"))
        continue
    ustd = uniformity(fpng)
    if gpng:
        d = metrics(fpng, gpng)
        flag = ""
        if ustd < 3.0 and uniformity(gpng) >= 3.0:
            flag = "EMPTY-FIXED?"
        elif uniformity(gpng) < 3.0 and ustd >= 3.0:
            flag = "EMPTY-FLOAT?"
        rows.append((d, cat, name, fpng, gpng, flag))
    else:
        fixed_only.append((cat, name, fpng, "EMPTY?" if ustd < 3.0 else ""))

float_only = [k for k in flt if k not in fixed and flt[k]]

rows.sort(reverse=True)
h = ["<!-- contact sheet -->",
     "<title>Fixed3D vs float Box3D — per-sample end states</title>",
     "<style>body{font-family:system-ui;margin:1rem;background:#161618;color:#ddd}"
     "table{border-collapse:collapse}td{padding:4px 8px;vertical-align:top}"
     "img{display:block;border-radius:4px}h2{margin-top:2rem}"
     ".flag{color:#ff6b6b;font-weight:700}.d{color:#9ad}</style>",
     "<h1>Fixed3D @ 100 km vs vanilla float Box3D @ origin</h1>",
     f"<p>{len(rows)} matched samples, {len(fixed_only)} fixed-only, "
     f"{len(float_only)} float-only. 120 frames each, same renderer, same authored "
     "cameras. Sorted by end-state difference (chaotic scenes diverge legitimately; "
     "look for empty/garbage frames, not pixel equality).</p>",
     "<table><tr><th>diff</th><th>sample</th><th>Fixed3D (100 km)</th><th>float (origin)</th></tr>"]
for d, cat, name, fpng, gpng, flag in rows:
    h.append(f"<tr><td class=d>{d:.1f}</td><td><b>{cat}</b><br>{name}"
             f"{'<br><span class=flag>' + flag + '</span>' if flag else ''}</td>"
             f"<td><img src='{thumb_datauri(fpng)}'></td>"
             f"<td><img src='{thumb_datauri(gpng)}'></td></tr>")
h.append("</table><h2>Fixed3D-only samples</h2><table>")
for cat, name, fpng, flag in fixed_only:
    h.append(f"<tr><td><b>{cat}</b><br>{name}"
             f"{'<br><span class=flag>' + flag + '</span>' if flag else ''}</td>"
             f"<td><img src='{thumb_datauri(fpng)}'></td></tr>")
h.append("</table><h2>Float-only samples (no fixed counterpart)</h2><ul>")
for cat, name in sorted(float_only):
    h.append(f"<li>{cat} / {name}</li>")
h.append("</ul>")
if missing:
    h.append("<h2 class=flag>Missing captures</h2><ul>")
    for cat, name, why in missing:
        h.append(f"<li>{cat} / {name}: {why}</li>")
    h.append("</ul>")
open(OUT_HTML, "w").write("\n".join(h))
print(f"wrote {OUT_HTML}: {len(rows)} pairs, {len(fixed_only)} fixed-only, "
      f"{len(float_only)} float-only, {len(missing)} missing")
print("top diffs:")
for d, cat, name, *_ in rows[:10]:
    print(f"  {d:7.1f}  {cat} / {name}  {_[-1] if _ else ''}")
