#!/usr/bin/env python3
"""Capture matching samples and compare decoded pixels, with repeatability checks.

Requires macOS capture-enabled binaries and Pillow. Hash equality is exact image
identity, not proof of physics equivalence. Float/fixed numerical drift is expected.
"""
import argparse
import hashlib
import html
import json
import re
import subprocess
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

from PIL import Image, ImageChops, ImageStat


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def pixels(path):
    with Image.open(path) as source:
        image = source.convert('RGBA')
    header = f'RGBA:{image.width}x{image.height}\0'.encode('ascii')
    digest = hashlib.sha256(header + image.tobytes()).hexdigest()
    return image, digest


def enumerate_samples(binary, cwd):
    result = subprocess.run([str(binary), '--list-samples'], cwd=cwd,
                            capture_output=True, text=True, check=True, timeout=30)
    samples = {}
    for line in result.stdout.splitlines():
        index, category, name = line.split('\t', 2)
        key = f'{category}/{name}'
        if key in samples:
            raise ValueError(f'duplicate sample name: {key}')
        samples[key] = int(index)
    return samples


def run(args):
    pattern = re.compile(args.match)
    frames = args.frames
    if args.seconds is not None:
        count = args.seconds * 60
        if not count.is_finite() or count != count.to_integral_value() or count < 0:
            raise ValueError('--seconds must be a nonnegative multiple of 1/60')
        frames = int(count)
    if frames < 0 or args.repeat < 1 or args.timeout <= 0:
        raise ValueError('frames must be nonnegative; repeat and timeout must be positive')
    out = args.output.resolve()
    # Refuse reuse: a stale PNG must never count as a successful new capture.
    out.mkdir(parents=True, exist_ok=False)
    builds = {}
    for label in ('fixed', 'float'):
        binary = getattr(args, label).resolve(strict=True)
        data = getattr(args, label + '_data').resolve(strict=True)
        cwd = out / label / 'cwd'
        cwd.mkdir(parents=True)
        (cwd / 'data').symlink_to(data, target_is_directory=True)
        builds[label] = dict(binary=str(binary), binary_sha256=sha256(binary),
                             data=str(data), samples=enumerate_samples(binary, cwd))
    names = sorted(k for k in set(builds['fixed']['samples']) | set(builds['float']['samples'])
                   if pattern.search(k))
    if not names:
        raise ValueError('no samples match the expression')
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), frames=frames,
                  nominal_hz=60, repeat=args.repeat, match=args.match, no_axes=args.no_axes, builds=builds,
                  pixel_hash='SHA256 of ASCII RGBA:WIDTHxHEIGHT followed by NUL and decoded RGBA bytes',
                  rows=[])
    failure = False
    for name in names:
        slug = re.sub(r'[^A-Za-z0-9_.-]', '_', name) + '-' + hashlib.sha256(name.encode()).hexdigest()[:12]
        row = dict(sample=name, captures={}, errors=[])
        for label, build in builds.items():
            if name not in build['samples']:
                row['errors'].append(f'{label}: no matching sample')
                continue
            captures = []
            for repeat in range(args.repeat):
                png = out / label / f'{slug}-{repeat}.png'
                log = png.with_suffix('.log')
                command = [build['binary'], '--headless', '--sample', str(build['samples'][name]),
                           '--frames', str(frames), '--capture', str(png)]
                if args.no_axes:
                    command.append('--capture-no-axes')
                try:
                    with log.open('w') as stream:
                        subprocess.run(command, cwd=out / label / 'cwd', stdout=stream,
                                       stderr=subprocess.STDOUT, check=True, timeout=args.timeout)
                    image, digest = pixels(png)
                    captures.append(dict(png=str(png.relative_to(out)), pixel_sha256=digest,
                                         width=image.width, height=image.height))
                    print(f'{label} {name} run={repeat} pixel_sha256={digest}', flush=True)
                except (subprocess.SubprocessError, OSError, ValueError) as error:
                    row['errors'].append(f'{label} run {repeat}: {error}')
            row['captures'][label] = captures
            if len({c['pixel_sha256'] for c in captures}) > 1:
                row['errors'].append(f'{label}: pixels differ between repeated runs')
        fixed, flt = (row['captures'].get(k, []) for k in ('fixed', 'float'))
        if fixed and flt:
            a, _ = pixels(out / fixed[0]['png'])
            b, _ = pixels(out / flt[0]['png'])
            row['exact_pixel_match'] = fixed[0]['pixel_sha256'] == flt[0]['pixel_sha256']
            if a.size == b.size:
                diff = ImageChops.difference(a.convert('RGB'), b.convert('RGB'))
                row['mean_absolute_rgb_difference'] = sum(ImageStat.Stat(diff).mean) / 3
                channels = diff.split()
                changed = ImageChops.lighter(ImageChops.lighter(channels[0], channels[1]), channels[2])
                row['changed_pixel_percent'] = 100 * (1 - changed.histogram()[0] / (a.width * a.height))
                row['mean_luminance_difference_64x36'] = ImageStat.Stat(ImageChops.difference(
                    a.convert('L').resize((64, 36)), b.convert('L').resize((64, 36)))).mean[0]
            else:
                row['errors'].append('image dimensions differ')
            for label, image in (('fixed', a), ('float', b)):
                thumb = out / label / f'{slug}-thumb.png'
                image.thumbnail((640, 360))
                image.save(thumb)
                row[label + '_thumbnail'] = str(thumb.relative_to(out))
        failure |= bool(row['errors']) or (args.exact and not row.get('exact_pixel_match', False))
        report['rows'].append(row)
        # Preserve completed evidence if a later sample is interrupted.
        (out / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    rows = sorted(report['rows'], key=lambda r: r.get('mean_luminance_difference_64x36', -1), reverse=True)
    doc = ['<!doctype html><meta charset="utf-8"><title>Box3D / Fixed3D sample comparison</title>',
           '<style>body{font:16px system-ui;background:#161618;color:#eee;margin:24px}table{border-collapse:collapse}'
           'td,th{padding:12px;vertical-align:top;text-align:left}img{width:100%;max-width:640px}.error{color:#ff9c9c}</style>',
           '<h1>Box3D / Fixed3D sample comparison</h1>',
           f'<p>{len(rows)} samples; {frames} steps at nominal 60 Hz; {args.repeat} independent runs per build. '
           'Sorted by mean luminance difference at 64×36. Hashes compare full-resolution RGBA pixels. '
           'Different pixels are evidence to inspect, not automatically a physics defect.</p>',
           '<p><a href="report.json">Run settings, binary hashes, pixel hashes and metrics</a></p>',
           '<table><tr><th>Sample / difference</th><th>Fixed3D</th><th>Box3D float</th></tr>']
    for row in rows:
        info = html.escape(row['sample'])
        if 'exact_pixel_match' in row:
            info += '<br>Exact pixels: ' + str(row['exact_pixel_match'])
        if 'changed_pixel_percent' in row:
            info += f"<br>Changed: {row['changed_pixel_percent']:.3f}%<br>RGB MAE: {row['mean_absolute_rgb_difference']:.4f}/255"
        for error in row['errors']:
            info += '<p class="error">' + html.escape(error) + '</p>'
        doc.append('<tr><td>' + info + '</td>')
        for label in ('fixed', 'float'):
            if label + '_thumbnail' in row:
                full = html.escape(row['captures'][label][0]['png'], quote=True)
                thumb = html.escape(row[label + '_thumbnail'], quote=True)
                doc.append(f'<td><a href="{full}"><img src="{thumb}" alt="{label}"></a></td>')
            else:
                doc.append('<td>Capture unavailable</td>')
        doc.append('</tr>')
    doc.append('</table>')
    (out / 'report.html').write_text('\n'.join(doc))
    print(f"Report: {out / 'report.html'}", flush=True)
    return int(failure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for label in ('fixed', 'float'):
        parser.add_argument('--' + label, required=True, type=Path, help='capture-enabled samples binary')
        parser.add_argument('--' + label + '-data', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='new directory for captures and reports')
    duration = parser.add_mutually_exclusive_group()
    duration.add_argument('--frames', type=int, default=120)
    duration.add_argument('--seconds', type=Decimal, help='nominal simulation time, converted to steps at 60 Hz')
    parser.add_argument('--match', default='.', help='regex against Category/Sample Name')
    parser.add_argument('--repeat', type=int, default=2)
    parser.add_argument('--no-axes', action='store_true', help='hide absolute world-axis lines in both headless captures')
    parser.add_argument('--timeout', type=float, default=300, help='per-capture timeout in seconds')
    parser.add_argument('--exact', action='store_true', help='also fail when the two builds have different pixel hashes')
    args = parser.parse_args()
    try:
        return run(args)
    except (OSError, ValueError, re.error, subprocess.SubprocessError) as error:
        parser.exit(2, f'capture comparison: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
