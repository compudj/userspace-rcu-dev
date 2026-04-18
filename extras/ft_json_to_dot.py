#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 EfficiOS Inc.
# SPDX-License-Identifier: MIT
"""
Render a Fractal Trie JSON snapshot (as produced by
cds_ft_show(ft, out, CDS_FT_SHOW_JSON)) as a GraphViz DOT graph.

Layout:
  - Each observed trie level becomes a `rank=same` subgraph cluster
    labeled "level N".  An invisible anchor per level, chained with
    invisible edges top-to-bottom, enforces strict row ordering.
  - Nodes are coloured by kind (internal=yellow, compressed=green,
    collapsed=purple, external=blue) with distinct shapes.
  - Solid black arrows show the precise (exact/inequality) lookup
    path: `parent -key_byte-> compressed -path NB-> child`, exactly
    as a writer or an exact reader would traverse.
  - Dashed blue arrows, drawn with constraint=false so they float
    over the layout without distorting rank assignment, show the
    skip-compressed (candidate) read fast path: the compressed
    node's parent jumps directly to cn->child, bypassing the
    compressed node body.

Both POVs are drawn simultaneously — mentally follow only solid for
the precise structure, or only dashed where available for the skip
fast path.

Input JSON schema (as emitted by cds_ft_show):
  Root:       { "ft": "0xPTR", "root": <node> }
  Internal:   { "ptr", "kind", "level", "nr_child", "density",
                "external_nodes"?, "children": [
                    {"key_byte": N, "child": <node>}, ... ] }
  Compressed: { "ptr", "kind":"COMPRESSED", "level", "path_len",
                "nr_keys", "density", "key_bytes": [b,...],
                "external_nodes"?, "child": <node> }
  Collapsed:  { "ptr", "kind":"COLLAPSED", "level",
                "scan_zone_size", "nr_entries", "external_nodes"?,
                "entries": [{"suffix":[b,...], "child": <node>}, ...] }
  External:   { "ptr", "kind":"EXTERNAL", "level" }

Usage:
    # Dump JSON from a test or long-running program that calls
    # cds_ft_show(ft, fp, CDS_FT_SHOW_JSON); then render:
    ft_json_to_dot.py trie.json -o trie.svg
    ft_json_to_dot.py trie.json -o trie.png
    ft_json_to_dot.py trie.json                  # DOT to stdout
    ft_json_to_dot.py trie.json --format svg     # SVG to stdout
"""
import argparse
import io
import json
import os
import shutil
import subprocess
import sys


KIND_SHAPE = {
    'EXTERNAL':   'ellipse',
    'COMPRESSED': 'diamond',
    'COLLAPSED':  'hexagon',
}


def kind_color(kind):
    if kind == 'EXTERNAL':
        return '#d0e8ff'                      # light blue
    if kind == 'COMPRESSED':
        return '#c8f0c8'                      # light green
    if kind == 'COLLAPSED':
        return '#e8d0ff'                      # light purple
    if kind.startswith(('LINEAR', 'POOL', 'PIGEON')):
        return '#fff0c0'                      # light yellow (internal)
    return '#eeeeee'                          # unknown


def node_id(ptr):
    return 'n' + ptr.replace('0x', '').lower()


def walk(n, parent=None, key_byte=None, by_level=None, edges=None, skips=None):
    """Populate by_level/edges/skips from a JSON node tree.

    edges: (parent_id, child_id, value, kind)
      kind == 'key'  : solid black, label = ordinal key byte
      kind == 'path' : solid black, label = "path NB" (compressed descent)

    skips: (source_id, child_id, path_len)
      Dashed blue, drawn with constraint=false (skip-compressed POV).
    """
    if n is None:
        return
    my_id = node_id(n['ptr'])
    by_level.setdefault(n['level'], []).append(n)
    if parent is not None and key_byte is not None:
        edges.append((parent, my_id, key_byte, 'key'))

    kind = n['kind']
    if kind == 'COMPRESSED':
        child = n.get('child')
        if child is not None:
            child_id = node_id(child['ptr'])
            # Precise POV: compressed -> child (solid, path label).
            edges.append((my_id, child_id, n['path_len'], 'path'))
            # Candidate POV: compressed's parent -> child (dashed skip).
            if parent is not None:
                skips.append((parent, child_id, n['path_len']))
            walk(child, by_level=by_level, edges=edges, skips=skips)
    elif kind == 'COLLAPSED':
        for e in n.get('entries', []):
            c = e['child']
            if c is None:
                continue
            edges.append((my_id, node_id(c['ptr']),
                          len(e['suffix']), 'path'))
            walk(c, by_level=by_level, edges=edges, skips=skips)
    elif kind != 'EXTERNAL':
        # Internal node.
        for c in n.get('children', []):
            walk(c['child'], parent=my_id, key_byte=c['key_byte'],
                 by_level=by_level, edges=edges, skips=skips)


def emit_dot(doc, out):
    by_level, edges, skips = {}, [], []
    walk(doc['root'], by_level=by_level, edges=edges, skips=skips)

    w = out.write
    w(f'digraph FT {{\n')
    w(f'  label="FT {doc["ft"]}";\n')
    w('  rankdir=TB; newrank=true;\n')
    w('  node [shape=box, fontname="Monospace", fontsize=10, '
      'style="filled,rounded"];\n')
    w('  edge [fontname="Monospace", fontsize=9];\n')

    sorted_levels = sorted(by_level)
    for lvl in sorted_levels:
        w(f'  subgraph "cluster_lvl_{lvl}" {{\n')
        w(f'    label="level {lvl}"; style="dashed"; color="gray70"; '
          f'fontsize=11;\n')
        w('    { rank=same;\n')
        w(f'      "lvl_anchor_{lvl}" [shape=plaintext, label="", '
          f'width=0, height=0];\n')
        for n in by_level[lvl]:
            parts = [n['ptr'], n['kind'], f'lvl={lvl}']
            if 'nr_child' in n:
                parts.append(f'nc={n["nr_child"]}')
            if 'path_len' in n:
                parts.append(f'plen={n["path_len"]}')
            if 'key_bytes' in n:
                parts.append('k=' +
                             ','.join(f'{b:02x}' for b in n['key_bytes']))
            if 'nr_entries' in n:
                parts.append(f'ne={n["nr_entries"]}')
            label = '\\n'.join(parts)
            shape = KIND_SHAPE.get(n['kind'], 'box')
            color = kind_color(n['kind'])
            w(f'      "{node_id(n["ptr"])}" [label="{label}", '
              f'shape={shape}, fillcolor="{color}"];\n')
        w('    }\n  }\n')

    if len(sorted_levels) > 1:
        w('  edge [style=invis];\n  ')
        w(' -> '.join(f'"lvl_anchor_{l}"' for l in sorted_levels))
        w(';\n  edge [style=solid];\n')

    # Precise edges: key-byte and path-descent.
    for p, c, v, ek in edges:
        if ek == 'key':
            w(f'  "{p}" -> "{c}" [color=black, penwidth=1.6, '
              f'arrowsize=0.9, label="{v:#04x}"];\n')
        else:  # 'path'
            w(f'  "{p}" -> "{c}" [color=black, penwidth=1.4, '
              f'arrowsize=0.9, label="path {v}B", '
              f'fontcolor="#404040"];\n')

    # Candidate/skip-compressed edges: overlay without affecting rank.
    for p, c, pl in skips:
        w(f'  "{p}" -> "{c}" [style=dashed, color="#2060a0", '
          f'penwidth=1.2, arrowsize=0.9, label="skip {pl}B", '
          f'fontcolor="#2060a0", constraint=false];\n')

    w('}\n')


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('json_file', help='Path to a cds_ft_show JSON dump')
    p.add_argument('-o', '--output', default=None,
                   help='Output file.  Extension (.svg/.png/.pdf/.dot) '
                        'picks the format; default is DOT to stdout.')
    p.add_argument('--format', default=None,
                   help='Override format (svg, png, pdf, dot).  '
                        'Inferred from --output when not given.')
    args = p.parse_args()

    fmt = args.format
    if fmt is None and args.output is not None:
        fmt = os.path.splitext(args.output)[1].lstrip('.').lower() or 'dot'
    if fmt is None:
        fmt = 'dot'
    if fmt not in ('dot', 'svg', 'png', 'pdf'):
        sys.exit(f"error: unsupported format: {fmt!r}")
    if fmt != 'dot' and shutil.which('dot') is None:
        sys.exit("error: graphviz 'dot' not found; "
                 "install it or use --format dot")

    with open(args.json_file) as f:
        doc = json.load(f)

    buf = io.StringIO()
    emit_dot(doc, buf)
    dot_text = buf.getvalue()

    if fmt == 'dot':
        sink = open(args.output, 'w') if args.output else sys.stdout
        try:
            sink.write(dot_text)
        finally:
            if sink is not sys.stdout:
                sink.close()
        return

    cmd = ['dot', f'-T{fmt}']
    if args.output:
        cmd += ['-o', args.output]
    proc = subprocess.run(cmd, input=dot_text.encode(),
                          capture_output=True, check=False)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr.decode(errors='replace'))
        sys.exit(f"error: dot failed (exit {proc.returncode})")
    if not args.output:
        sys.stdout.buffer.write(proc.stdout)


if __name__ == '__main__':
    main()
