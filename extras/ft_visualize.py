#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 EfficiOS Inc.
# SPDX-License-Identifier: MIT
"""
Reconstruct a GraphViz snapshot of a Fractal Trie from an LTTng-UST
ft_tp trace.

Replays the authoritative structural events (tree_edge_set,
compressed_publish, collapsed_publish, collapsed_entry) plus node
lifetime events (detach_node_enter, compressed_free, node_recompact)
to build a shadow model of the trie at an arbitrary point in trace
time, then emits a DOT graph in the same style as ft_json_to_dot.py.

Node identity:
  A node is keyed by (ptr, creation_ts).  creation_ts is the trace
  timestamp of the first event that references the pointer in a
  structural role.  Nodes are "killed" (their current lifetime
  ends) on detach_node_enter / compressed_free / node_recompact of
  the old node.  The next event naming the same pointer starts a
  new lifetime.  This is necessary because the allocator reuses
  memory: the same ptr can denote several logically-distinct nodes
  over the trace's span.

Event consumption:
  - tree_edge_set(ft, parent, parent_kind, parent_level, key_byte,
                   child, child_kind, child_skip_len):
      authoritative structural edit.  Updates
      edges[(parent_life, key_byte)] = (child_life, parent_level)
      and seeds kind/level for both endpoints.  child == NULL
      clears the slot.
  - compressed_publish(cn, len, key_bytes, child, parent):
      establishes cn's path length, path bytes, and current child.
      Fires at creation (parent = NULL) and on every cn->child
      rewire via ft_publish_to_parent (with the real parent).
  - collapsed_publish(col, nr_entries, scan_zone_size): updates
      col's scan zone; the consumer uses nr_entries to prune dead
      entries if nr_entries shrinks.
  - collapsed_entry(col, entry_idx, suffix, child, dead):
      installs (dead=0) or tombstones (dead=1) a single entry.
  - Traversal events (slowpath_step, post_traversal, fastpath_enter,
      ineq_going_up_step) still update kind/level as a fallback
      when tree_edge_set hasn't covered a node yet (e.g. a subtree
      that predates the trace window).

Rendering:
  Per-level rank=same clusters with invisible anchors force strict
  top-to-bottom ordering.  Solid black edges show the precise
  lookup path (parent -0xKK-> compressed -path NB-> child).  Dashed
  blue edges (constraint=false) overlay the skip-compressed
  candidate fast path (parent-of-compressed -skip NB-> child).
  Node colour/shape by kind: internal=yellow box, compressed=green
  diamond, collapsed=purple hexagon, external=blue ellipse.

Usage:
    ft_visualize.py TRACE_DIR [--ft PTR] [--until TS] > trie.dot
    dot -Tsvg trie.dot -o trie.svg
    ft_visualize.py TRACE_DIR --ft 0x... -o trie.svg
    ft_visualize.py TRACE_DIR --ft 0x... -o trie.png
"""
import argparse
import io
import os
import shutil
import subprocess
import sys

try:
    import bt2
except ImportError:
    sys.exit("error: python3 bt2 bindings required (python3-babeltrace2)")


KIND_SHAPE = {
    'EXTERNAL':   'ellipse',
    'COMPRESSED': 'diamond',
    'COLLAPSED':  'hexagon',
}


def kind_color(kind):
    if kind == 'EXTERNAL':
        return '#d0e8ff'
    if kind == 'COMPRESSED':
        return '#c8f0c8'
    if kind == 'COLLAPSED':
        return '#e8d0ff'
    if kind.startswith(('LINEAR', 'POOL', 'PIGEON')):
        return '#fff0c0'
    return '#eeeeee'


def enum_label(field):
    """First symbolic label for an LTTng enum field, or numeric str."""
    try:
        labels = field.labels
    except AttributeError:
        return str(int(field))
    return next(iter(labels)) if labels else str(int(field))


def _parse_time(s):
    """Parse --begin/--end.  Accepts a bare seconds-since-epoch float
    or a `[YYYY-MM-DD] hh:mm:ss[.nnnnnnnnn]` string.  When the date
    is omitted, today's local date is assumed (babeltrace2's CLI
    does the same for the convert command's --begin/--end)."""
    try:
        return float(s)
    except ValueError:
        pass
    import datetime
    for fmt in ('%Y-%m-%d %H:%M:%S.%f', '%Y-%m-%d %H:%M:%S'):
        try:
            return datetime.datetime.strptime(s, fmt)
        except ValueError:
            continue
    today = datetime.date.today()
    # Handle hh:mm:ss with optional fractional part of up to 9 digits.
    mo = None
    import re as _re
    mo = _re.match(r'^(\d{1,2}):(\d{2}):(\d{2})(?:\.(\d{1,9}))?$', s)
    if mo:
        h, mi, sec = int(mo.group(1)), int(mo.group(2)), int(mo.group(3))
        frac = mo.group(4) or ''
        # Pad/truncate to microseconds (6 digits) for datetime.
        us = int((frac + '000000')[:6]) if frac else 0
        t = datetime.time(h, mi, sec, us)
        return datetime.datetime.combine(today, t)
    sys.exit(f"error: cannot parse time spec: {s!r}")


PUBLIC_ENTER = {
    'ft_tp:insert_enter', 'ft_tp:insert_unique_enter',
    'ft_tp:insert_replace_enter', 'ft_tp:lookup_key_enter',
    'ft_tp:graft_enter', 'ft_tp:graft_swap_enter',
    'ft_tp:detach_enter', 'ft_tp:remove_enter',
    'ft_tp:replace_enter', 'ft_tp:lookup_enter',
    'ft_tp:iter_create',
}


class Model:
    """Shadow model keyed by (ptr, creation_ts)."""

    def __init__(self):
        # Current live lifetime per ptr: ptr -> life_id
        # life_id is a unique integer; nodes dict is keyed by it.
        self.cur_life = {}
        self.next_life = 0
        # life_id -> {'ptr': int, 'created_ts': int, 'kind': str,
        #             'level': int|None, 'meta': dict}
        self.nodes = {}
        # (parent_life, key_byte) -> child_life.  parent/child
        # levels are derived from m.nodes at render time so they
        # stay consistent with subsequent moves or force-updates.
        self.edges = {}
        # life_id -> {'len': int, 'key_bytes': list[int]|None,
        #             'child_life': life_id|None}
        self.cnodes = {}
        # col_life -> {'nr_entries': int, 'scan_zone_size': int,
        #              'entries': {entry_idx: {'suffix': list, 'child_life': id, 'dead': bool}}}
        self.cols = {}

    def birth(self, ptr, ts, kind=None):
        if not ptr:
            return None
        lid = self.cur_life.get(ptr)
        if lid is None:
            lid = self.next_life
            self.next_life += 1
            self.cur_life[ptr] = lid
            self.nodes[lid] = {
                'ptr': ptr, 'created_ts': ts,
                'kind': kind or 'UNKNOWN', 'level': None,
                'meta': {},
            }
        elif kind and self.nodes[lid]['kind'] == 'UNKNOWN':
            self.nodes[lid]['kind'] = kind
        return lid

    def kill(self, ptr):
        """End the current lifetime of ptr (memory freed / detached)."""
        lid = self.cur_life.pop(ptr, None)
        if lid is None:
            return
        # Drop edges referencing this lifetime.
        for k in list(self.edges):
            p_life, _kb = k
            c_life = self.edges[k]
            if p_life == lid or c_life == lid:
                del self.edges[k]
        self.cnodes.pop(lid, None)
        self.cols.pop(lid, None)
        self.nodes.pop(lid, None)

    def set_kind(self, lid, kind):
        if lid is not None and kind != 'UNKNOWN':
            self.nodes[lid]['kind'] = kind

    def set_level(self, lid, level, force=False):
        """First-write-wins per lifetime unless force=True.

        A node's level is an invariant of its current lifetime (a
        node at level N stays at level N until it is freed and the
        memory is reused as a different node).  The first
        structural event that names it is authoritative; later
        events that observe it at a different level (e.g. a
        traversal reporting the post-skip level of a compressed
        node) must not overwrite the true structural level.
        """
        if lid is None or level is None or level < 0:
            return
        n = self.nodes.get(lid)
        if n is None:
            return
        if force or n.get('level') is None:
            n['level'] = level


def consume_trace(args, m):
    cpu_ft = {}

    def cpu(ev):
        try:
            return int(ev.packet.context_field['cpu_id'])
        except Exception:
            return None

    def ft_filter(ev, name, pf):
        """Return True to include this event given --ft filter."""
        if args.ft is None:
            return True
        if name in PUBLIC_ENTER:
            ok = int(pf['ft']) == args.ft
            if ok:
                c = cpu(ev)
                if c is not None:
                    cpu_ft[c] = args.ft
            return ok
        # tree_edge_set carries ft directly.
        if name == 'ft_tp:tree_edge_set':
            return int(pf['ft']) == args.ft
        # Structural events without explicit ft: attribute by CPU.
        attr = cpu_ft.get(cpu(ev))
        if attr is not None:
            return attr == args.ft
        return True  # unattributed; let it through

    it = bt2.TraceCollectionMessageIterator(args.trace_dir,
        begin=_parse_time(args.begin) if args.begin else None,
        end=_parse_time(args.end) if args.end else None)
    for msg in it:
        if not isinstance(msg, bt2._EventMessageConst):
            continue
        ts = msg.default_clock_snapshot.value
        ev = msg.event
        name = ev.name
        pf = ev.payload_field

        # Update cpu->ft mapping on any observed public-API event.
        if name in PUBLIC_ENTER:
            try:
                c = cpu(ev)
                if c is not None:
                    cpu_ft[c] = int(pf['ft'])
            except Exception:
                pass

        if not ft_filter(ev, name, pf):
            continue

        if name == 'ft_tp:tree_edge_set':
            parent = int(pf['parent'])
            key_byte = int(pf['key_byte'])
            child = int(pf['child'])
            parent_level = int(pf['parent_level'])
            parent_kind = enum_label(pf['parent_kind'])
            child_kind = enum_label(pf['child_kind'])
            if not parent:
                continue

            # Lifetime reset on level conflict for INTERNAL-kind
            # parents only.  Internal-node frees aren't traced in
            # all paths, so a pointer can legitimately be reused at
            # a different depth without a kill event in between;
            # the conflict reset closes the gap.
            #
            # Compressed and collapsed parents are intentionally
            # exempted: the library emits tree_edge_set with
            # parent_kind=COMPRESSED (or COLLAPSED) at different
            # parent_level values during a single node's lifetime
            # (e.g. when structural replaces happen against slots
            # inside a compressed node's cn->child scope — the
            # parent_level reported is a depth inside the compressed
            # path, not the compressed node's own trie-level).
            # Killing on that signal would wipe the compressed node
            # out of the graph prematurely.
            is_internal_parent = not (
                parent_kind in ('COMPRESSED', 'COLLAPSED'))
            existing = m.cur_life.get(parent)
            if existing is not None and is_internal_parent:
                prev_lvl = m.nodes[existing].get('level')
                if prev_lvl is not None and prev_lvl != parent_level:
                    m.kill(parent)
            p_life = m.birth(parent, ts, parent_kind)
            m.set_kind(p_life, parent_kind)
            if is_internal_parent:
                m.set_level(p_life, parent_level, force=True)
            else:
                # For compressed/collapsed parents, only seed the
                # level if unset; rely on other paths (derive_levels,
                # tree_edge_set where the parent is a CHILD) for
                # authoritative level attribution.
                m.set_level(p_life, parent_level)

            if child:
                existing_c = m.cur_life.get(child)
                if existing_c is not None:
                    prev_clvl = m.nodes[existing_c].get('level')
                    if prev_clvl is not None and prev_clvl != parent_level + 1:
                        m.kill(child)
                c_life = m.birth(child, ts, child_kind)
                m.set_kind(c_life, child_kind)
                m.set_level(c_life, parent_level + 1, force=True)
                # Slot replacement: if the same (parent, key_byte)
                # previously held a different child lifetime, that
                # child has been removed from the trie — end its
                # lifetime so its collected state doesn't accumulate
                # across generations.  (Compressed/collapsed nodes
                # have no explicit free event, so this is the only
                # way to bound their lifetime in the consumer.)
                prev = m.edges.get((p_life, key_byte))
                if prev is not None and prev != c_life:
                    prev_node = m.nodes.get(prev)
                    if prev_node:
                        m.kill(prev_node['ptr'])
                m.edges[(p_life, key_byte)] = c_life
            else:
                prev = m.edges.pop((p_life, key_byte), None)
                if prev is not None:
                    prev_node = m.nodes.get(prev)
                    if prev_node:
                        m.kill(prev_node['ptr'])

        elif name == 'ft_tp:compressed_publish':
            cn = int(pf['cn'])
            if not cn:
                continue
            lid = m.birth(cn, ts, 'COMPRESSED')
            info = m.cnodes.setdefault(lid, {
                'len': 0, 'key_bytes': None,
                'child_life': None, 'parent_life': None,
            })
            info['len'] = int(pf['len'])
            try:
                info['key_bytes'] = [int(b) for b in pf['key_bytes']]
            except (KeyError, TypeError):
                pass
            child = int(pf['child'])
            info['child_life'] = (
                m.birth(child, ts) if child else None
            )
            # parent is NULL at creation time and non-NULL on
            # re-emissions from ft_publish_to_parent.  Record it
            # so derive_levels can bridge cn's level from its
            # parent when tree_edge_set never attached cn directly
            # (fresh compressed-node installs bypass
            # ft_node_set_nth).
            parent = int(pf['parent'])
            if parent:
                info['parent_life'] = m.birth(parent, ts)

        elif name in ('ft_tp:compressed_free', 'ft_tp:collapsed_free'):
            m.kill(int(pf['node']))

        elif name == 'ft_tp:collapsed_publish':
            col = int(pf['col'])
            if not col:
                continue
            lid = m.birth(col, ts, 'COLLAPSED')
            info = m.cols.setdefault(lid, {
                'nr_entries': 0, 'scan_zone_size': 0, 'entries': {},
            })
            info['nr_entries'] = int(pf['nr_entries'])
            info['scan_zone_size'] = int(pf['scan_zone_size'])

        elif name == 'ft_tp:collapsed_entry':
            col = int(pf['col'])
            if not col:
                continue
            lid = m.birth(col, ts, 'COLLAPSED')
            info = m.cols.setdefault(lid, {
                'nr_entries': 0, 'scan_zone_size': 0, 'entries': {},
            })
            idx = int(pf['entry_idx'])
            dead = int(pf['dead']) != 0
            if dead:
                info['entries'].pop(idx, None)
            else:
                child = int(pf['child'])
                child_life = m.birth(child, ts) if child else None
                info['entries'][idx] = {
                    'suffix': [int(b) for b in pf['suffix']],
                    'child_life': child_life,
                    'dead': False,
                }

        elif name == 'ft_tp:node_recompact':
            # Recompact allocates a new node at a different address
            # and frees the old one; children are carried over
            # (the library doesn't re-fire tree_edge_set for each
            # carried-over child).  The old pointer's lifetime
            # ends here, so a later tree_edge_set on the same
            # pointer denotes memory reuse.  Before dropping the
            # old lifetime, rewire its outgoing edges to the new
            # lifetime so the subtree rooted at this node stays
            # connected across the recompaction.
            old = int(pf['old_node'])
            new = int(pf['new_node'])
            if old and new:
                old_life = m.cur_life.get(old)
                new_life = m.birth(new, ts)
                if old_life is not None and new_life is not None:
                    # Outgoing edges: move from old_life to new_life
                    # at the same key_byte.
                    for key in list(m.edges):
                        p_life, kb = key
                        if p_life == old_life:
                            m.edges[(new_life, kb)] = m.edges.pop(key)
                    # Incoming edges: the grandparent's slot now
                    # points to new_life.  Rewire so the subtree
                    # stays connected above.  Recompact updates the
                    # parent's slot via ft_publish_to_parent but
                    # bypasses ft_node_set_nth, so no tree_edge_set
                    # fires to re-establish this edge.
                    for key in list(m.edges):
                        if m.edges[key] == old_life:
                            m.edges[key] = new_life
                    # Promote compressed/collapsed metadata too.
                    if old_life in m.cnodes:
                        m.cnodes[new_life] = m.cnodes.pop(old_life)
                        # If the cn was someone's child_life, fix it.
                    if old_life in m.cols:
                        m.cols[new_life] = m.cols.pop(old_life)
                    # Fix any cnode child_life / parent_life and
                    # collapsed child_life references to old_life.
                    for info in m.cnodes.values():
                        if info.get('child_life') == old_life:
                            info['child_life'] = new_life
                        if info.get('parent_life') == old_life:
                            info['parent_life'] = new_life
                    for cinfo in m.cols.values():
                        for e in cinfo['entries'].values():
                            if e.get('child_life') == old_life:
                                e['child_life'] = new_life
                    # Copy level/kind if not already set on new.
                    old_n, new_n = m.nodes.get(old_life), m.nodes.get(new_life)
                    if old_n and new_n:
                        if new_n.get('level') is None:
                            new_n['level'] = old_n.get('level')
                        if new_n.get('kind') == 'UNKNOWN':
                            new_n['kind'] = old_n.get('kind', 'UNKNOWN')
                m.kill(old)
            elif old:
                m.kill(old)
            elif new:
                m.birth(new, ts)

        # detach_node_enter is deliberately NOT a kill: detach
        # unlinks a node from its current position but the node
        # itself may survive and be re-attached (graft) or may be
        # freed later through a different path.  The node's
        # lifetime ends only at the actual free (compressed_free
        # or node_recompact above).

        # Traversal events are intentionally NOT used to set level.
        # tree_edge_set (authoritative for structural edits) gives
        # each node its level exactly once per lifetime; traversal
        # events observed later can report a different level
        # (e.g. a compressed-node skip target one level later) and
        # would overwrite the correct value.  Traversal events are
        # used only to fill in `kind` for lifetimes that haven't
        # yet been touched by a structural event.
        elif name in ('ft_tp:slowpath_step', 'ft_tp:post_traversal',
                      'ft_tp:fastpath_enter'):
            ptr = int(pf['node_flag'])
            if not ptr:
                continue
            lid = m.birth(ptr, ts)
            m.set_kind(lid, enum_label(pf['node_kind']))

        elif name == 'ft_tp:ineq_going_up_step':
            ptr = int(pf['path_entry'])
            if not ptr:
                continue
            lid = m.birth(ptr, ts)
            m.set_kind(lid, enum_label(pf['path_entry_kind']))


def derive_levels(m):
    """Propagate levels between compressed/collapsed nodes and their
    children.  tree_edge_set sets direct-child levels; compressed
    and collapsed containers bridge multiple levels via their path
    length / suffix length, so we propagate both ways (forward from
    container to child, and backward from child to container)."""
    changed = True
    while changed:
        changed = False
        for lid, info in m.cnodes.items():
            node = m.nodes.get(lid)
            if node is None:
                continue
            ch = info.get('child_life')
            ch_node = m.nodes.get(ch) if ch is not None else None
            par = info.get('parent_life')
            par_node = m.nodes.get(par) if par is not None else None
            # Compressed level from its parent (via compressed_publish).
            if node['level'] is None and par_node and \
               par_node['level'] is not None:
                node['level'] = par_node['level'] + 1
                changed = True
            # Forward: cn known, child unknown.
            if node['level'] is not None and ch_node and \
               ch_node['level'] is None:
                ch_node['level'] = node['level'] + info['len']
                changed = True
            # Backward: child known, cn unknown.
            elif node['level'] is None and ch_node and \
                 ch_node['level'] is not None:
                node['level'] = ch_node['level'] - info['len']
                changed = True
        for lid, info in m.cols.items():
            node = m.nodes.get(lid)
            if node is None:
                continue
            for e in info['entries'].values():
                ch = e.get('child_life')
                ch_node = m.nodes.get(ch) if ch is not None else None
                if node['level'] is not None and ch_node and \
                   ch_node['level'] is None:
                    ch_node['level'] = node['level'] + len(e['suffix'])
                    changed = True
                elif node['level'] is None and ch_node and \
                     ch_node['level'] is not None:
                    node['level'] = ch_node['level'] - len(e['suffix'])
                    changed = True


def model_to_json(m, ft_ptr):
    """Serialize the shadow model as a JSON document mirroring the
    schema emitted by cds_ft_show(CDS_FT_SHOW_JSON).  This makes the
    trace-reconstructed snapshot directly comparable to an
    authoritative in-process dump (e.g. via `diff <(...) <(...)`).

    Finds the root by picking the level-0 lifetime that has the
    most reachable descendants (the real root of the filtered FT)
    and walks from there.  Lifetimes that are disconnected from the
    root are not included — this matches the authoritative dump
    which only walks from `ft->root`.
    """
    roots = [lid for lid, n in m.nodes.items() if n.get('level') == 0]
    if not roots:
        return {
            'ft': _fmt_ptr(ft_ptr) if ft_ptr else None,
            'root': None,
            'note': 'no level-0 node found in model',
        }

    # Pick the root with the most reachable descendants.
    def count_reachable(start):
        seen = set([start])
        q = [start]
        while q:
            cur = q.pop()
            for (p, _kb), c in m.edges.items():
                if p == cur and c not in seen:
                    seen.add(c); q.append(c)
            info = m.cnodes.get(cur)
            if info and info.get('child_life') not in seen:
                if info['child_life'] is not None:
                    seen.add(info['child_life']); q.append(info['child_life'])
            colinfo = m.cols.get(cur)
            if colinfo:
                for e in colinfo['entries'].values():
                    ch = e.get('child_life')
                    if ch is not None and ch not in seen:
                        seen.add(ch); q.append(ch)
        return len(seen)

    root_lid = max(roots, key=count_reachable)
    visited = set()

    def node_to_json(lid):
        if lid is None or lid in visited:
            return None
        visited.add(lid)
        n = m.nodes.get(lid)
        if n is None:
            return None
        kind = n['kind']
        out = {
            'ptr': _fmt_ptr(n['ptr']),
            'kind': kind,
            'level': n['level'],
        }
        if kind == 'EXTERNAL':
            return out
        if kind == 'COMPRESSED':
            info = m.cnodes.get(lid, {})
            out['path_len'] = info.get('len', 0)
            if info.get('key_bytes') is not None:
                out['key_bytes'] = info['key_bytes']
            child_life = info.get('child_life')
            out['child'] = node_to_json(child_life) if child_life else None
            return out
        if kind == 'COLLAPSED':
            info = m.cols.get(lid, {})
            out['nr_entries'] = info.get('nr_entries', 0)
            out['scan_zone_size'] = info.get('scan_zone_size', 0)
            out['entries'] = [
                {
                    'suffix': e['suffix'],
                    'child': node_to_json(e['child_life']),
                }
                for _idx, e in sorted(info.get('entries', {}).items())
                if not e.get('dead')
            ]
            return out
        # Internal node.
        children = []
        for (p, kb), c in sorted(m.edges.items()):
            if p != lid:
                continue
            children.append({
                'key_byte': kb,
                'child': node_to_json(c),
            })
        out['children'] = children
        return out

    return {
        'ft': _fmt_ptr(ft_ptr) if ft_ptr else None,
        'root': node_to_json(root_lid),
    }


def _fmt_ptr(p):
    return f'{p:#x}' if isinstance(p, int) else p


def emit_dot(m, out):
    w = out.write

    # Build the set of lifetimes actually referenced by surviving
    # edges (tree_edge_set slot assignments, compressed cn->child,
    # and collapsed entries).  Drop orphan lifetimes to avoid
    # flooding the graph with detached externals whose
    # freed/reused state cannot be observed from the trace alone.
    reachable = set()
    for (p_life, _kb), c_life in m.edges.items():
        reachable.add(p_life)
        reachable.add(c_life)
    for lid, info in m.cnodes.items():
        if info.get('child_life') is not None:
            reachable.add(lid)
            reachable.add(info['child_life'])
    for lid, info in m.cols.items():
        for e in info['entries'].values():
            if e.get('child_life') is not None:
                reachable.add(lid)
                reachable.add(e['child_life'])

    by_level = {}
    for lid in reachable:
        n = m.nodes.get(lid)
        if n is None:
            continue
        by_level.setdefault(n.get('level'), []).append(lid)

    w('digraph FT {\n  rankdir=TB; newrank=true;\n')
    w('  node [shape=box, fontname="Monospace", fontsize=10, '
      'style="filled,rounded"];\n')
    w('  edge [fontname="Monospace", fontsize=9];\n')

    sorted_levels = sorted(l for l in by_level if l is not None)
    for lvl in sorted_levels:
        w(f'  subgraph "cluster_lvl_{lvl}" {{\n')
        w(f'    label="level {lvl}"; style="dashed"; color="gray70"; '
          f'fontsize=11;\n')
        w('    { rank=same;\n')
        w(f'      "lvl_anchor_{lvl}" [shape=plaintext, label="", '
          f'width=0, height=0];\n')
        for lid in sorted(by_level[lvl], key=lambda x: m.nodes[x]['ptr']):
            _emit_node(m, lid, w)
        w('    }\n  }\n')

    if None in by_level:
        w('  subgraph "cluster_unknown" {\n')
        w('    label="level unknown"; style="dotted"; color="gray70";\n')
        for lid in sorted(by_level[None], key=lambda x: m.nodes[x]['ptr']):
            _emit_node(m, lid, w)
        w('  }\n')

    if len(sorted_levels) > 1:
        w('  edge [style=invis];\n  ')
        w(' -> '.join(f'"lvl_anchor_{l}"' for l in sorted_levels))
        w(';\n  edge [style=solid];\n')

    # Precise structural edges.
    for (p_life, kb), c_life in sorted(m.edges.items()):
        if p_life not in m.nodes or c_life not in m.nodes:
            continue
        w(f'  "n{p_life}" -> "n{c_life}" [color=black, penwidth=1.6, '
          f'arrowsize=0.9, label="{kb:#04x}"];\n')

    # Compressed: compressed -> child (precise descent, solid) plus
    # parent-of-compressed -> child (skip, dashed).
    for lid, info in m.cnodes.items():
        ch = info.get('child_life')
        if ch is None or ch not in m.nodes or lid not in m.nodes:
            continue
        # Precise.
        w(f'  "n{lid}" -> "n{ch}" [color=black, penwidth=1.4, '
          f'arrowsize=0.9, label="path {info["len"]}B", '
          f'fontcolor="#404040"];\n')
        # Candidate skip: find all edges where this cn is a child.
        for (p_life, _kb), c_life in m.edges.items():
            if c_life == lid:
                w(f'  "n{p_life}" -> "n{ch}" [style=dashed, color="#2060a0", '
                  f'penwidth=1.2, arrowsize=0.9, '
                  f'label="skip {info["len"]}B", fontcolor="#2060a0", '
                  f'constraint=false];\n')

    # Collapsed entries: collapsed -> child per live entry.
    for lid, info in m.cols.items():
        if lid not in m.nodes:
            continue
        for idx, e in sorted(info['entries'].items()):
            ch = e.get('child_life')
            if ch is None or ch not in m.nodes:
                continue
            w(f'  "n{lid}" -> "n{ch}" [color=black, penwidth=1.3, '
              f'arrowsize=0.9, label="[{idx}] +{len(e["suffix"])}B", '
              f'fontcolor="#404040"];\n')

    w('}\n')


def _emit_node(m, lid, w):
    n = m.nodes[lid]
    parts = [f'{n["ptr"]:#x}', n['kind']]
    lvl = n['level']
    if lvl is not None:
        parts.append(f'lvl={lvl}')
    cn = m.cnodes.get(lid)
    if cn:
        parts.append(f'plen={cn["len"]}')
        kb = cn.get('key_bytes')
        if kb:
            parts.append('k=' + ','.join(f'{b:02x}' for b in kb))
    col = m.cols.get(lid)
    if col:
        parts.append(f'ne={col["nr_entries"]}')
        if col['scan_zone_size']:
            parts.append(f'scan={col["scan_zone_size"]}')
    label = '\\n'.join(parts)
    shape = KIND_SHAPE.get(n['kind'], 'box')
    color = kind_color(n['kind'])
    w(f'      "n{lid}" [label="{label}", shape={shape}, '
      f'fillcolor="{color}"];\n')


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('trace_dir', help='Path to an LTTng CTF trace directory')
    p.add_argument('--ft', type=lambda x: int(x, 0), default=None,
                   help='Filter to a specific FT pointer')
    p.add_argument('--begin', default=None,
                   help='Replay start time.  Accepts seconds since epoch '
                        '(float) or "YYYY-MM-DD hh:mm:ss[.nnnnnnnnn]".  '
                        'Passed straight to babeltrace2.')
    p.add_argument('--end', default=None,
                   help='Replay end time.  Same format as --begin.  '
                        'Use this to take a snapshot of the trie at a '
                        'point before drain/destroy.')
    p.add_argument('-o', '--output', default=None,
                   help='Output file (extension picks format)')
    p.add_argument('--format', default=None,
                   help='Override format (svg, png, pdf, dot)')
    args = p.parse_args()

    fmt = args.format
    if fmt is None and args.output is not None:
        fmt = os.path.splitext(args.output)[1].lstrip('.').lower() or 'dot'
    if fmt is None:
        fmt = 'dot'
    if fmt not in ('dot', 'svg', 'png', 'pdf', 'json'):
        sys.exit(f"error: unsupported format: {fmt!r}")
    if fmt in ('svg', 'png', 'pdf') and shutil.which('dot') is None:
        sys.exit("error: graphviz 'dot' not found")

    m = Model()
    consume_trace(args, m)
    derive_levels(m)

    if fmt == 'json':
        import json as _json
        doc = model_to_json(m, args.ft)
        sink = open(args.output, 'w') if args.output else sys.stdout
        try:
            _json.dump(doc, sink, indent=2)
            sink.write('\n')
        finally:
            if sink is not sys.stdout:
                sink.close()
        return

    buf = io.StringIO()
    emit_dot(m, buf)
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
