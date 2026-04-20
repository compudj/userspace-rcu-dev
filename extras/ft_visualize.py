#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 EfficiOS Inc.
# SPDX-License-Identifier: MIT
"""
Reconstruct a GraphViz snapshot of a Fractal Trie from an LTTng-UST
cds_ft trace.

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

Trace session setup:
    Always enable the vpid context so pointers from different
    processes don't alias each other — the script needs it to
    scope (group, ft) tuples to a single process:

        lttng create <session>
        lttng enable-event --userspace 'cds_ft:*'
        lttng add-context --userspace --type vpid
        lttng start

Usage:
    # Discover (vpid, group, ft) triples + their lifetimes:
    ft_visualize.py TRACE_DIR --list

    # Reconstruct a specific trie (all four args required):
    ft_visualize.py TRACE_DIR --vpid N --group 0x.. --ft 0x.. \
        --begin HH:MM:SS.nnnnnnnnn --end HH:MM:SS.nnnnnnnnn -o trie.svg

    # Render without graphviz:
    ft_visualize.py TRACE_DIR --vpid N --group 0x.. --ft 0x.. \
        --format json > trie.json
"""
import argparse
import datetime
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


def _ts_str(ns):
    """Format a ns-since-epoch timestamp as local HH:MM:SS.nnnnnnnnn
    (the same format babeltrace2 prints at the start of each event)."""
    sec, n = divmod(int(ns), 1_000_000_000)
    t = datetime.datetime.fromtimestamp(sec)
    return f"{t.strftime('%H:%M:%S')}.{n:09d}"


def list_lifetimes(trace_dir):
    """Walk the trace once and print every (group, ft) lifetime as
    [group, ft, begin, end, duration].  Unmatched create events (ft
    still alive at trace end) are reported with end='(open)'.
    """
    import bt2
    # (ft_ptr, vpid, instance) -> dict(group, begin_ns, end_ns)
    ft_instances = []
    open_ft = {}  # (vpid, ft_ptr) -> index of current open entry
    # vpid -> list of violation timestamps (ns).  Violations are test
    # invariant failures; attributing them to a specific ft requires
    # matching by vpid and time range (violation payload carries no ft).
    violations = {}
    it = bt2.TraceCollectionMessageIterator(trace_dir)
    for msg in it:
        if not isinstance(msg, bt2._EventMessageConst):
            continue
        ev = msg.event
        name = ev.name
        pf = ev.payload_field
        try:
            ts = msg.default_clock_snapshot.ns_from_origin
        except Exception:
            ts = msg.default_clock_snapshot.value
        vpid = _event_vpid(ev)
        if name == 'cds_ft:ft_create':
            ft = int(pf['ft'])
            group = int(pf['ft_group'])
            ft_instances.append({'ft': ft, 'group': group, 'vpid': vpid,
                                 'begin': ts, 'end': None})
            open_ft[(vpid, ft)] = len(ft_instances) - 1
        elif name == 'cds_ft:ft_destroy':
            ft = int(pf['ft'])
            key = (vpid, ft)
            if key in open_ft:
                ft_instances[open_ft.pop(key)]['end'] = ts
        elif name == 'cds_ft:violation':
            violations.setdefault(vpid, []).append(ts)
    # Annotate each lifetime with violations that fired in the same
    # vpid during its interval.
    for e in ft_instances:
        vlist = violations.get(e['vpid'], [])
        end = e['end'] if e['end'] is not None else float('inf')
        e['violations'] = [v for v in vlist if e['begin'] <= v <= end]
    # Sort by vpid then group then begin
    ft_instances.sort(key=lambda e: (e['vpid'] or 0, e['group'], e['begin']))
    header = (f"{'vpid':<8} {'group':<20} {'ft':<20} {'begin':<20} "
              f"{'end':<20} {'duration':<12} violations")
    print(header)
    print("-" * len(header))
    for e in ft_instances:
        begin = _ts_str(e['begin'])
        end = _ts_str(e['end']) if e['end'] is not None else '(open)'
        if e['end'] is not None:
            dur = f"{(e['end'] - e['begin']) / 1e9:.6f}s"
        else:
            dur = '-'
        vpid_s = str(e['vpid']) if e['vpid'] is not None else '?'
        if e['violations']:
            # Absolute wall-clock timestamp for each violation, so
            # the user can grep the raw trace (babeltrace2 output
            # uses the same format) to pinpoint the failing event.
            vs = ",".join(_ts_str(v) for v in e['violations'])
        else:
            vs = '-'
        print(f"{vpid_s:<8} {hex(e['group']):<20} {hex(e['ft']):<20} "
              f"{begin:<20} {end:<20} {dur:<12} {vs}")
    if any(e['vpid'] is None for e in ft_instances):
        print()
        print("note: some events have no vpid.  For unambiguous")
        print("      disambiguation across processes, enable the vpid")
        print("      context at trace-session setup:")
        print("          lttng add-context --userspace --type vpid")


def _event_vpid(ev):
    """Extract vpid from an event's common context, if present."""
    try:
        return int(ev.common_context_field['vpid'])
    except Exception:
        pass
    try:
        return int(ev.context_field['vpid'])
    except Exception:
        pass
    return None


PUBLIC_ENTER = {
    'cds_ft:insert_enter', 'cds_ft:insert_unique_enter',
    'cds_ft:insert_replace_enter', 'cds_ft:lookup_key_enter',
    'cds_ft:graft_enter', 'cds_ft:graft_swap_enter',
    'cds_ft:detach_enter', 'cds_ft:remove_enter',
    'cds_ft:replace_enter', 'cds_ft:lookup_enter',
    'cds_ft:iter_create',
}


class Model:
    """Shadow model keyed by (ptr, creation_ts)."""

    def __init__(self):
        # Current live lifetime per ptr: ptr -> life_id
        # life_id is a unique integer; nodes dict is keyed by it.
        self.cur_life = {}
        self.next_life = 0
        # Most recent root_publish target (life_id) — authoritative
        # root pointer for the filtered FT.  Populated by
        # consume_trace() on every root_publish event.
        self.last_root_life = None
        # Per-ft last root_publish (ft_ptr -> root life_id).  When
        # the scope covers a whole cds_ft_group, multiple ft's
        # publish roots; this map lets model_to_json pick the right
        # one for a specific --ft target.
        self.last_root_per_ft = {}
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

        When force=True and the level actually changes, propagate
        the new level through descendants (edges, compressed
        child, collapsed entries).  graft / graft_swap and
        root_publish transplant an entire subtree to a new depth
        without emitting per-edge tree_edge_set events for the
        carried-over children, so the depth invariant must be
        re-derived on the receiving side.
        """
        if lid is None or level is None or level < 0:
            return
        n = self.nodes.get(lid)
        if n is None:
            return
        prev = n.get('level')
        if force or prev is None:
            n['level'] = level
            if force and prev != level:
                self._propagate_level(lid, set([lid]))

    def _propagate_level(self, lid, visited):
        """Push the current level of `lid` down to its descendants.

        Called after a force-update changes a lifetime's level.
        Respects skip-compressed interposition: an edge whose
        (parent_ptr, child_ptr) matches a cn's (parent_ptr,
        child_ptr) has that cn at parent_level+1 and the stored
        child at parent_level+1+cn.len.
        """
        n = self.nodes.get(lid)
        if n is None:
            return
        level = n.get('level')
        if level is None:
            return
        parent_ptr = n['ptr']
        for (p_life, _kb), c_life in list(self.edges.items()):
            if p_life != lid or c_life in visited:
                continue
            c_node = self.nodes.get(c_life)
            if c_node is None:
                continue
            interposed_cn_lid = None
            interposed_len = 0
            for cn_lid, cn_info in self.cnodes.items():
                if (cn_info.get('parent_ptr') == parent_ptr
                        and cn_info.get('child_ptr') == c_node['ptr']):
                    interposed_cn_lid = cn_lid
                    interposed_len = cn_info.get('len', 0)
                    break
            if interposed_cn_lid is not None:
                cn_n = self.nodes.get(interposed_cn_lid)
                if cn_n is not None and interposed_cn_lid not in visited:
                    cn_n['level'] = level + 1
                    visited.add(interposed_cn_lid)
                visited.add(c_life)
                c_node['level'] = level + 1 + interposed_len
                self._propagate_level(c_life, visited)
            else:
                visited.add(c_life)
                c_node['level'] = level + 1
                self._propagate_level(c_life, visited)
        cn_info = self.cnodes.get(lid)
        if cn_info is not None:
            child_life = cn_info.get('child_life')
            if child_life is not None and child_life not in visited:
                child_node = self.nodes.get(child_life)
                if child_node is not None:
                    visited.add(child_life)
                    child_node['level'] = level + cn_info.get('len', 0)
                    self._propagate_level(child_life, visited)
        col_info = self.cols.get(lid)
        if col_info is not None:
            for e in col_info.get('entries', {}).values():
                ch = e.get('child_life')
                if ch is None or ch in visited:
                    continue
                ch_node = self.nodes.get(ch)
                if ch_node is None:
                    continue
                visited.add(ch)
                ch_node['level'] = level + 1
                self._propagate_level(ch, visited)


def consume_trace(args, m):
    cpu_ft = {}
    # Filter events by the reconstruction scope.  Scope is the set of
    # ft pointers that belong to --group, because graft / graft_swap
    # can move subtrees across ft's within the group (events for those
    # nodes may be tagged with either ft).  The mapping ft -> group is
    # learned from cds_ft:ft_create events collected in a pre-scan.
    #
    # Both --group and --ft are required when filtering.  --group
    # scopes event acceptance; --ft identifies which ft's root_publish
    # to walk from in the reconstructed model.  They must be consistent
    # (ft must belong to the named group).  The --begin/--end options
    # disambiguate when an ft pointer is reused across generations.
    ft_group_set = set()
    ft_to_group = {}   # (vpid, ft) -> group
    if (args.ft is not None or args.group is not None
            or args.vpid is not None):
        if args.ft is None or args.group is None or args.vpid is None:
            sys.exit("error: --vpid, --group and --ft must be provided "
                     "together (or omit all three to disable filtering)")
        pre_it = bt2.TraceCollectionMessageIterator(args.trace_dir,
            begin=_parse_time(args.begin) if args.begin else None,
            end=_parse_time(args.end) if args.end else None)
        for pmsg in pre_it:
            if not isinstance(pmsg, bt2._EventMessageConst):
                continue
            pev = pmsg.event
            if pev.name != 'cds_ft:ft_create':
                continue
            pvpid = _event_vpid(pev)
            try:
                ft = int(pev.payload_field['ft'])
                group = int(pev.payload_field['ft_group'])
            except Exception:
                continue
            ft_to_group[(pvpid, ft)] = group
        # Validate consistency of --vpid, --ft and --group.
        seen_group = ft_to_group.get((args.vpid, args.ft))
        if seen_group is None:
            sys.exit(f"error: no cds_ft:ft_create for vpid={args.vpid} "
                     f"ft={args.ft:#x} in the --begin/--end window; "
                     "widen the range or run with --list to find the "
                     "right triple")
        if seen_group != args.group:
            sys.exit(f"error: ft {args.ft:#x} in vpid {args.vpid} "
                     f"belongs to group {seen_group:#x}, not --group "
                     f"{args.group:#x}")
        ft_group_set = {ft for (vpid, ft), g in ft_to_group.items()
                        if vpid == args.vpid and g == args.group}

    def cpu(ev):
        try:
            return int(ev.packet.context_field['cpu_id'])
        except Exception:
            return None

    def ft_filter(ev, name, pf):
        """Return True to include this event given the scope filter."""
        if not ft_group_set:
            return True
        # First: must be from the target vpid.  Events from other
        # processes can never describe nodes in our trie even if
        # the pointer happens to collide.
        if _event_vpid(ev) != args.vpid:
            return False
        if name in PUBLIC_ENTER:
            ok = int(pf['ft']) in ft_group_set
            if ok:
                c = cpu(ev)
                if c is not None:
                    cpu_ft[c] = int(pf['ft'])
            return ok
        # tree_edge_set and root_publish carry ft directly.
        if name in ('cds_ft:tree_edge_set', 'cds_ft:root_publish'):
            return int(pf['ft']) in ft_group_set
        # Structural events without explicit ft: attribute by CPU.
        attr = cpu_ft.get(cpu(ev))
        if attr is not None:
            return attr in ft_group_set
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

        if name == 'cds_ft:tree_edge_set':
            parent = int(pf['parent'])
            key_byte = int(pf['key_byte'])
            child = int(pf['child'])
            parent_level = int(pf['parent_level'])
            parent_kind = enum_label(pf['parent_kind'])
            child_kind = enum_label(pf['child_kind'])
            try:
                child_skip_len = int(pf['child_skip_len'])
            except Exception:
                child_skip_len = 0
            if not parent:
                continue
            # Skip-compressed pointer: the `child` raw value encodes
            # the compressed path length in its high bits; the real
            # child address sits in the low 57 bits (64-bit).  Mask
            # it so subsequent m.birth / m.edges use the true child
            # pointer (same lifetime as the compressed node reached
            # via other events).  Without this, a skip edge creates
            # a phantom lifetime for the skip-tagged value that's
            # never connected to anything.
            if child and child_skip_len:
                child = child & ((1 << 57) - 1)

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
                # Do NOT kill on child-level conflict: graft and
                # graft_swap move a subtree to a new depth, so the
                # same logical node can legitimately appear at a
                # different level.  Killing here would wipe the
                # edges that were emitted while the subtree still
                # lived in the source trie.  Internal-node pointer
                # reuse at a different level without a free event
                # is a theoretical risk but has no observed trigger.
                c_life = m.birth(child, ts, child_kind)
                m.set_kind(c_life, child_kind)
                # With skip-compressed, the child's real depth is
                # parent_level + 1 + skip_len: the slot at `key`
                # holds a skip pointer whose skip_len bytes are
                # absorbed by the virtual compressed node before
                # the real child's level.
                m.set_level(c_life,
                    parent_level + 1 + child_skip_len, force=True)
                # Slot replacement: rewire the edge to the new child.
                # We deliberately do NOT kill the previous child's
                # lifetime here — in graft / graft_swap, a slot
                # reassignment can mean the previous child moved to
                # another ft in the same cds_ft_group rather than
                # being freed.  Killing would wipe its still-valid
                # outgoing edges.  Real frees come through dedicated
                # events (compressed_free, collapsed_free, and
                # node_recompact of the old_node).
                m.edges[(p_life, key_byte)] = c_life
            else:
                m.edges.pop((p_life, key_byte), None)

            # Skip-compressed: when the slot at (parent, key)
            # carries a skip pointer (child_skip_len > 0), the
            # compressed node referenced by the skip is attached
            # to `parent` at `key_byte`.  The underlying child
            # pointer is the compressed's cn->child (after mask).
            # We don't know the compressed node's own pointer from
            # tree_edge_set alone, but any cn whose child_ptr and
            # path_len match this event's (raw_child, skip_len)
            # is the one logically at this slot.  Seed its
            # parent_ptr so model_to_json can interpose it without
            # waiting for the re-publish compressed_publish (which
            # fires only AFTER the auth snapshot in typical
            # insert-then-dump sequences).
            if child_skip_len:
                for cn_lid, cn_info in m.cnodes.items():
                    if (cn_info.get('len') == child_skip_len
                            and cn_info.get('child_ptr') == child):
                        cn_info['parent_ptr'] = parent
                        cn_info['parent_life'] = p_life
                        break

        elif name == 'cds_ft:set_parent':
            # Mirror ft_set_parent: the child's parent is recorded
            # in the child's metadata.  For skip-compressed mode
            # the event's `child` is a skip pointer whose high
            # bits encode the compressed path length and whose low
            # bits point at the compressed's cn->child (not at the
            # compressed itself).  The library's ft_set_parent
            # maps the skip pointer to the underlying compressed
            # (via ft_skip_to_compressed) and stores `parent` in
            # the cn's metadata.  We mirror that by reverse-
            # looking up the cn: any cn whose child_ptr equals
            # the masked pointer is the one whose parent is
            # being set.  This seeds cn's parent_ptr/parent_life
            # well before the late re-publish that would
            # otherwise carry the same info.
            child = int(pf['child'])
            parent = int(pf['parent'])
            if not child or not parent:
                continue
            # Only act when the child is either a known COMPRESSED
            # lifetime (non-skip case) OR a skip pointer (high bits
            # encode skip_len).  Plain internal/external children
            # carry their own parent back-pointer and don't need
            # cn-parent lookup.
            raw_child = child & ((1 << 57) - 1)
            has_skip = child != raw_child
            target_cn = None
            if has_skip:
                # Skip pointer: find cn whose child_ptr == raw_child.
                for cn_lid, cn_info in m.cnodes.items():
                    if cn_info.get('child_ptr') == raw_child:
                        target_cn = cn_lid
                        break
            else:
                # Non-skip: cn directly if it's a known COMPRESSED.
                c_life = m.cur_life.get(raw_child)
                if c_life is not None and m.nodes.get(
                        c_life, {}).get('kind') == 'COMPRESSED':
                    target_cn = c_life
            if target_cn is None:
                continue
            info = m.cnodes.get(target_cn)
            if info is None:
                continue
            p_life = m.cur_life.get(parent)
            if p_life is None:
                p_life = m.birth(parent, ts)
            info['parent_life'] = p_life
            info['parent_ptr'] = parent

        elif name == 'cds_ft:compressed_publish':
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
            old_child_life = info.get('child_life')
            if child:
                child_kind = enum_label(pf['child_kind'])
                cl = m.birth(child, ts, child_kind)
                m.set_kind(cl, child_kind)
                info['child_life'] = cl
                info['child_ptr'] = child
            else:
                info['child_life'] = None
                info['child_ptr'] = None
            # parent is NULL at creation time and non-NULL on
            # re-emissions from ft_publish_to_parent.  Record it
            # so derive_levels can bridge cn's level from its
            # parent when tree_edge_set never attached cn directly
            # (fresh compressed-node installs bypass
            # ft_node_set_nth).
            parent = int(pf['parent'])
            if parent:
                info['parent_life'] = m.birth(parent, ts)
                info['parent_ptr'] = parent
            # Skip-compressed mode: when this cn's child changes,
            # the grandparent's skip slot is updated in place to
            # point at the new child — no tree_edge_set fires.
            # Rewrite any m.edges entry at the grandparent (parent
            # of this cn) that still resolves to the old child
            # life, so the consumer's view of the tree stays in
            # sync with the new skip target.  Done AFTER the
            # parent_life update so the current compressed_publish
            # event's parent pointer is used for the rewrite.
            gp_life = info.get('parent_life')
            if (gp_life is not None and old_child_life is not None
                    and info.get('child_life') is not None
                    and old_child_life != info['child_life']):
                for key in list(m.edges):
                    if m.edges[key] != old_child_life:
                        continue
                    if key[0] != gp_life:
                        continue
                    m.edges[key] = info['child_life']
            # parent can legitimately be NULL (creation-time publish
            # from ft_publish_compressed before the cn is attached).
            # Don't overwrite a previously-known parent_ptr.
            elif 'parent_ptr' not in info:
                info['parent_ptr'] = None

        elif name in ('cds_ft:compressed_free', 'cds_ft:collapsed_free'):
            m.kill(int(pf['node']))

        elif name == 'cds_ft:root_publish':
            # ft->root was rewritten.  The value IS the root, so it
            # is at level 0 and is the authoritative root for JSON
            # rendering / root-finding.  Seed kind/level; override
            # the "picked by descendant count" heuristic by keeping
            # the most recent root_publish within the --ft filter.
            root = int(pf['root'])
            if not root:
                continue
            root_kind = enum_label(pf['root_kind'])
            rlid = m.birth(root, ts, root_kind)
            m.set_kind(rlid, root_kind)
            m.set_level(rlid, 0, force=True)
            m.last_root_life = rlid
            m.last_root_per_ft[int(pf['ft'])] = rlid

        elif name == 'cds_ft:collapsed_publish':
            col = int(pf['col'])
            if not col:
                continue
            lid = m.birth(col, ts, 'COLLAPSED')
            info = m.cols.setdefault(lid, {
                'nr_entries': 0, 'scan_zone_size': 0, 'entries': {},
            })
            info['nr_entries'] = int(pf['nr_entries'])
            info['scan_zone_size'] = int(pf['scan_zone_size'])

        elif name == 'cds_ft:collapsed_entry':
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
                if child:
                    child_kind = enum_label(pf['child_kind'])
                    child_life = m.birth(child, ts, child_kind)
                    m.set_kind(child_life, child_kind)
                else:
                    child_life = None
                info['entries'][idx] = {
                    'suffix': [int(b) for b in pf['suffix']],
                    'child_life': child_life,
                    'dead': False,
                }

        elif name == 'cds_ft:node_recompact':
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
                # `new` is a freshly allocated node.  If the
                # reconstructor still holds a prior lifetime at
                # that address, the library must have freed it
                # before allocating again — the trace does not
                # carry an explicit free for every such release.
                # Kill the stale lifetime so this recompact's
                # carried-over edges are not wiped later by a
                # parent-level-conflict check on the reborn node.
                existing_new = m.cur_life.get(new)
                if (existing_new is not None
                        and existing_new != old_life):
                    m.kill(new)
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
                            info['child_ptr'] = new
                        if info.get('parent_life') == old_life:
                            info['parent_life'] = new_life
                            info['parent_ptr'] = new
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
        elif name in ('cds_ft:slowpath_step', 'cds_ft:post_traversal',
                      'cds_ft:fastpath_enter'):
            ptr = int(pf['node_flag'])
            if not ptr:
                continue
            lid = m.birth(ptr, ts)
            m.set_kind(lid, enum_label(pf['node_kind']))

        elif name == 'cds_ft:ineq_going_up_step':
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
    # Prefer the last root_publish event's target for the requested
    # ft (authoritative).  Fall back to the global last root when
    # no --ft was specified or no per-ft root was seen.
    root_lid = None
    if ft_ptr is not None:
        root_lid = m.last_root_per_ft.get(ft_ptr)
    if root_lid is None or root_lid not in m.nodes:
        root_lid = m.last_root_life if m.last_root_life in m.nodes else None
    if root_lid is None:
        # Fallback: pick the level-0 lifetime with most descendants.
        roots = [lid for lid, n in m.nodes.items() if n.get('level') == 0]
        if not roots:
            return {
                'ft': _fmt_ptr(ft_ptr) if ft_ptr else None,
                'root': None,
                'note': 'no level-0 node found in model',
            }

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
        # Internal node.  When skip-compressed mode is active, an
        # m.edges entry (lid, kb) -> C may logically represent
        # "parent -> compressed_cn -> C", with the compressed node
        # never appearing as a tree_edge_set child (the slot holds
        # a skip pointer encoding the compressed path).  Detect
        # this by looking for a cn whose parent pointer matches
        # `lid`'s pointer and whose child pointer matches C's
        # pointer — compare by ptr, not life_id, because lifetimes
        # can be reborn after kills while the logical node (and
        # its cn->child binding) persists.
        parent_ptr = n['ptr']
        children = []
        for (p, kb), c in sorted(m.edges.items()):
            if p != lid:
                continue
            c_node = m.nodes.get(c)
            c_ptr = c_node['ptr'] if c_node else None
            interposed = None
            for cn_lid, cn_info in m.cnodes.items():
                if (cn_info.get('parent_ptr') == parent_ptr
                        and cn_info.get('child_ptr') == c_ptr):
                    interposed = cn_lid
                    break
            children.append({
                'key_byte': kb,
                'child': node_to_json(interposed) if interposed
                         is not None else node_to_json(c),
            })
        out['children'] = children
        return out

    return {
        'ft': _fmt_ptr(ft_ptr) if ft_ptr else None,
        'root': node_to_json(root_lid),
    }


def _fmt_ptr(p):
    return f'{p:#x}' if isinstance(p, int) else p


def _key_label(kb, ascii_mode):
    """Format a key byte for edge labels.  When ascii_mode is set
    and the byte is printable ASCII, append the character."""
    if ascii_mode and 0x20 <= kb <= 0x7E:
        # Escape characters that would break DOT string syntax.
        c = chr(kb)
        if c in ('"', '\\'):
            c = '\\' + c
        return f"{kb:#04x} '{c}'"
    return f"{kb:#04x}"


def _key_sequence_label(kbs, ascii_mode):
    """Format a sequence of key bytes for an edge label.  Hex
    values are grouped together on the first row; the ASCII
    spelling (when ascii_mode) goes on a second row below — one
    row per representation, not interleaved byte-by-byte.

    Non-printable bytes are rendered as '.' in the ASCII row,
    keeping byte positions aligned.  If no byte is printable, the
    ASCII row is omitted.
    """
    if not kbs:
        return ''
    hex_part = ' '.join(f'{b:#04x}' for b in kbs)
    if not ascii_mode:
        return hex_part
    if not any(0x20 <= b <= 0x7E for b in kbs):
        return hex_part
    chars = []
    for b in kbs:
        if 0x20 <= b <= 0x7E:
            c = chr(b)
            if c in ('"', '\\'):
                c = '\\' + c
            chars.append(c)
        else:
            chars.append('.')
    return f"{hex_part}\\n'{''.join(chars)}'"


def emit_dot(m, out, ascii_mode=False):
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
            _emit_node(m, lid, w, ascii_mode)
        w('    }\n  }\n')

    if None in by_level:
        w('  subgraph "cluster_unknown" {\n')
        w('    label="level unknown"; style="dotted"; color="gray70";\n')
        for lid in sorted(by_level[None], key=lambda x: m.nodes[x]['ptr']):
            _emit_node(m, lid, w, ascii_mode)
        w('  }\n')

    if len(sorted_levels) > 1:
        w('  edge [style=invis];\n  ')
        w(' -> '.join(f'"lvl_anchor_{l}"' for l in sorted_levels))
        w(';\n  edge [style=solid];\n')

    # Precise structural edges.  dir=both surfaces the back-pointer
    # that every internal child keeps to its parent (ft_meta->parent,
    # cn_meta->parent, col_meta->parent), so one line represents both
    # the forward slot pointer and the reverse back-pointer.
    #
    # Skip-mode edges are elided here and rendered further down as
    # dashed-blue skip overlays instead: in skip mode the parent
    # slot holds a skip pointer whose low bits point at cn->child,
    # so the m.edges entry (p_life, kb) -> cn_child is physically a
    # skip, not a direct structural edge.  Drawing it solid would
    # double-render every skip as a long, level-spanning black arrow.
    skip_edge_keys = set()
    for cn_lid, cn_info in m.cnodes.items():
        gp_ptr = cn_info.get('parent_ptr')
        ch_ptr = cn_info.get('child_ptr')
        if gp_ptr is None or ch_ptr is None:
            continue
        # This cn is interposed (skip mode) only when the parent's
        # slot does NOT directly point at the cn itself; scan below
        # uses cn_is_direct_child, mirror the same check here.
        if any(cl == cn_lid for cl in m.edges.values()):
            continue
        # Compare by pointer, not life_id: cn_info's parent_life
        # / child_life can refer to earlier lifetimes than the one
        # m.edges currently holds after a rebirth.  The ptr is
        # stable across rebirths and is what the DOT edge emits.
        for (p_life, kb), c_life in m.edges.items():
            p_node = m.nodes.get(p_life)
            c_node = m.nodes.get(c_life)
            if p_node is None or c_node is None:
                continue
            if p_node['ptr'] == gp_ptr and c_node['ptr'] == ch_ptr:
                skip_edge_keys.add((p_life, kb))
    for (p_life, kb), c_life in sorted(m.edges.items()):
        if p_life not in m.nodes or c_life not in m.nodes:
            continue
        if (p_life, kb) in skip_edge_keys:
            continue
        w(f'  "n{p_life}" -> "n{c_life}" [color=black, penwidth=1.6, '
          f'arrowsize=0.9, dir=both, '
          f'label="{_key_label(kb, ascii_mode)}"];\n')

    # Compressed: compressed -> child (precise descent, solid) plus
    # parent-of-compressed -> child (skip, dashed, skip-mode only).
    #
    # The dashed edge represents the actual skip-compressed fast
    # path: the parent's slot holds a skip pointer whose low bits
    # point at cn->child, bypassing cn.  In non-skip mode the
    # parent's slot holds cn itself and there is no skip fast path,
    # so the dashed edge must not render.  Detection: in non-skip
    # mode cn appears as a c_life in m.edges (tree_edge_set with
    # skip_len=0); in skip mode cn is never a c_life, and the
    # edge from parent lands on cn->child (interposed, skip_len>0).
    for lid, info in m.cnodes.items():
        ch = info.get('child_life')
        if ch is None or ch not in m.nodes or lid not in m.nodes:
            continue
        # Precise.  dir=both mirrors the cn->child / child->cn
        # (child's parent back-pointer points to cn in both modes).
        # Label carries the full absorbed path byte-by-byte so the
        # reader can trace the lookup without cross-referencing
        # the cn's internal k= field (which now omits key_bytes).
        key_bytes = info.get('key_bytes')
        if key_bytes:
            cn_edge_label = _key_sequence_label(key_bytes, ascii_mode)
        else:
            cn_edge_label = f'path {info["len"]}B'
        w(f'  "n{lid}" -> "n{ch}" [color=black, penwidth=1.4, '
          f'arrowsize=0.9, dir=both, label="{cn_edge_label}", '
          f'fontcolor="#404040"];\n')
        cn_is_direct_child = any(c_life == lid
                                 for c_life in m.edges.values())
        if cn_is_direct_child:
            continue
        # Skip mode: collect (parent_life, slot_kb) pairs — the
        # slot_kb is the key byte under which the parent selects
        # the skip pointer, i.e. the first byte consumed by the
        # walk that reaches cn->child.  m.edges is authoritative
        # for slot_kb; the grey back-arrow and the dashed skip
        # share the same set of (parent, slot_kb) pairs.
        skip_slots = set()
        for (p_life, kb), c_life in m.edges.items():
            if c_life == ch and p_life != lid and p_life in m.nodes:
                skip_slots.add((p_life, kb))
        # Fallback: if m.edges has no edge to cn->child from a
        # distinct parent (rare — happens when the edge got torn
        # down before the snapshot window), use cn_info's
        # parent_ptr / parent_life so the cn still renders with
        # a back-arrow.  No slot_kb available in this fallback.
        if not skip_slots:
            gp_ptr = info.get('parent_ptr')
            fallback_parent = None
            if gp_ptr is not None:
                cur = m.cur_life.get(gp_ptr)
                if cur is not None and cur in m.nodes and cur != lid:
                    fallback_parent = cur
            if fallback_parent is None:
                gp = info.get('parent_life')
                if gp is not None and gp in m.nodes and gp != lid:
                    fallback_parent = gp
            if fallback_parent is not None:
                skip_slots.add((fallback_parent, None))
        for p_life, slot_kb in skip_slots:
            if slot_kb is not None:
                skip_label = (f'{_key_label(slot_kb, ascii_mode)}'
                              f' skip {info["len"]}B')
            else:
                skip_label = f'skip {info["len"]}B'
            w(f'  "n{p_life}" -> "n{ch}" [style=dashed, color="#2060a0", '
              f'penwidth=1.2, arrowsize=0.9, '
              f'label="{skip_label}", fontcolor="#2060a0", '
              f'constraint=false];\n')
        # Back-pointer: in skip-compressed mode the parent slot
        # holds a skip pointer (parent -> cn_child), so there is
        # no forward parent -> cn pointer — the only parent link
        # the cn carries is cn_meta->parent (child -> parent).
        # Render it as a thin grey back-arrow with constraint=false
        # so the cn doesn't visually look orphaned and the layout
        # is not distorted.  In non-skip mode the parent's slot
        # points at cn directly, so the back-arrow would duplicate
        # the forward solid edge's information and is skipped.
        seen_back = set()
        for p_life, _slot_kb in skip_slots:
            if p_life in seen_back:
                continue
            seen_back.add(p_life)
            w(f'  "n{lid}" -> "n{p_life}" [color="#808080", '
              f'penwidth=0.8, arrowsize=0.7, style=solid, '
              f'label="parent", fontcolor="#808080", '
              f'constraint=false];\n')

    # Collapsed entries: collapsed -> child per live entry.  The
    # edge label pairs the entry index with the full absorbed
    # sub-key — every byte the walker consumes between the
    # collapsed node and the external leaf — so a reader can
    # trace the lookup path without cross-referencing the
    # authoritative cds_ft_show(JSON) dump.
    for lid, info in m.cols.items():
        if lid not in m.nodes:
            continue
        for idx, e in sorted(info['entries'].items()):
            ch = e.get('child_life')
            if ch is None or ch not in m.nodes:
                continue
            suffix = e.get('suffix') or []
            if suffix:
                label = (f'[{idx}] '
                         f'{_key_sequence_label(suffix, ascii_mode)}')
            else:
                label = f'[{idx}] +0B'
            w(f'  "n{lid}" -> "n{ch}" [color=black, penwidth=1.3, '
              f'arrowsize=0.9, dir=both, '
              f'label="{label}", '
              f'fontcolor="#404040"];\n')

    w('}\n')


def _emit_node(m, lid, w, ascii_mode=False):
    n = m.nodes[lid]
    parts = [f'{n["ptr"]:#x}', n['kind']]
    lvl = n['level']
    if lvl is not None:
        parts.append(f'lvl={lvl}')
    cn = m.cnodes.get(lid)
    if cn:
        parts.append(f'plen={cn["len"]}')
        # key_bytes intentionally omitted here — they're shown on
        # the cn -> child edge label, matching the way collapsed
        # entry suffixes are rendered on the col -> entry edge.
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
    p.add_argument('--vpid', type=int, default=None,
                   help='Process ID the target trie lives in.  Required '
                        'alongside --group and --ft.  Requires the trace '
                        'session to have been started with '
                        '"lttng add-context --userspace --type vpid".')
    p.add_argument('--group', type=lambda x: int(x, 0), default=None,
                   help='cds_ft_group pointer for the target trie.  '
                        'All FTs in the group are accepted during '
                        'reconstruction (graft/graft_swap is bounded '
                        'to a group, so the group is the natural scope).')
    p.add_argument('--ft', type=lambda x: int(x, 0), default=None,
                   help='FT pointer for the target trie.  Must belong '
                        'to --group and live in --vpid.  Used to pick '
                        'the root (from cds_ft:root_publish events on '
                        'this ft) for model_to_json output.')
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
    p.add_argument('--ascii', action='store_true',
                   help="On key-byte edge labels, show the printable "
                        "ASCII character in addition to the hex byte "
                        "(e.g. \"0x61 'a'\").  Useful for tries keyed "
                        "on strings or paths.")
    p.add_argument('--list', action='store_true',
                   help='List every (group, ft, begin, end) lifetime '
                        'observed in the trace and exit.  Use the values '
                        'as --group / --ft / --begin / --end to scope a '
                        'subsequent reconstruction.')
    args = p.parse_args()

    if args.list:
        list_lifetimes(args.trace_dir)
        return

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
    emit_dot(m, buf, ascii_mode=args.ascii)
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
