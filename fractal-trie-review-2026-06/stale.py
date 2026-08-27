import re, sys
# A dedupe (shared=1) is SUSPECT when the most recent REAL take (shared=0) of
# that word was by a DIFFERENT thread: a dedupe may only ride the op's own take.
last_take = {}      # lock -> vtid of most recent shared=0 note
susp = 0; ok = 0; ex = []
for line in open(sys.argv[1]):
    m = re.search(r'vtid = (\d+) \}, \{ lock = (0x[0-9A-F]+), member = (0x[0-9A-F]+), shared = (\d)', line)
    if not m: continue
    vtid, lock, member, sh = m.group(1), m.group(2), m.group(3), m.group(4)
    if sh == '0':
        last_take[lock] = vtid
    else:
        prev = last_take.get(lock)
        if prev is None:
            continue                    # no take seen in-window; unclassifiable
        if prev != vtid:
            susp += 1
            if len(ex) < 5:
                ex.append((line.split(']')[0][1:], vtid, lock, prev))
        else:
            ok += 1
print("dedupes rideable on OWN take :", ok)
print("dedupes after a FOREIGN take :", susp, "  <== stale-dedupe pattern")
print("suspect fraction             :", round(100.0*susp/max(ok+susp,1), 2), "%")
for e in ex: print("   ts=%s vtid=%s lock=%s lastRealTakeBy=%s" % e)
