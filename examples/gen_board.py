#!/usr/bin/env python3
"""Generate a synthetic stress-test DSN: an NxN grid of 2-pad components with
deliberately crossing nets (each component connects to one a few rows away)."""

import random
import sys

N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
OUT = sys.argv[2] if len(sys.argv) > 2 else f"stress{N}x{N}.dsn"
random.seed(42)

PITCH = 6000       # component spacing, um
MARGIN = 5000
size = MARGIN * 2 + (N - 1) * PITCH

comps = []
for r in range(N):
    for c in range(N):
        ref = f"U{r*N+c+1}"
        x = MARGIN + c * PITCH
        y = -(MARGIN + r * PITCH)
        rot = random.choice([0, 90, 180, 270])
        comps.append((ref, x, y, rot))

# Nets: "cross" pairs each component with one half a board away (pathological,
# maximum central congestion); "local" connects to a nearby component
# (realistic placement locality).
MODE = sys.argv[3] if len(sys.argv) > 3 else "cross"
nets = []
n = len(comps)
used_targets = set()
for i, (ref, *_rest) in enumerate(comps):
    if MODE == "local":
        r, c = i // N, i % N
        j = None
        for _ in range(50):
            dr, dc = random.randint(-2, 2), random.randint(-2, 2)
            nr, nc = r + dr, c + dc
            cand = nr * N + nc
            if (dr, dc) != (0, 0) and 0 <= nr < N and 0 <= nc < N and cand not in used_targets:
                j = cand
                break
        if j is None:
            continue
    else:
        j = (i + n // 2 + (i % 3)) % n
        if j in used_targets:
            continue
    if i == j:
        continue
    used_targets.add(j)
    nets.append((f"N{i+1}", f"{ref}-2", f"{comps[j][0]}-1"))

places = "\n".join(
    f'      (place {ref} {x} {y} front {rot} (PN X))' for ref, x, y, rot in comps
)
netdefs = "\n".join(
    f'    (net "{name}" (pins {a} {b}))' for name, a, b in nets
)

print(f"""(pcb "{OUT}"
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "KiCad's Pcbnew")
  )
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary
      (path pcb 0 0 0 {size} 0 {size} -{size} 0 -{size} 0 0)
    )
    (via "Via[0-1]_800:400_um")
    (rule (width 250) (clearance 200.1))
  )
  (placement
    (component "R_0805"
{places}
    )
  )
  (library
    (image "R_0805"
      (pin RoundRect[T]Pad_1025x1400_um 1 -937.5 0)
      (pin RoundRect[T]Pad_1025x1400_um 2 937.5 0)
    )
    (padstack RoundRect[T]Pad_1025x1400_um
      (shape (rect F.Cu -512.5 -700 512.5 700))
      (attach off)
    )
    (padstack "Via[0-1]_800:400_um"
      (shape (circle F.Cu 800 0 0))
      (shape (circle B.Cu 800 0 0))
      (attach off)
    )
  )
  (network
{netdefs}
  )
  (wiring)
)""", file=open(OUT, "w"))
print(f"wrote {OUT}: {len(comps)} components, {len(nets)} nets", file=sys.stderr)
