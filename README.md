# AutoRoute

[![CI](https://github.com/pingpongshow/AutoRoute/actions/workflows/ci.yml/badge.svg)](https://github.com/pingpongshow/AutoRoute/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

A fast PCB autorouter in Rust, with a live-visualization GUI and KiCad
integration via the Specctra DSN/SES interchange format (the same workflow as
freerouting).

- **Netclass-aware**: per-net track width, clearance and via padstack, with
  KiCad's max-of-pair clearance semantics and `.kicad_dru` netclass-pair rules.
- **Negotiated congestion** (PathFinder-style rip-up and reroute) over a
  multi-layer grid, with separate wire and via congestion fields.
- **Exact DRC** built in: routing is DRC-correct by construction, and `--check`
  verifies the emitted geometry independently — including sessions produced by
  other routers.
- **Plays well with hand routing**: existing tracks are kept as obstacles, so
  you can route the critical nets yourself and autoroute the rest.
- **Copper pours, differential pairs, any-angle refinement**, post-route
  optimization and a live GUI that shows the negotiation as it happens.

## Contents

- [Quick start](#quick-start)
- [Using with KiCad](#using-with-kicad)
- [Command-line reference](#command-line-reference)
- [Features in detail](#features-in-detail)
- [GUI](#gui)
- [How it works](#how-it-works)
- [Results](#results)
- [Status and limits](#status-and-limits)
- [Development](#development)

## Quick start

Requirements: a stable Rust toolchain ([rustup](https://rustup.rs)). On Linux
the GUI also needs GTK and xkbcommon headers
(`sudo apt-get install libgtk-3-dev libxkbcommon-dev`). KiCad is only needed
for routing `.kicad_pcb` files directly.

```bash
git clone https://github.com/pingpongshow/AutoRoute.git
cd AutoRoute
cargo build --release
```

Binaries land in `target/release/autoroute` and `target/release/autoroute-gui`.
Try it on a bundled example:

```bash
./target/release/autoroute examples/local12x12.dsn --check
```

```
routed in 0.26s: 7 iterations, 64 vias, 0 failed nets, 0 conflicted nets
DRC: clean (5 tolerance um)
wrote examples/local12x12.ses
```

## Using with KiCad

**Directly on a board file** (requires KiCad installed — its bundled Python
does the format conversion, so board interpretation always matches KiCad):

```bash
autoroute board.kicad_pcb
```

This exports `board.autoroute.dsn`, routes it, writes `board.ses`, and imports
the session into a new `board_routed.kicad_pcb`. The original file is never
modified. Set `KICAD_PYTHON` if KiCad's python3 isn't auto-detected (the macOS
app bundle and the system python3 are tried).

**Manually through DSN/SES:**

1. In pcbnew: **File > Export > Specctra DSN**
2. `autoroute board.dsn` (writes `board.ses`)
3. In pcbnew: **File > Import > Specctra Session**

**As a pcbnew plugin:** copy `kicad-plugin/autoroute_plugin.py` into your KiCad
`scripting/plugins` directory (e.g. `~/Documents/KiCad/9.0/scripting/plugins/`
on macOS) for an **AutoRoute board** toolbar button with an options dialog.
The result is imported into the live board and can be undone. Put `autoroute`
on your `PATH` or set `AUTOROUTE_BIN`.

After importing, refill zones in KiCad (**B**) — AutoRoute does not refill
pour shapes.

## Command-line reference

```
autoroute <board.dsn | board.kicad_pcb> [OPTIONS]
autoroute <board.dsn> --render-ses <session.ses> [--svg out.svg] [--check]
```

| Option | Default | Description |
|---|---|---|
| `-o, --output FILE` | input `.ses` | Output session file |
| `--check` | off | Run the exact geometric DRC on the result and report violations |
| `-v, --verbose` | off | Print per-iteration progress |
| `--grid UM` | from rules | Routing grid pitch in µm |
| `--iterations N` | 40 | Maximum rip-up/reroute iterations |
| `--via-cost UM` | 5000 | Via cost, in µm of equivalent track length |
| `--nets PATTERNS` | all | Route only matching nets, e.g. `'ETH_*,/RX*'` |
| `--skip-classes A,B` | none | Leave these net classes unrouted; their pads remain obstacles |
| `--clearance-cap UM` | none | Cap every net-class clearance at this value |
| `--reroute-wired` | off | Ignore existing routing and pours; route everything from scratch |
| `--dru FILE` | auto | Apply a `.kicad_dru` (auto-detected next to a `.kicad_pcb`) |
| `--layer-dirs STR` | inner h/v | Preferred direction per layer: `h`, `v` or `a`(ny), e.g. `ahva` |
| `--optimize N` | 1 | Post-route cleanup passes (straighter tracks, fewer vias) |
| `--no-refine` | off | Skip any-angle taut-polyline refinement |
| `--refine-45` | off | Restrict refinement to 0/45/90° |
| `--no-vias-in-pads` | off | Never place a via inside a pad, even on its own net |
| `--neck-floor F` | 0 | Minimum pad-entry neck width as a fraction of class width (1.0 = never neck) |
| `--render-ses SES` | — | Render an existing session to SVG instead of routing |
| `--svg FILE` | SES `.svg` | SVG output path for `--render-ses` |

**Exit codes:** `0` = complete; `2` = incomplete (failed or conflicted nets —
a partial SES is still written, and the remaining nets are reported).

## Features in detail

### Net classes

Classes from the DSN (`(class ...)` blocks) are applied per net: track width,
clearance and via padstack. Pair clearance is the max of the two nets'
clearances (KiCad semantics). Internally each distinct rule combination is a
"bucket" with its own obstacle and congestion fields, so a 2 mm power track and
a 0.25 mm signal track each see exactly the keep-outs their own width implies.
SES output carries per-net widths and via padstacks, so KiCad imports power
nets at their real width.

Large class clearances can make signal escapes next to power pads impossible;
`--clearance-cap` or `--skip-classes` (for classes meant to be pours rather
than tracks) are the practical remedies.

### Design rules (`.kicad_dru`)

Unconditional minimums (clearance, track width) and conditional netclass-pair
clearance rules (`A.NetClass == 'X' && B.NetClass == 'Y'`) are applied
automatically when a `.kicad_dru` sits next to the input board, or with
`--dru FILE`. Other rule shapes are reported and ignored.

### Existing routing is preserved

Nets that already have tracks or vias are left untouched and their copper
becomes an obstacle, so you can hand-route critical nets in KiCad and autoroute
the rest around them. This also protects against KiCad's session import
deleting a hand-routed net's tracks. `--reroute-wired` routes everything from
scratch.

### Copper pours and keepouts

Zones (exported by KiCad as Specctra planes) become routable pour regions. Pads
not already inside their pour are stitched to it with stubs and vias, and
pours never block other nets (they refill around foreign copper). Footprint and
board keepouts are hard obstacles.

### Vias

A via may go anywhere its barrel is legal (clearance + via radius to foreign
copper on every layer), including under IC bodies and in fanout moats, while
wire clearances are enforced only on the layers the wire actually runs.
`--no-vias-in-pads` keeps barrels out of pads entirely.

### Pad-entry necking

Tracks are narrowed where they enter a pad smaller than their width so they do
not overhang into neighbouring pins. `--neck-floor` limits how far — at `1.0`
every track keeps its full class width, which matters for current-carrying
rails.

### Differential pairs

Nets named with `+`/`-` or `_P`/`_N` suffixes are detected automatically:
partners route back-to-back and the second is attracted to the first's path,
producing loosely coupled pairs.

### Optimization and refinement

After negotiation settles, each net is rerouted alone against the final board
and the new route kept only when it is strictly better (fewer vias, then
shorter). Then every polyline is pulled taut: staircase grid runs become
straight any-angle segments wherever the exact geometry clears all other
copper. With per-layer preferred directions the output approaches hand-routed
appearance.

### Performance

The first routing iteration plans spatially disjoint nets in parallel; grid
rasterization, footprint stamping and the DRC are multi-threaded (rayon).
Searches run in a local window first and escalate to the full grid only on
failure, and stuck conflict clusters are retried in alternative orders.

## GUI

```bash
autoroute-gui [board.dsn | board.kicad_pcb]
```

Open a board, press **Route**, and watch the router negotiate in real time
(conflicted nets flash magenta). The GUI has layer visibility toggles,
pan (drag) and zoom (scroll/pinch), net glob highlighting, a ratsnest overlay
for unrouted nets, a DRC panel (click a violation to jump to it), SES export
and routed-`.kicad_pcb` export.

## How it works

- **Grid.** A uniform multi-layer routing grid with pitch derived from
  `width + clearance` (override with `--grid`). Pads and the board outline are
  rasterized as hard obstacles per net; a net's own pad interiors are its
  terminals.
- **Search.** Per net, a multi-source multi-target A* (8 directions plus
  through-via moves) builds an incremental Steiner tree over the net's pads.
- **Negotiation.** Nets may overlap at first. Each overlap is priced by a
  rising "present" cost plus a decaying history cost, and conflicted nets are
  ripped up and rerouted until the design is conflict-free. Two congestion
  fields are kept — one for entering wires, one for entering vias — so pricing
  and conflict detection match the exact DRC thresholds for wire-wire,
  wire-via and via-via pairs. Negotiation stops early once the conflict set
  plateaus.
- **Aura.** A soft margin one cell wider than the hard footprint makes routes
  keep spacing where room allows, which stabilizes convergence.
- **Output.** Grid runs are merged into polylines, optimized and refined, then
  written as a session file KiCad imports directly.

### Correctness model

Hard clearances are enforced at exact DRC distances. Grid quantization is
handled by "no-diagonal" zones around pads, the board edge and committed vias
(a 45° segment can pass closer to copper than its endpoint cells), rather than
by padding margins, which would eat the tens of µm of slack that fine-pitch
fanout depends on. A pad whose only cells fall inside a foreign pad's
clearance fails honestly instead of being forced through. Via barrels have
their own static obstacle field (`clearance + via radius` to foreign pads).

A conflict-free result is DRC-correct by construction (via-pair stamp radii
include the grid quantization error), and `--check` verifies the emitted
geometry independently.

## Results

Bundled examples on an Apple Silicon laptop (`--check`, default options):

| Board | Time | Iterations | Vias | Result |
|---|---|---|---|---|
| `demo` | 0.02 s | 7 | 2 | complete, DRC clean |
| `local12x12` | 0.26 s | 7 | 64 | complete, DRC clean |
| `stress6x6` | 0.33 s | 7 | 46 | complete, DRC clean |
| `stress8x8` | 1.2 s | 10 | 102 | complete, DRC clean |
| `stress12x12` | 15 s | 34 | 306 | incomplete (torture test) |

The `stress` boards wire every component to one half a board away — a
worst-case congestion test. `stress12x12` is deliberately beyond what the grid
can complete and is kept as a benchmark.

On a real six-layer design (504 footprints, 348 routable nets) the router
finishes with 0 conflicts, path length at 1.00× the MST lower bound, and a
handful of residual clearance violations flagged by `--check`.

## Status and limits

- Through vias only (no blind or buried vias).
- Differential pairs are attraction-coupled, not gap-controlled; no length
  matching yet.
- Pour shapes are not refilled or DRC-checked by this tool — refill zones in
  KiCad after import.
- Only unconditional and netclass-pair `.kicad_dru` rules are applied; other
  conditional rules are reported and ignored.
- Extremely congested boards can end incomplete. Remaining conflicted nets are
  reported and highlighted, never silently emitted.

## Development

```
crates/core    router: board model, grid, negotiated-congestion A*, DRC
crates/dsn     Specctra DSN parser and SES session writer
crates/cli     `autoroute` command-line tool
crates/gui     `autoroute-gui` egui app with live routing display
kicad-plugin   pcbnew action plugin (export DSN -> route -> import SES)
examples       sample boards and a synthetic board generator
```

```bash
cargo test --workspace --release
```

CI builds the workspace, runs the tests, and routes the `demo`, `local12x12`,
`stress6x6` and `stress8x8` examples, failing unless each completes with a
clean DRC.

Generate synthetic boards with
`examples/gen_board.py N out.dsn [cross|local]`: `cross` is the worst-case
congestion pattern, `local` mimics realistic placement locality.

## License

[MIT](LICENSE)
