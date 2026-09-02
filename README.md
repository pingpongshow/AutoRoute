# AutoRoute

A fast PCB autorouter in Rust, with a live-visualization GUI and KiCad
integration via the Specctra DSN/SES interchange format (the same workflow as
freerouting).

```
crates/core   router: board model, grid, negotiated-congestion A* router, DRC
crates/dsn    Specctra DSN parser + SES session writer
crates/cli    `autoroute` command line tool
crates/gui    `autoroute-gui` egui app with live routing display
kicad-plugin  pcbnew action plugin (export DSN -> route -> import SES)
```

## Build

```
cargo build --release
```

Binaries land in `target/release/autoroute` and `target/release/autoroute-gui`.

## Use with KiCad

The direct way (requires KiCad installed; its bundled Python does the
format conversion, so board interpretation always matches KiCad exactly):

```
autoroute board.kicad_pcb
```

This exports `board.autoroute.dsn`, routes, writes `board.ses`, and imports
the session back into a new `board_routed.kicad_pcb` — the original file is
never modified. Set `KICAD_PYTHON` if KiCad's python3 isn't auto-detected
(macOS app bundle and system python3 are tried).

The manual way:

1. In pcbnew: **File > Export > Specctra DSN**
2. `autoroute board.dsn` (writes `board.ses`)
3. In pcbnew: **File > Import > Specctra Session**

Or install `kicad-plugin/autoroute_plugin.py` into your KiCad
`scripting/plugins` directory for a one-click toolbar button. The GUI also
opens `.kicad_pcb` files directly.

## Net classes

Classes from the DSN (`(class ...)` blocks) are applied per net: track width,
clearance, and via padstack. Pair clearance is the max of the two nets'
clearances (KiCad semantics). Internally each distinct rule combination is a
"bucket" with its own obstacle and congestion fields, so a 2 mm power track
and a 0.25 mm signal track each see exactly the keep-outs their own width
implies. SES output carries per-net widths and via padstacks, so KiCad
imports power nets at their real width.

### CLI

```
autoroute board.dsn|board.kicad_pcb
          [-o out.ses] [--grid UM] [--iterations N] [--via-cost UM]
          [--skip-classes A,B,...] [--check] [-v]
autoroute board.dsn --render-ses out.ses [--svg out.svg] [--check]
```

`--check` runs an exact geometric DRC on the routed result and prints any
violation. Exit code 0 = complete, 2 = incomplete (failed or conflicted nets;
a partial SES is still written).

`--skip-classes` leaves the named classes unrouted while their pads still act
as obstacles — the practical mode for boards whose power classes are meant to
be copper pours rather than tracks (a 2 mm track with an 800 um clearance
usually cannot legally reach the pads its class connects to; the pour can).

`--render-ses` draws a board + session to SVG without routing, and with
`--check` runs the DRC on the session's geometry — including sessions written
by other routers.

**Existing routing is preserved by default**: nets that already have tracks
or vias are left untouched and their copper becomes an obstacle — so you can
hand-route the critical nets in KiCad, then autoroute the rest around them.
`--reroute-wired` ignores existing routing and routes everything from
scratch. (This also protects against KiCad's session import deleting a
hand-routed net's tracks.)

Vias may be placed anywhere the barrel itself is legal (clearance + via
radius to foreign copper on every layer) — including under IC bodies and in
fanout moats — while wire clearances are only enforced on the layers the
wire actually runs.

**Copper pours**: nets with a zone/pour are left to the pour (KiCad exports
them as Specctra planes); footprint and board keepouts become hard obstacles.
`--reroute-wired` also disables this.

**Differential pairs** (names ending `+`/`-` or `_P`/`_N`) are detected
automatically: partners route back-to-back and the second is attracted to
the first's path, producing loosely coupled pairs.

**Post-route optimization** (`--optimize N`, default 1): after the
negotiation settles, each net is rerouted alone against the final board and
the new route kept only when it is strictly better (fewer vias, then shorter)
— straighter tracks, fewer vias, never a new conflict.

**Selective routing**: `--nets 'ETH_*,/RX*'` routes only matching nets
(`*` wildcard). Complements `--skip-classes`.

**.kicad_dru**: simple unconditional minimums (clearance, track width) are
applied automatically when a `.kicad_dru` sits next to the input board
(or pass `--dru FILE`); conditional rules are reported and ignored.

Grid rasterization, footprint stamping, and the DRC run multi-threaded
(rayon).

### GUI

```
autoroute-gui [board.dsn]
```

Open a DSN, press **Route**, and watch the router negotiate in real time
(conflicted nets flash magenta). Layer visibility toggles, pan (drag), zoom
(scroll/pinch), SES export.

## How it works

- Uniform multi-layer routing grid; pitch derived from `width + clearance`
  (override with `--grid`). Pads and the board outline are rasterized as
  hard obstacles per net; pad interiors of a net are its terminals.
- Per net: multi-source multi-target A* (8-direction plus through-via moves)
  builds an incremental Steiner tree over the net's pads.
- Negotiated congestion (PathFinder style): nets may overlap at first; each
  overlap is priced by a rising "present" cost plus an accumulating history
  cost, and conflicted nets are ripped up and rerouted until the design is
  conflict-free. Two congestion fields are kept — one for entering wires, one
  for entering vias — so both pricing and conflict detection match the exact
  DRC thresholds for wire-wire, wire-via, and via-via pairs.
- A soft "aura" one cell wider than the hard footprint makes routes keep
  spacing margin where room allows, which stabilizes convergence.
- Result extraction merges grid runs into polylines; the SES writer emits a
  session file KiCad imports directly.

A conflict-free result is DRC-correct by construction (via-pair stamp radii
include the grid quantization error), and `--check` verifies the emitted
geometry independently.

## Correctness model

Hard clearances are enforced at exact DRC distances; grid quantization is
handled by "no-diagonal" zones (a 45-degree segment can pass closer to copper
than its endpoint cells) around pads, the board edge, and committed vias,
rather than by padding margins — padding eats the tens of um of slack that
fine-pitch fanout depends on. Via barrels get their own static obstacle field
(`clearance + via radius` to foreign pads) and their own congestion field.
Negotiation stops early when the conflict set is stable for several
iterations (plateau).

## Status / limits

- Through vias only (no blind/buried).
- Differential pairs are attraction-coupled, not strictly gap-controlled;
  no length matching yet.
- Pours are respected as connectivity (pour nets skipped), but pour shapes
  are not re-filled or checked by this tool — refill zones in KiCad.
- Conditional `.kicad_dru` rules are ignored (reported).
- Extremely congested boards can end incomplete; remaining conflicted nets
  are reported and highlighted rather than silently emitted.

## Test boards

`examples/gen_board.py N out.dsn [cross|local]` generates synthetic stress
boards. `cross` wires every component to one half a board away (a worst-case
congestion torture test); `local` mimics realistic placement locality.
