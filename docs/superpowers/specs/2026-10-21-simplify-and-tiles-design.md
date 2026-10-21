# dem2mesh: tolerance-driven simplification, land-only footprints and print tiles

Status: approved by the owner on 2026-10-21 after a builder check. Written from the brainstorming
conversation of 2026-10-20 and 2026-10-21, which included two map screens of the
City at 5 km blocks and the owner's choices on them.

Part A (section 4) was implemented on 2026-10-21 through
`docs/superpowers/plans/2026-10-21-simplify.md`; Part B (section 5) is pending. The
whole City at full resolution, simplified at 2.5 m with Blender left out, took 26 s
in a Release build, against the five minutes that section 4.4 targets.

Builds on `2026-10-19-dem-to-mesh-design.md` (the implemented `dem2mesh`), whose
rules stand wherever this document is silent.

## 1. Intent

The owner wants to print the whole City of Cape Town at full 5 m resolution as a
set of 10 cm square relief blocks, made by an online printing service, that sit
next to each other like puzzle pieces with no joints. At 1:50,000 that is 13 blocks
across by 20 down, each 5 km of terrain; 146 of the 260 squares hold land. Today
`dem2mesh` cannot do this: it refuses grids above 25,000,000 vertices (the City is
254,314,752), a 5 km block at full resolution is 2,000,000 triangles and a 100 MB
STL, and most of those triangles describe flat sea and the land beyond the City
boundary, which the owner does not want printed at all.

Two things change. First, a simplifier replaces the regular grid of triangles with
a triangulated irregular network (TIN) that stays within a tolerance of the
terrain, so flat ground costs almost nothing and detail is kept where the terrain
has it. Second, the solid can be cut off where the data ends, and the window can be
split into blocks, each its own closed solid, whose shared edges match exactly.

The simplifier is also the first piece of a later, separate project the owner has
in mind: a level-of-detail terrain server for driving around Cape Town in an
OpenGL game. That project is not designed here, but the simplifier is built as a
library function of a grid and a tolerance so it can serve it.

## 2. Goals, non-goals, anti-goals

Goals:

- `--tolerance T` makes `dem2mesh` write a TIN whose vertical error at every grid
  sample is at most T model units, with the sea and other flat ground collapsing to
  a handful of triangles. Everything that works today keeps working without it.
- `--footprint data` cuts the solid off where the DEM has no data, with a wall
  along the data's edge, so a model ends at the coast and at the municipal
  boundary.
- `--tile SIZE` splits the window into square blocks of SIZE model units, written
  as one STL each, with identical terrain profiles along shared edges and one base
  plane for all, so the printed blocks sit flush and meet without a step.
- The whole City at 1:50,000, full resolution, land only, runs on the owner's
  16 GB machine in minutes and writes 146 block files small enough for a printing
  service's upload limit (typically 50 to 100 MB; the owner checks the service's
  actual figure).
- Everything is self-checked as today: every solid written is closed and faces
  out, and Blender's check runs on every block.

Non-goals:

- No connectors, pegs, or labels on the blocks. (Embossing a block's name on its
  underside would help assembly; it is a follow-up.)
- No sea. The owner chose a land-only puzzle; sea blocks are not written. Keeping
  the sea as flat slabs would need a land mask or coastline file to tell sea from
  land beyond the City boundary, which the DEM alone cannot do; this document
  leaves room for it (section 5.2) but does not design it.
- No smoothing or cleaning of the shoreline outline beyond what section 5.2 says,
  and no detection of features too thin to print; the service's checks and the
  owner's eye do that for now.
- No level-of-detail server, no streaming, no OpenGL work.

Anti-goals, which would make this a failure even if it ran:

- A step between two neighbouring blocks where their edges meet.
- A block that is not watertight or has inward faces, or a hole punched through a
  block where a vlei or a missing patch of data is.
- A simplified surface that quietly exceeds its tolerance, or that moves the
  outer edge of a block off its square.
- A whole-City run that needs more memory than the machine has, or an hour.

## 3. Constraints

- C++20, the existing `demmesh` library and its conventions (`CLAUDE.md`). No new
  external dependency: the triangulation is written in-house, which is practical
  because every vertex sits on an integer grid, so orientation and in-circle tests
  are exact in 64-bit integers for chunk sides up to 16,384 cells (the bound is
  about 29,000; section 4.5).
- Memory: the window is held once as float32 (1 GB for the whole City) and the
  resampled grid once more; everything else is per block or proportional to the
  output. The dry run states the memory a run will need.
- Blender 4.2+ as today; the scene script keeps working for a surface of any
  triangulation. Blender's memory estimate (0.67 KB per triangle) now applies to
  the whole-area surface, which is small after simplification.
- The owner builds Debug by default; the whole-City run is the first job for which
  a Release build matters, and `CLAUDE.md` says so.
- The tests stay offline and spawn only the built binaries and, where present,
  the real Blender in the background.

## 4. Part A: the simplifier

Part A is its own plan and lands first. It is useful alone: smaller STL and PLY
files, less Blender memory, and full-resolution crops that today's cap refuses.

### 4.1 Command line

| Flag | Default | Meaning |
|---|---|---|
| `--tolerance T` | off | simplify to a TIN; T is the largest vertical error allowed at any grid sample, in model units (after `--z-scale` and `--scale`), above 0; there is no default value, the flag is simply absent or given |

T is in model units because that is what the printer can resolve: 0.05 on a
1:50,000 print is 0.05 mm, which is 2.5 m of terrain, and is the value the
owner's puzzle uses. At `--scale 1` the unit is a metre. The equivalent in metres, T / (zScale x scale), is printed. Without
`--tolerance` nothing changes.

### 4.2 Where it sits in the pipeline

read window -> fill no-data -> resample to the output grid (unchanged, including
the 1:1 case) -> **simplify the grid to a TIN** -> base plane -> solid -> check ->
write -> Blender. The TIN replaces the regular surface mesh; everything downstream
takes a surface with an ordered boundary instead of assuming grid topology.

The 25,000,000-vertex cap keeps its meaning, the size of what is written, so with
`--tolerance` it no longer applies to the grid: the grid may be as large as the
window, and the dry run prints the two float32 buffers' size as the plan's
`memory:` line, which counts those two buffers only; the fill's work lists and a
large TIN come on top.
The 4,000,000-vertex warning likewise moves to the simplified output and is
printed after simplification. A simplified mesh above 25,000,000 vertices is
refused once it has been simplified, before the solid is built and with nothing
written:

    error: the simplified mesh is N vertices; the limit is 25000000: raise --tolerance, or use --grid or crop

The grid itself is limited to 2,147,483,647 vertices, the most an int holds, which
keeps both of its sides within an int, and is refused before any pixel is read
above that:

    error: the output grid is N vertices; with --tolerance the limit is 2147483647: use --grid or crop

(Without `--tolerance` both apply to the grid as today.)

### 4.3 What the simplifier guarantees

Given a grid of cols x rows heights and a tolerance in metres, it returns a
triangulated surface over the grid's rectangle such that:

- every vertex is a grid sample, at the sample's exact height;
- the four corners are vertices, and the rectangle's four sides are made of
  boundary vertices only (the surface never leaves the rectangle);
- for every grid sample, the vertical distance between the sample and the surface
  above or below it is at most the tolerance;
- the surface is a single sheet: triangles counter-clockwise seen from above, no
  overlaps, no gaps, no vertex used by no triangle;
- the result is deterministic: the same grid and tolerance give the same
  triangles on every run and platform.

It does not promise the fewest triangles possible, but for a grid that fits one
chunk (section 4.5) a plane of any slope must come out as its four corners and two
triangles, and a grid of random noise with a tolerance above the noise's range must
do the same. A larger grid gives two triangles per chunk, since the chunk seams keep
their sides' vertices, which section 4.5 accepts.

### 4.4 Algorithm (the decision, with the parts left to the builder)

Greedy insertion in the style of Garland and Heckbert's terrain simplification:
start from the boundary, repeatedly insert the grid sample whose vertical error
against the current surface is largest, and stop when the largest error is within
the tolerance. Each triangle tracks its worst sample and its error; a heap keyed
by error gives the next insertion; only the triangles changed by an insertion are
rescanned. Points are scanned by walking the grid rows a triangle covers, and a
sample on an edge between two triangles is counted by exactly one of them, so the
error of the final surface is checked at every sample exactly once.

Two decisions make it exact and tileable:

- **Integer coordinates.** Vertices are addressed by their grid column and row.
  Orientation and in-circle predicates are evaluated in exact integer arithmetic
  on coordinates local to a chunk. The grid is processed in chunks (section 4.5)
  whose side never exceeds 16,384 cells, so the in-circle determinant, about
  12 x side^4, stays inside 64 bits with margin. No epsilon anywhere.
- **Boundaries first, in one dimension.** Before a chunk's interior is touched,
  each of its four sides is simplified on its own: the profile of heights along
  the side, with the two corners fixed, is reduced to the smallest set of samples
  such that the straight segments between them are within the tolerance of every
  sample on the side (Douglas and Peucker's split rule with vertical distance,
  which is deterministic given the profile and the tolerance). Those side vertices
  are inserted first and their segments are kept as the chunk's boundary; the
  interior insertion never adds a vertex to a side, because every sample on the
  side is already within tolerance of the boundary segments. Two chunks that share
  a side therefore share the very same boundary vertices.

Left to the builder: the point-location and cavity-retriangulation scheme
(incremental Delaunay with the boundary as fixed edges, or any equivalent that
keeps the guarantees of 4.3); the tie rule when several samples share the largest
error (it must be deterministic, for instance the first in scan order); how the
heap and the per-triangle sample lists are stored. Part A's plan pins a run-time
target, measured rather than enforced: the whole City (254 million samples, sea
included) simplified at 2.5 m in about five minutes in a Release build on the
owner's machine, Blender excluded. Section 5.5 gives Part B's.

### 4.5 Chunks and the whole-window surface

The simplifier works on a lattice of chunks: an origin and a chunk size in output
cells. Without `--tile`, the lattice is 2048 x 2048 cells from the window's
north-west corner (the last chunks cut short), so memory for the triangulation is
bounded and the predicates stay exact. With `--tile` (Part B) each block is one
chunk, so a block's sides are exactly the constraints that make shared edges
identical; a block side above 16,384 cells is refused before anything is read,
with a message suggesting a smaller `--tile` or a coarser `--grid`. Chunks share
boundary vertices by construction (4.4), so their surfaces join into one sheet
over the window: the builder merges chunk vertices along shared sides (they are
identical samples) and keeps the outer boundary as the window's boundary loop,
which the solid builder uses for its walls. The seams add a few triangles along
chunk edges compared with one global triangulation; that is accepted.

### 4.6 The solid from a surface with a boundary

`solidMesh` today assumes grid topology. Part A generalises it: a surface plus an
ordered outer boundary loop (as four sides, each a list of vertices from one
corner to the next) gives walls and a base. For the rectangle the existing
construction stays, side by side: each wall is the monotone polygon between a
side's vertices and its two bottom corners, triangulated as today, and the base is
two triangles. The triangle count is no longer a closed formula; the self-check
compares the count the construction reports with what was built, and keeps the
directed-edge, zero-area and signed-volume checks unchanged.

### 4.7 PLY, Blender and the summary

The PLY carries the TIN surface with the same planar UVs (x over width, y over
height), which need no grid. The Blender script changes only in that the STL check
compares the file's declared count with the import, as it already does; nothing
depends on the grid formula.

The summary gains one line after `grid:`, in the format the plan pinned (plain
integers, like every other count of the summary, and no "in the mesh"), here for
the Table Mountain crop at `--grid 500 --scale 0.02 --tolerance 0.05`:

    simplify: tolerance 0.05 units (2.5 m), 255500 grid vertices to 21985, 508980 triangles to 43649

The tolerance is shown as given and in the metres of terrain it stands for, to the
centimetre; the triangles are the surface's, the grid's then the TIN's. The dry
run's plan puts one more line before it, for the memory of section 4.2 (the sum to
the nearest megabyte, or from 1,000 MB up in gigabytes to one decimal):

    memory: window 568452 samples and grid 255500 vertices, 4 bytes each: about 3 MB

The `solid:` line counts the simplified solid, and with `--tolerance` the `summit:`
line gives the highest vertex of the simplified mesh, which the tolerance can leave
below the grid's summit. The dry run simplifies too (it has always built and checked
the solid), so its counts and sizes are exact; with `--verbose` the `time: simplify`
line shows the cost.

## 5. Part B: footprints and tiles

Part B is its own plan and lands after Part A, which it builds on.

### 5.1 Command line

| Flag | Default | Meaning |
|---|---|---|
| `--footprint window\|data` | `window` | `window`: the solid covers the whole window as today; `data`: the solid exists only where the DEM has data (section 5.2) |
| `--tile SIZE` | off | split the window into square blocks of SIZE model units, each written as its own STL (section 5.3) |
| `--blocks ROWS:COLS` | all | write only these blocks, each part a number or a range, for instance `3-5:2-4` or `7:12`; the others are listed but not written |
| `--min-piece A` | 1 | a connected piece of footprint smaller than A square model units is dropped and reported (section 5.2) |

The puzzle run is:

    dem2mesh -i CCT_GLM2019_5m_DEM_1D_v1.tif -o <dir>/cape-town --scale 0.02 --tolerance 0.05 --footprint data --tile 100

`--footprint data` works with or without `--tile`, so a Table Mountain crop can
end at the coast as one piece. `--tile` works with `--footprint window` too, which
gives full squares with the fill where the sea was.

### 5.2 The footprint

The DEM's no-data (`GDAL_NODATA` and NaN) is the mask. The footprint is decided on
the window's grid before the fill, and carried through resampling by taking, for
each output vertex, the status of the nearest source sample.

- A **cell** is the square between four neighbouring grid vertices. A cell is in
  the footprint when all four of its vertices have data. The footprint is the
  union of those cells, so its outline runs along grid lines through real samples
  with real heights; the shoreline wall's top edge is measured ground, and all
  vertices keep integer coordinates for the simplifier. The outline sits up to
  one cell inside the true data edge, which is 5 m here.
- No-data that is connected (4-connected) to the window's edge is outside: sea,
  and land beyond the City boundary. No-data that is enclosed by data is **filled**
  as today and stays inside the footprint, so a vlei, a dam or a missing patch
  does not punch a hole through a block. (With a crop, a bay whose mouth lies
  outside the window counts as enclosed and is filled; this is accepted and
  documented.)
- Before the enclosed test, no-data slits one or two cells wide are closed: the
  footprint is closed morphologically with a 3 x 3 neighbourhood, so an estuary or
  vlei joined to the sea by a one-sample channel counts as enclosed and is filled
  rather than cut as a 10 m slit into the land. Wider channels are real water and
  stay cut.
- A **piece** is a 4-connected set of footprint cells (cells sharing an edge).
  Data that forms a separate piece, Robben Island for instance, is its own part of
  the footprint and is written in whichever block holds it. Two pieces may touch
  at a single vertex; the simplifier gives them that one shared vertex and both
  shells keep it (an STL has no shared vertices anyway, and the PLY keeps one).
  Such contacts pass the edge check and Blender's; the summary counts them,
  `corner contacts: N`, so the owner can look at them if a printing service
  objects.
- A piece smaller than `--min-piece` square model units (1 mm², which is 50 x
  50 m or 100 cells at 1:50,000) is dropped and listed, so a few stray cells do not
  become slivers no printer can make. Pieces are measured after tiling, per block.

Inside the footprint the simplifier of Part A runs with the footprint's outline as
its boundary. The outline's vertices are the cells' corner samples; straight runs
along a grid line are kept as single segments where every sample on the run is
within tolerance of the segment, by the same one-dimensional rule as the sides
(4.4), so a straight stretch of coast does not cost a vertex per sample. The
outline is never moved off the grid lines: a wall stands exactly on the data's
edge.

Leaving room, not building it: a later `--land-mask FILE` could add a second mask
(a coastline polygon or raster) so that no-data inside the mask becomes a flat
slab at a given height instead of nothing. The footprint logic takes a mask, not
the DEM's no-data specifically, so that addition would not change the design.

### 5.3 Tiles

- The block grid is aligned to the window's north-west corner and counted in
  output-grid cells, since those are what the simplifier sees. A block is
  `round(SIZE / (scale x cellX))` cells wide and `round(SIZE / (scale x cellY))`
  cells tall, where cellX and cellY are the output grid's cell sizes in metres
  (the window's extent over grid - 1; at full resolution the sample spacings,
  equal for a projected file and different for a geographic one), so each block
  is as close to SIZE square as the grid allows; the actual block size in cells,
  metres and model units is printed. A block side above 16,384 cells is refused
  (4.5). Blocks share their boundary vertices: a 1000-cell block has 1001
  vertices across, the last column being the next block's first. The east and
  south edges may end in blocks cut short.
- Blocks are numbered from the north-west, row then column, from 1. Files are
  `STEM_rRR_cCC.stl` with as many digits as the counts need (`cape-town_r03_c05.stl`).
  Each block's STL is in its own local frame: x and y from the block's south-west
  corner, z as in every `dem2mesh` output (sea level at 0, the base plane below).
- **Shared edges match exactly.** A block side is simplified in one dimension
  (4.4) over runs split wherever the data status of the cells on either side of
  the line changes, each run with its ends fixed. A run that both neighbours
  include is simplified from the same samples by the same rule in both, so their
  boundary vertices, and hence their walls, are identical; a run only one block
  includes is that block's shoreline. The test that pins this compares the two
  blocks' vertex lists along every shared side of a synthetic island, byte for
  byte.
- **One base plane.** The base plane and plinth are chosen once from the whole
  window's data heights (the footprint's, not the fill's), with today's rule, and
  every block uses it, so the blocks sit flush. `--base` overrides as today. On the
  City the lowest data point is -37.9 m, so the plane is about 0.8 mm below sea
  level plus the plinth; the coast's slab is therefore about the plinth thick.
- **Walls and base of a footprint block.** The surface's boundary loops (one per
  piece) are known. Each boundary edge becomes a wall quad, two triangles from the
  edge down to the base plane. The base is the footprint polygon at the base
  plane, triangulated over the same boundary vertices (ear clipping or a
  constrained triangulation, builder's choice) and wound to face down. No
  T-junctions arise because every boundary vertex has its base counterpart. The
  rectangle's two-triangle base and monotone walls of Part A remain for
  `--footprint window` without tiles; a tiled or data-footprint block uses the
  polygon construction.
- **Not written:** blocks with no footprint, and blocks outside `--blocks`. Both
  are listed in the summary. A block that holds land is always written, however
  little, unless every piece in it falls under `--min-piece`.

### 5.4 Outputs, Blender and the summary

- Per block: `STEM_rRR_cCC.stl`, written through the atomic writer as today.
- For the whole area: `STEM.ply`, the union of the blocks' surfaces in the global
  frame (shared vertices merged, which is exact since they are identical samples),
  with planar UVs over the whole window; and `STEM.blend` built from it, so one
  scene previews the assembled puzzle. `--no-blend` skips both. The size warning
  and Blender's memory estimate apply to this whole-area surface.
- Blender's STL check runs over every block file in one Blender session: the job
  passes a list file of STL paths, the script imports each into an empty scene in
  turn, and prints one line, `stl check: 146 files, 12,345,678 triangles, 0
  non-manifold edges, 0 inverted faces`, failing on the first bad file with its
  name. `dem2mesh` repeats that line as its last.
- Summary: `input:`, `crop:`, `window:`, `grid:`, `simplify:` (totals), `heights:`,
  `summit:`, `base:`, then `tiles: 13 x 20 blocks of 1000 x 1000 samples (5,000 x
  5,000 m, 100 x 100 mm); 146 with land, 114 without, 32 cut short at the east or
  south edge`, then one line per written block, `block r03 c05: 72% land, 184,220
  triangles, 9.2 MB, cape-town_r03_c05.stl`, then `skipped: no land in r01 c05,
  r01 c06, ...` and `dropped pieces:` when any, then the `wrote:` lines for the
  PLY and `.blend` and Blender's check. The dry run prints the same with `would
  write` and no files.
- With `--overwrite`, existing block files are replaced; an existing block file
  that this run would not write (a block that no longer has land, after a change
  of crop or tolerance) is left alone and named in a `stale:` line.

### 5.5 Memory and time for the whole City

Read 1 GB, fill, resample 1:1 into a second 1 GB buffer, release the window, then
blocks one at a time; the whole-area surface for the PLY accumulates in memory
(order of a few hundred MB at 2.5 m). Target on the owner's machine, Release build:
under ten minutes end to end, Blender included; the plan measures and records it.
A Debug build may take several times longer and `CLAUDE.md` says so.

## 6. What stays the same

Inputs, projections, the crop, the fill rule for enclosed holes, resampling, the
base-plane rule, `--z-scale` and `--scale`, atomic writes, exit codes, the Blender
search and checks, and every existing test. `--footprint window` without
`--tolerance` and `--tile` is byte-for-byte today's output.

## 7. Verification

Beyond the unit tests the plans will pin, the acceptance checks are:

- Part A: the Table Mountain crop at `--grid 500 --scale 0.02 --tolerance 0.05`
  writes a closed solid whose vertical error at every grid sample is at most 2.5 m
  (checked by a test reading the STL back and the grid), with far fewer than
  511,004 triangles; a plane gives 12 triangles (the box); the whole r3_c1 tile at
  `--grid 400` loses its sea triangles.
- Part B: a synthetic island file split into 2 x 2 blocks: every block closed and
  outward, shared edges identical, the island's outline on grid lines, the
  enclosed hole filled, the sea absent; the owner's puzzle command on the City
  writes about 146 files (the map screens counted squares with any data sample;
  the cell rule, the slit closing and `--min-piece` may move the count by a few,
  and the summary's skipped and dropped lines account for each), each under the
  service's limit, Blender's check passes on all, and two neighbouring blocks
  opened together in Blender show no step.

## 8. Where it lives

- `src/mesh/Predicates.h/.cpp`: exact integer orientation and in-circle.
- `src/mesh/ProfileSimplifier.h/.cpp`: the one-dimensional rule of 4.4.
- `src/mesh/Simplifier.h/.cpp`: the greedy TIN over a rectangle or a footprint,
  with fixed boundary runs; the library function the later terrain project reuses.
- `src/mesh/Footprint.h/.cpp`: mask to cells, components, enclosed holes, outline
  loops, `--min-piece`.
- `src/mesh/Tiles.h/.cpp`: the block grid, shared-side runs, block naming.
- `src/mesh/SolidMesh`: generalised to a surface with boundary loops; the polygon
  base and wall quads for footprints.
- `src/app/MeshCli.cpp`, `src/app/dem2mesh_main.cpp`: the flags, the per-block
  loop, the summary; `src/mesh/blender_scene.py`: the multi-file check.
- Two plans: `2026-10-21-simplify.md` (Part A) and, after it lands,
  `2026-10-21-footprints-and-tiles.md` (Part B).

## 9. Decisions taken on the owner's behalf

- Tolerance in model units with no default: off unless given; 0.05 (2.5 m at
  1:50,000) is the puzzle's value.
- Chunks of 2048 cells without tiles, the block itself with them, 16,384 cells at
  most; deterministic scan-order tie rule; Douglas and Peucker with vertical
  distance for the sides.
- Cells need all four corners for data; slits of one or two cells are closed;
  enclosed no-data is filled; edge-connected no-data is cut; pieces are
  4-connected and may touch at a vertex; `--min-piece` 1 mm² (50 x 50 m at
  1:50,000).
- Run-time targets of five minutes (Part A, whole City) and ten (Part B, the
  puzzle, Blender included) in a Release build, measured rather than enforced.
- Blocks numbered from the north-west, `STEM_rRR_cCC.stl`, local frames, one
  base plane from the whole window's data.
- The dry run simplifies, so a whole-City dry run costs as much as the run minus
  the writing.
- With `--tolerance`, the 25,000,000-vertex limit is applied to the simplified
  mesh once it has been simplified, with its own message (section 4.2).
- With `--tolerance`, `summit:` reports the TIN's highest vertex, not the grid's
  (section 4.7).
- Whole-area PLY and `.blend` in tile mode; Blender checks all blocks in one
  session.
- Part A and Part B as two plans in that order.
