# dem2mesh Footprints and Tiles (Part B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `dem2mesh --footprint data --tile SIZE` writes the window as 10 cm puzzle blocks that exist only where the DEM has data, each a closed solid whose shared edges match its neighbours exactly, with one PLY and Blender scene of the whole area and Blender's check over every block.

**Architecture:** A footprint unit turns the window's no-data into a cell mask (cells with four data corners, slits closed, enclosed holes filled, edge-connected no-data cut) and, per block, into 4-connected pieces with outline loops split into straight runs. A tiles unit lays the block lattice over the output grid and names, selects and frames blocks. The Part A simplifier gains a footprint mode: each piece's outline is simplified run by run through the existing side cache (so neighbours share vertices), triangulated by exact ear clipping, legalised, and refined by greedy insertion over the piece's interior samples. A footprint solid builder adds wall quads per outline edge and a base from the same ear clipping. `dem2mesh` loops over blocks, writes each STL in its local frame, accumulates one whole-area surface for the PLY, and hands Blender a list of STLs to check in one session.

**Tech Stack:** C++20, Catch2 v3.16.0, CLI11 v2.7.2, CMake + Ninja, Blender 4.2+ (background); no new dependency.

**Spec:** `docs/superpowers/specs/2026-10-21-simplify-and-tiles-design.md`, sections 5, 6, 7 (Part B), 8 and 9; section 4 (Part A, implemented in `docs/superpowers/plans/2026-10-21-simplify.md`) is the engine this plan extends.

## Global Constraints

- C++20, `-Wall -Wextra` clean, the conventions of `CLAUDE.md`; headers as `"mesh/<Name>.h"`; namespace `demdl::mesh`; no new external dependency (spec 3).
- Exact integer predicates on block-local coordinates; a block side above 16,384 cells is refused (spec 4.5, 5.3).
- The footprint rules of spec 5.2, verbatim: a cell is in the footprint when all four of its vertices have data; no-data 4-connected to the window's edge is cut, enclosed no-data is filled; before the enclosed test the footprint is closed morphologically with a 3 x 3 neighbourhood; a piece is a 4-connected set of cells; pieces may touch at a vertex, counted as `corner contacts: N`; a piece under `--min-piece` square model units (default 1) is dropped and listed; the mask is carried through resampling by the nearest source sample; the outline is never moved off the grid lines.
- Shared edges match exactly (spec 5.3): a block side is simplified over runs split wherever the data status of the cells on either side of the line changes, each run with its ends fixed, by the one-dimensional rule in the fixed direction of Part A (rows west to east, columns north to south).
- One base plane for all blocks, from the whole window's data heights with today's rule (spec 5.3).
- Blocks are counted in output-grid cells from the window's north-west corner, `round(SIZE / (scale x cellX))` by `round(SIZE / (scale x cellY))`, numbered from the north-west, row then column, from 1; files `STEM_rRR_cCC.stl` with as many digits as the counts need; each block in its own local frame, x and y from the block's south-west corner, z unchanged (spec 5.3).
- Outputs of spec 5.4: per-block STLs through the atomic writer; one `STEM.ply` and `STEM.blend` of the whole area (shared vertices merged, planar UVs over the window); Blender checks every block in one session and prints one `stl check:` line; the summary's `tiles:`, `block`, `skipped:`, `dropped pieces:`, `stale:` lines; the dry run prints the same with `would write`.
- `--footprint window` without `--tile` and without the new flags is byte for byte today's output (spec 6).
- Every solid written is closed and faces out; the surface check of Part A (`downwardFacingTriangles`) runs on every block (spec 6).
- Tests stay offline; Blender only in the background; test names must not begin with `-`; never `git add -A`; nothing written into `~/Downloads`; commit messages end with the attribution line of the model doing the work.

## Deviations from the spec (deliberate, small)

1. **Counts print as plain integers and sizes in "units"**: `block r03 c05: 72% land, 184220 triangles, 9.2 MB, ...` and `tiles: ... (5000 x 5000 m, 100 x 100 units)`, matching every other line (Part A deviation 1); the spec's "184,220" and "100 x 100 mm" are illustrative.
2. **Without `--tile`, `--footprint data` runs the window as one block**, so a window above 16,384 cells a side is refused with `the window is W x H cells; a footprint without --tile is limited to 16384 a side: use --tile or --grid`. The puzzle uses `--tile`; the spec's "with or without --tile" holds for crops.
3. **The cell mask, its closing and the enclosed test are computed once for the whole window**, not per block, so that two blocks see the same cells along their shared side (spec 5.2 describes the rules, not where they run).
4. **Without `--overwrite`, any existing block file of the lattice is refused before anything is read**, whether or not this run would write it (`<file> exists; use --overwrite`), because which blocks hold land is known only after reading. With `--overwrite`, the `stale:` line of spec 5.4 names the ones left alone.
5. **The ear clipping that triangulates each base also seeds the piece's TIN** (spec 5.3 leaves the method to the builder): one exact algorithm for both, and the outline's edges are hull edges that the legalisation never flips.

## Review Focus

1. A block whose only land is a speck under `--min-piece` must be skipped and listed as dropped, not written as a sliver: Task 7.
2. Two pieces touching at one vertex in a block must give two closed shells, pass the checks, and print `corner contacts: 1`: Tasks 5 and 7.
3. A tile size that does not divide the grid must give cut-short blocks on the east and south that are closed and match their neighbours: Tasks 4 and 7.
4. `--footprint data` on a file without any no-data must give the whole window as one piece, closed, with the same surface as `--footprint window`: Task 7.
5. `--overwrite` after a change that empties a block must leave the old block file alone and name it in `stale:`: Task 7.

## File structure

| File | Responsibility |
|---|---|
| `src/mesh/PolygonTriangulation.h/.cpp` (new) | `triangulatePolygon`: exact ear clipping of a simple polygon on grid points |
| `src/mesh/Footprint.h/.cpp` (new) | `dataMaskOf`, `resampleMaskNearest`, `footprintCells`, `blockFootprint`: cells, closing, enclosed fill, pieces, runs, drops, corner contacts |
| `src/mesh/Tiles.h/.cpp` (new) | `TileLayout`, `blockRect`, `blockName`, `BlockSelection`, `parseBlocks` |
| `src/mesh/Simplifier.h/.cpp` (modify) | `BlockSimplifier`: Part A's engine per block, rectangle or footprint pieces, one whole-area surface |
| `src/mesh/FootprintSolid.h/.cpp` (new) | `solidFromPieces`, `solidTriangleCount(pieces, ...)`: wall quads and ear-clipped bases |
| `src/app/MeshCli.h/.cpp` (modify) | `--footprint`, `--tile`, `--blocks`, `--min-piece` |
| `src/app/dem2mesh_main.cpp` (modify) | the block loop, local frames, the new summary and plan lines, `stale:` |
| `src/mesh/BlenderRunner.h/.cpp`, `src/mesh/blender_scene.py` (modify) | the STL list and the one-line check over many files |
| `tests/` (new and modified) | `test_PolygonTriangulation.cpp` `[polygon]`, `test_Footprint.cpp` `[footprint]`, `test_Tiles.cpp` `[tiles]`, `test_Simplifier.cpp`, `test_FootprintSolid.cpp` `[footsolid]`, `test_MeshCli.cpp`, `test_Dem2MeshEndToEnd.cpp`, `test_BlenderRunner.cpp`, `FakeBlender.h`, and `tests/IslandFixture.h` (new) |
| `CLAUDE.md`, the spec (modify) | Task 9 |

---

### Task 1: Exact ear clipping of a polygon on grid points

Spec 5.3 (base triangulation, builder's choice) and deviation 5.

**Files:**
- Create: `src/mesh/PolygonTriangulation.h`, `src/mesh/PolygonTriangulation.cpp`
- Test: `tests/test_PolygonTriangulation.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `GridPoint`, `orient2d`, `kMostPredicateCoordinate` (`mesh/Predicates.h`).
- Produces:
  ```cpp
  // The triangles of a simple polygon given as its vertices in counter-clockwise order (orient2d positive turns, with
  // x to the right and y up), as triples of indices into `loop`, each counter-clockwise, n - 2 of them. Vertices may be
  // collinear with their neighbours (a straight run of a rectilinear outline keeps samples along it); such a vertex is
  // never an ear itself, and no triangle has another vertex of the polygon inside it or on one of its edges, so the
  // result has no T-junction and no triangle without area. Ear clipping with the exact predicates, deterministic:
  // the first ear in index order is clipped at every step. Coordinates must lie within kMostPredicateCoordinate.
  // Throws std::invalid_argument when loop has fewer than 3 vertices, repeats a vertex, is clockwise or has no area,
  // or no ear can be found (a self-intersecting polygon).
  [[nodiscard]] std::vector<std::array<std::uint32_t, 3>> triangulatePolygon(std::span<const GridPoint> loop);
  ```

- [ ] **Step 1: Write the failing tests** in `tests/test_PolygonTriangulation.cpp`, tag `[polygon]`, with a helper `checkTriangulation(loop, triangles)` asserting: `triangles.size() == loop.size() - 2`; every triangle `orient2d > 0`; the doubled areas sum to the polygon's doubled shoelace area exactly (integers); every polygon edge `(k, k+1)` appears in exactly one triangle in that direction and every other triangle edge in exactly two, opposite ways; no vertex lies inside or on an edge of a triangle it is not a corner of. Cases:
  - a 2 x 2 square `{(0,0),(2,0),(2,2),(0,2)}`: two triangles;
  - the same square with every lattice point on its sides as a vertex (8 vertices): 6 triangles, `checkTriangulation` passes;
  - an L-shape `{(0,0),(4,0),(4,2),(2,2),(2,4),(0,4)}` and the same with all lattice points on its edges;
  - a staircase coast: the outline of the cells `{(0,0),(1,0),(1,1),(2,1),(2,2),(3,2)}` (6 cells) with every lattice point on it;
  - 500 random rectilinear polygons: take a random 4-connected set of cells in a 12 x 12 box (grown from a seed by random steps, 1 to 60 cells), trace its outer outline with the test's own tracer (write it in the test; Task 2's production tracer is independent), keep every lattice point on it, and call `checkTriangulation`;
  - determinism: the staircase twice gives identical triples;
  - refusals: two vertices; the square clockwise; a repeated vertex; a bow tie `{(0,0),(2,2),(2,0),(0,2)}`; each `std::invalid_argument`.
- [ ] **Step 2: Build; expect** the missing-header errors.
- [ ] **Step 3: Implement.** Ear test for vertex v with neighbours u, w in the remaining polygon: `orient2d(u, v, w) > 0` (strictly convex) and no other remaining vertex p with `orient2d(u, v, p) >= 0 && orient2d(v, w, p) >= 0 && orient2d(w, u, p) >= 0` (inside or on an edge). O(n^2) per clip is acceptable here (outlines are a few thousand vertices); use a doubly linked index list so a clip is O(1) and the ear search restarts from the clipped vertex's predecessor.
- [ ] **Step 4: Run** `./build/demdl_tests "[polygon]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: exact ear clipping of a grid polygon`.

---

### Task 2: The footprint

Spec 5.2, deviation 3.

**Files:**
- Create: `src/mesh/Footprint.h`, `src/mesh/Footprint.cpp`
- Test: `tests/test_Footprint.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `GridPoint` (`mesh/Predicates.h`).
- Produces:
  ```cpp
  // Which samples of a window have data: 1 where the height is finite, 0 where it is NaN (readWindow's no-data).
  [[nodiscard]] std::vector<std::uint8_t> dataMaskOf(const std::vector<float>& window);

  // The mask on the output grid: output vertex (i, j) takes the status of the nearest source sample, at the position
  // u_i = i (srcCols - 1) / (outCols - 1) rounded to the nearest sample (half rounds down), likewise down the rows.
  [[nodiscard]] std::vector<std::uint8_t> resampleMaskNearest(const std::vector<std::uint8_t>& mask, int srcCols,
                                                              int srcRows, int outCols, int outRows);

  // The footprint's cells on a grid of cols x rows vertices: (cols - 1) x (rows - 1) flags, row by row from the
  // north-west. A cell is in when all four of its vertices have data; the result is then closed with a 3 x 3
  // neighbourhood (dilated, then eroded); then every 4-connected region of cells still out that touches the grid's
  // edge stays out, and every enclosed one is turned in. Throws std::invalid_argument when cols or rows is below 2 or
  // mask does not hold cols x rows values.
  [[nodiscard]] std::vector<std::uint8_t> footprintCells(const std::vector<std::uint8_t>& mask, int cols, int rows);

  struct BlockRect { int firstColumn, firstRow, lastColumn, lastRow; };  // vertex indices, sides included

  // One piece of a block's footprint: its outline as every sample on it in order, counter-clockwise seen from above
  // (x east, y north: with rows numbered southward that is orient2d negative turns in (column, row)), closed (the
  // first sample is not repeated), and the indices into `loop` where its straight runs start: a run ends where the
  // outline turns, and on a block side also where the status of the cell across the side changes, so that the
  // neighbouring block splits the same runs. cells is the piece's area in cells.
  struct Piece {
    std::vector<GridPoint> loop;        // grid coordinates (column, row) of the window
    std::vector<std::uint32_t> runStarts;
    std::uint64_t cells = 0;
  };

  struct BlockFootprint {
    std::vector<Piece> pieces;          // kept, largest first, ties by the north-west-most cell
    std::vector<Piece> dropped;         // under minPieceUnits2
    std::uint64_t cells = 0;            // footprint cells in the block, kept pieces only
    std::uint64_t cornerContacts = 0;   // vertices of the block where two kept pieces touch at a point
  };

  // The pieces of the footprint inside rect: 4-connected components of the cells in rect's cell range, each outlined
  // and split into runs as Piece says. A component whose cells x cellAreaUnits2 is below minPieceUnits2 goes to
  // dropped. `cells` is footprintCells' result for the whole grid, which is also consulted across rect's sides to
  // split the runs. Throws std::invalid_argument when rect is not inside the grid or has no cell.
  [[nodiscard]] BlockFootprint blockFootprint(const std::vector<std::uint8_t>& cells, int cols, int rows,
                                              BlockRect rect, double cellAreaUnits2, double minPieceUnits2);
  ```

- [ ] **Step 1: Write the failing tests** in `tests/test_Footprint.cpp`, tag `[footprint]`, with a 24 x 18 mask drawn in the test as rows of characters (`#` data, `.` no-data) holding: an island of about 10 x 8 data samples; inside it a 2 x 2 lake of no-data; a 1-sample-wide channel from the lake to the sea; a separate 3 x 3 islet; a single data sample alone; and two 2 x 2 blobs touching at one vertex. Cases:
  - `dataMaskOf` maps NaN to 0 and finite to 1 on a 5-value vector;
  - `resampleMaskNearest` of a 4 x 3 mask to 7 x 5 and to 2 x 2: pinned values at the corners and the middle;
  - `footprintCells`: the lake is in (enclosed), the channel is in (closed as a slit), the sea is out, the lone sample makes no cell, the islet makes 4 cells; a 3-wide channel stays out; the whole grid of data gives every cell in; a grid of no-data gives none;
  - `blockFootprint` over the whole grid: pieces = the island (lake included), the islet and the two touching blobs, in area order; `cornerContacts == 1`; with `minPieceUnits2` just above the islet's area the islet is in `dropped`; a piece's loop is counter-clockwise seen from above (shoelace in model orientation positive), closed, every consecutive pair one cell apart along a grid line, and its `runStarts` are exactly the turns;
  - a block side splitting runs: with `rect` the west half, the island's run along the block's east side splits where the east half's cells across the side change from in to out; the east half's piece has the same split points on that line;
  - refusals: a rect outside the grid; a rect of one column.
- [ ] **Step 2: Build; expect** the missing-header errors.
- [ ] **Step 3: Implement.** Components by breadth-first search over the block's cells; the outline by walking the boundary edges of the component's cells keeping the component on the left (seen from above), so that at a corner contact the walk turns back along the piece's own edge rather than crossing into the other piece; corner contacts counted as block vertices whose four surrounding cells form a diagonal pair of kept pieces.
- [ ] **Step 4: Run** `./build/demdl_tests "[footprint]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: footprint cells, pieces and outlines from the window's no-data`.

---

### Task 3: The block lattice

Spec 5.3 and 5.1 (`--blocks`).

**Files:**
- Create: `src/mesh/Tiles.h`, `src/mesh/Tiles.cpp`
- Test: `tests/test_Tiles.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `BlockRect` (Task 2), `kMostChunkCells` (`mesh/Simplifier.h`).
- Produces:
  ```cpp
  // The block lattice over an output grid of cols x rows vertices: blocks of blockCols x blockRows cells from the
  // north-west corner, `across` x `down` of them, the last ones each way cut short. blockCols = round(tileUnits /
  // (scale x cellXM)) clamped to [1, cols - 1], likewise blockRows with cellYM and rows - 1. Throws
  // std::invalid_argument when a side would exceed kMostChunkCells, naming the side and suggesting a smaller --tile
  // or a coarser --grid, or when tileUnits, scale, cellXM or cellYM is not above 0 and finite.
  struct TileLayout { int cols, rows, blockCols, blockRows, across, down; };
  [[nodiscard]] TileLayout tileLayout(int cols, int rows, double tileUnits, double scale, double cellXM, double cellYM);
  // Block (r, c), 0-based, as vertex indices; the east and south edges cut short. Throws when out of range.
  [[nodiscard]] BlockRect blockRect(const TileLayout& layout, int r, int c);
  [[nodiscard]] bool cutShort(const TileLayout& layout, int r, int c) noexcept;
  // "r03_c05": 1-based, zero-padded to the digits `down` and `across` need (at least 1).
  [[nodiscard]] std::string blockName(const TileLayout& layout, int r, int c);
  // --blocks ROWS:COLS, each part N or A-B, 1-based, inclusive; `all` when no text was given.
  struct BlockSelection {
    bool all = true;
    int firstRow = 1, lastRow = 1, firstCol = 1, lastCol = 1;
    [[nodiscard]] bool selected(int r, int c) const noexcept;  // 0-based r, c
  };
  // Throws std::invalid_argument with a usage-style message for text that is not ROWS:COLS of whole numbers from 1,
  // or a range whose end is below its start.
  [[nodiscard]] BlockSelection parseBlocks(const std::string& text);
  ```

- [ ] **Step 1: Write the failing tests** in `tests/test_Tiles.cpp`, tag `[tiles]`:
  - the City at 1:50,000: `tileLayout(12912, 19696, 100, 0.02, 5, 5)` gives `blockCols 1000, blockRows 1000, across 13, down 20`; `blockRect(l, 19, 12)` is columns 12000 to 12911 and rows 19000 to 19695 and `cutShort`; `blockRect(l, 0, 0)` is 0 to 1000 both ways; `blockName(l, 2, 4) == "r03_c05"`; `blockName` on a 7 x 9 layout gives `"r3_c5"`;
  - a geographic cell size: `cellXM 92.2, cellYM 110.9` with `tileUnits 100, scale 0.01` gives different `blockCols` and `blockRows`, each `round(100 / (0.01 x cell))`;
  - a tile larger than the grid gives one block of the whole grid; a tile of under one cell gives blocks of one cell;
  - refusal above 16,384 cells a side with the message naming the side;
  - `parseBlocks("3-5:2-4")`, `"7:12"`, `"1-1:1-1"`; `selected` at the edges of the ranges; refusals: `"3-5"`, `"a:b"`, `"5-3:1"`, `"0:1"`, `":"`.
- [ ] **Step 2: Build; expect** the missing-header errors.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `./build/demdl_tests "[tiles]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: block lattice, names and --blocks selection for tiles`.

---

### Task 4: The simplifier per block, over a footprint

Spec 5.2 ("inside the footprint the simplifier of Part A runs with the footprint's outline as its boundary"), 5.3 (shared edges), 4.5 (block = chunk).

**Files:**
- Modify: `src/mesh/Simplifier.h`, `src/mesh/Simplifier.cpp`
- Test: `tests/test_Simplifier.cpp`, and `tests/IslandFixture.h` (new: the island mask of Task 2's test and matching heights as a shared fixture for Tasks 4, 5 and 7)

**Interfaces:**
- Consumes: `triangulatePolygon` (Task 1); `Piece`, `BlockFootprint`, `BlockRect` (Task 2); the existing `ChunkTin`, `SideCache`, `SurfaceBuilder`, `simplifyProfile`.
- Produces (in `Simplifier.h`; `simplifyGrid` stays as it is and is now implemented through `BlockSimplifier`):
  ```cpp
  // A piece's outline as the surface's vertex numbers (the samples the runs kept) with their grid coordinates, for
  // the walls and the base.
  struct PieceOutline {
    std::vector<std::uint32_t> vertices;  // counter-clockwise seen from above, closed, no repeat
    std::vector<GridPoint> points;        // the same vertices' (column, row) relative to the block's first column and row, within the predicates' bound
  };

  struct BlockResult {
    std::size_t firstTriangle = 0, triangles = 0;  // this block's triangles in surface()
    std::optional<Boundary> rectangle;             // set for a window block: its four sides, for solidFromSurface
    std::vector<PieceOutline> pieces;              // set for a footprint block, one per kept piece
  };

  // Simplifies a grid block by block into one surface in model units, Part A's engine per block (spec 4.4, 4.5),
  // so that blocks share the vertices their common sides keep. Construct it once per grid; call simplifyBlock for
  // each block in lattice order (row by row from the north-west); read surface() when done. A window block
  // (footprint nullptr) is simplified as a chunk of Part A. A footprint block is simplified piece by piece: each
  // piece's runs are simplified by the side cache in the fixed direction, the kept samples make the outline,
  // triangulatePolygon seeds the piece, legalisation and greedy insertion run over the samples strictly inside the
  // piece (never a sample on the outline), and the piece's outline edges are hull edges that no flip crosses.
  // Spec 4.3's guarantees hold inside every piece. Throws as simplifyGrid does, and std::invalid_argument when a
  // block exceeds kMostChunkCells a side or lies outside the grid.
  class BlockSimplifier {
   public:
    BlockSimplifier(const std::vector<float>& heights, int cols, int rows, double toleranceM, double widthM,
                    double heightM, double zScale, double scale, double levelM);
    [[nodiscard]] BlockResult simplifyBlock(BlockRect rect, const BlockFootprint* footprint);
    [[nodiscard]] const Mesh& surface() const noexcept;
    [[nodiscard]] Mesh takeSurface();  // moves the surface out
  };
  ```

- [ ] **Step 1: Write `tests/IslandFixture.h`**: the 24 x 18 mask of Task 2's test as `islandMask()` and `islandHeights()`, heights `20 + 2 i + 1.5 j` (metres) on data samples, with a 15 m knoll of radius 2 at the island's middle, NaN elsewhere, and `islandBox()` (any WGS 84 box); plus `islandGrid()` = the heights after `fillNearest`, as the simplifier needs them.
- [ ] **Step 2: Write the failing tests** in `tests/test_Simplifier.cpp` (tag `[simplify]`), with `checkGuarantees` extended to take an optional footprint: it checks the error at every sample strictly inside a piece, that every outline vertex is a sample on the piece's outline, and that no sample on the outline is a vertex unless a run kept it. Cases:
  - **Window blocks equal Part A**: a `BlockSimplifier` over the 64 x 48 fractal grid, blocks of 16 cells in lattice order with `footprint == nullptr`, gives a surface identical, vertex for vertex and triangle for triangle, to `simplifyGrid(..., chunkCells = 16)`.
  - **One footprint piece**: the island grid, one block over the whole grid with its `blockFootprint`: every piece's outline vertices are kept samples of its runs; `checkGuarantees` passes inside; the sea has no triangle (no triangle's centroid lies over a no-data cell); the lake is covered (its samples have a surface height within tolerance of the filled heights); the knoll's summit is within tolerance.
  - **Shared edges**: the island split into 2 x 2 blocks (12 x 9 cells): for each shared side, the two blocks' outline vertices on that line are the same vertex numbers in the same order (reversed between the two walks), and they equal the one-block run's kept samples where the run is common to both; cut-short blocks (a 2 x 2 lattice of 13 x 10 cells over the 24 x 18 grid) pass the same check (Review Focus 3).
  - **Corner contact**: the two touching blobs give two pieces whose outlines share exactly one vertex number.
  - **Determinism**: the island 2 x 2 twice gives identical surfaces.
  - **Refusals**: a block of 16,385 cells; a block outside the grid; a footprint piece given for a `rect` that does not contain it.
- [ ] **Step 3: Run** `./build/demdl_tests "[simplify]"`; expect the new cases to fail.
- [ ] **Step 4: Implement.** Keep `ChunkTin` for the rectangle; add a `PieceTin` that: converts the piece's loop to block-local `GridPoint`s; asks the side cache for each run's kept indices (a run along a row from west to east, along a column from north to south, reversed where the loop walks the other way); builds the outline; seeds with `triangulatePolygon`; marks every sample on the loop (kept or not) in a per-block bitmap of outline samples; legalises with the existing flips (outline edges have `kOutside` across them); runs the existing scan and insertion with the bitmap excluded; emits triangles counter-clockwise seen from above. `simplifyGrid` becomes a `BlockSimplifier` over the 2048-cell lattice with `footprint == nullptr`, which the first test pins. Memory stays per block: the bitmap and the piece's faces.
- [ ] **Step 5: Run** `./build/demdl_tests "[simplify],[solid]"`; expect all passed.
- [ ] **Step 6: Commit** `feat: the simplifier works block by block over a footprint's pieces`.

---

### Task 5: The footprint solid

Spec 5.3 (walls and base of a footprint block) and Review Focus 2.

**Files:**
- Create: `src/mesh/FootprintSolid.h`, `src/mesh/FootprintSolid.cpp`
- Test: `tests/test_FootprintSolid.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Mesh`, `checkSolid`, `downwardFacingTriangles` (`mesh/SolidMesh.h`); `PieceOutline` (Task 4); `triangulatePolygon` (Task 1).
- Produces:
  ```cpp
  // The printable solid of a block with footprint pieces: the surface's vertices and triangles first and unchanged,
  // then, piece by piece, a bottom copy of each outline vertex at z = planeZ, two wall triangles per outline edge
  // facing out of the piece, and the base, triangulatePolygon of the outline's points wound to face down. Each piece
  // is a closed shell; two pieces that touch at a vertex share it, which checkSolid accepts. Throws
  // std::invalid_argument when an outline has fewer than 3 vertices, names a vertex the surface lacks, or is not
  // counter-clockwise seen from above; when planeZ is not finite or a vertex is not above it; when the vertex count
  // would exceed 32-bit numbers.
  [[nodiscard]] Mesh solidFromPieces(const Mesh& surface, const std::vector<PieceOutline>& pieces, float planeZ);
  // surfaceTriangles + sum over pieces of (2 n + n - 2) for an outline of n vertices.
  [[nodiscard]] std::uint64_t solidTriangleCount(const std::vector<PieceOutline>& pieces,
                                                 std::uint64_t surfaceTriangles) noexcept;
  ```

- [ ] **Step 1: Write the failing tests** in `tests/test_FootprintSolid.cpp`, tag `[footsolid]`, building surfaces with `BlockSimplifier` over the island fixture:
  - one piece (the islet): the solid has `solidTriangleCount` triangles, `checkSolid` is ok, every wall triangle faces out (normal perpendicular to the wall's edge, pointing away from the piece's interior, tested with the centroid of the base), the base faces down, and the volume equals the sum over surface triangles of (mean height above planeZ x projected area) within 1e-6 relative;
  - the island with the lake: closed, the volume as above;
  - the two touching blobs: one solid of two shells sharing one vertex, `checkSolid` ok, `nonManifoldEdges == 0`;
  - a cut-short block's pieces (from the 13 x 10 lattice): closed;
  - refusals: an outline of two vertices; a clockwise outline; a vertex at planeZ.
- [ ] **Step 2: Build; expect** the missing-header errors.
- [ ] **Step 3: Implement.** Wall quads: for edge a->b of a counter-clockwise outline with bottom copies a', b', the triangles (a, b, b') and (a, b', a') face out; verify with the same `normalOf` rule `solidFromSurface` uses rather than assuming.
- [ ] **Step 4: Run** `./build/demdl_tests "[footsolid]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: closed solids for footprint pieces with wall quads and ear-clipped bases`.

---

### Task 6: The flags

Spec 5.1.

**Files:**
- Modify: `src/app/MeshCli.h`, `src/app/MeshCli.cpp`
- Test: `tests/test_MeshCli.cpp`

**Interfaces:**
- Produces in `MeshOptions`: `enum class Footprint { Window, Data }; Footprint footprint = Footprint::Window;`, `std::optional<double> tile;` (model units), `std::optional<std::string> blocks;` (the text, parsed by `parseBlocks` in main so the message names the flag), `double minPiece = 1;`.
- Options after `--tolerance`, in this order, help texts verbatim:
  - `--footprint window|data`: `where the solid exists: window, the whole window as today; data, only where the DEM has data, with a wall along the data's edge (default window)`;
  - `--tile SIZE`: `split the window into square blocks of SIZE model units, each written as STEM_rRR_cCC.stl, above 0; absent, one solid is written`;
  - `--blocks ROWS:COLS`: `write only these blocks, each part a number or a range from 1, such as 3-5:2-4 or 7:12; the others are listed`;
  - `--min-piece A`: `drop a piece of footprint smaller than A square model units and list it, 0 or more (default 1)`.
- `validateMeshOptions`, after tolerance: `tile must be finite`, `tile must be positive`; `min-piece must be finite`, `min-piece must be zero or positive`; `--blocks requires --tile`; `--footprint` text other than `window` or `data` is a CLI11 usage error (use `CLI::IsMember`).

- [ ] **Step 1: Write the failing tests** (tag `[meshcli]`): each flag parses; defaults; the four help texts; validation messages; `--footprint sea` is a usage error naming the two values; `--blocks 3-5:2-4` without `--tile` fails validation.
- [ ] **Step 2: Run** `./build/demdl_tests "[meshcli]"`; expect failures.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `./build/demdl_tests "[meshcli]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: dem2mesh --footprint, --tile, --blocks and --min-piece options`.

---

### Task 7: Blocks in dem2mesh

Spec 5.2 to 5.5, 6, 7 (Part B), deviations 1 to 4, Review Focus 1 to 5.

**Files:**
- Modify: `src/app/dem2mesh_main.cpp`
- Test: `tests/test_Dem2MeshEndToEnd.cpp` (new cases on an island GeoTIFF written from `tests/IslandFixture.h` through `writeGeographicFixture`, no-data as `DemRaster::kNoData`)

**Interfaces:**
- Consumes: Tasks 2 to 6; `writeBinaryStl`, `writeBinaryPly`, `solidFromSurface`, `checkSolid`, `downwardFacingTriangles`, `BlenderJob` as it is (Task 8 extends it).
- Produces, in `main`:
  - **Outputs.** Without `--tile`: `STEM.stl`, `STEM.ply`, `STEM.blend` as today. With `--tile`: `STEM_<blockName>.stl` per written block, plus `STEM.ply` and `STEM.blend`. Without `--overwrite`, every lattice file that exists is refused before reading (deviation 4); the lattice is known from the grid, before reading.
  - **Pipeline.** `parseBlocks(*options.blocks)` first, its message reported as `--blocks: <message>` (exit 1). Read, fill, resample as today; with `--footprint data`: `dataMaskOf` on the window before the fill, `resampleMaskNearest` to the grid, `footprintCells`; the base plane from `f.lowest`/`f.highest` as today (the fill copies data heights, so the range is the data's). The lattice: `tileLayout` with `--tile`, else one block of the whole grid, refused above 16,384 a side only with `--footprint data` (deviation 2; `--footprint window` without `--tile` runs today's path unchanged). Then for each block in lattice order: `blockFootprint` (data) or none (window); `simplifyBlock` when the block is selected and has footprint (or is a window block); the block's local mesh (its triangle range, vertices gathered and renumbered, x and y shifted by the block's south-west corner, which is `firstColumn x widthM / (cols - 1) x scale` and `(rows - 1 - lastRow) x heightM / (rows - 1) x scale`); `checkSurface`; the solid: `solidFromSurface` with the rectangle for a window block, `solidFromPieces` for a footprint block; `checkSolid` and the count check; `writeBinaryStl` with a header `dem2mesh <version> block rRR_cCC grid WxH window ...` (the existing header with the block name added before `grid`). Without `--tile` and with `--footprint data`, the one block writes `STEM.stl`.
  - **Whole area.** After the loop, `takeSurface()` is the PLY surface; `f.surfaceLowZ/HighZ`, `countOutputs` and the warning use it; `writeBinaryPly` as today.
  - **Lines.** After `base:`, with `--tile`: `tiles: 13 x 20 blocks of 1000 x 1000 cells (5000 x 5000 m, 100 x 100 units); 146 with land, 114 without, 32 cut short at the east or south edge` (the counts of written, empty and cut-short blocks; "with land" counts blocks that have a kept piece, or every block for `--footprint window`). Then one line per written block, `block r03 c05: 72% land, 184220 triangles, 9.2 MB, cape-town_r03_c05.stl` (percent = kept footprint cells over the block's cells, rounded; 100% for window blocks; the file's size to one decimal MB). Then, when any: `skipped: no land in r01 c05, r01 c06, ...`; `not selected: r02 c01, ...` (outside `--blocks`); `dropped pieces: r05 c02 2 cells 0.5 units2, ...` (cells and area in square model units to one decimal); `corner contacts: N` (when above 0, over all written blocks); `stale: cape-town_r01_c05.stl, ...` (existing lattice files this run did not write, `--overwrite` only). With `--footprint data` and no `--tile`, the `block` line is replaced by `footprint: 72% of the window, 2 pieces` and `dropped pieces:`/`corner contacts:` as above. The `solid:` line gives the total of all written solids' triangles and the sum of their STL bytes with the PLY's. The `wrote:` lines name every STL in lattice order, then the PLY. The dry run prints the same lines in its plan with `would write:` in place of `wrote:` and no `output:` lines for blocks beyond the PLY and `.blend`.
  - **Blender.** With `--tile`, `makeScene` passes every written STL (Task 8's `stls` list); until Task 8 lands, the Blender stage with `--tile` throws `Blender's check of several blocks comes in Task 8` so that Task 7's end-to-end tests run with `--no-blend`; Task 8 removes that line.

- [ ] **Step 1: Write the failing end-to-end tests** (tag `[mesh-e2e]`), each on the island fixture with `--no-blend`, `--scale 1` and `--tolerance 0.5`, and `--tile` equal to 12 times the x cell size in metres from `wholeFileExtent`, so that blocks are 12 cells wide and `round(12 cellX / cellY)` cells tall; the test derives every block expectation from `tileLayout` and `blockRect` rather than hard-coding them:
  - **"a footprint without tiles ends at the coast"**: `--footprint data`: one STL, closed, `downwardFacingTriangles == 0`, the sea point `topAt` is NaN (no surface), the lake point has a surface, the knoll's summit within 0.5 m; stdout has `footprint: `, `dropped pieces: ` (the lone sample makes no cell, so no drop: assert the line is absent) and no `tiles:`.
  - **"tiles write one closed block per piece of land and match along shared edges"**: `--footprint data --tile <12 cells>`: files for the blocks with land only, each read back closed with `solidTriangleCount` triangles; for every pair of neighbouring written blocks, the vertices on their shared side (found by position after undoing the local frames) agree in position within 1e-3 units and in z exactly; the `tiles:` line's counts; `skipped:` names the sea-only blocks; the PLY's vertex count equals the union's (the whole-area surface has no duplicate position); `wrote:` lists the STLs in lattice order then the PLY.
  - **"a cut-short lattice still closes and matches"** (Review Focus 3): `--tile` of 13 cells over the 24 x 18 grid: the same checks.
  - **"a speck under --min-piece is dropped and the block skipped"** (Review Focus 1): the islet with `--min-piece` above its area: its block is not written, `dropped pieces:` names it with its cells, `skipped:` names the block.
  - **"pieces touching at a corner are two shells in one block"** (Review Focus 2): the blobs' block: one file, `checkSolid` ok, `corner contacts: 1`.
  - **"a file without no-data gives the whole window as one piece"** (Review Focus 4): the plane fixture of the existing cases with `--footprint data`: closed, the same `simplify:` counts as `--footprint window`, the `footprint: 100% of the window, 1 piece` line.
  - **"--blocks writes a selection and lists the rest"**: `--blocks 1-1:1-2`: only those files; `not selected:` names the others.
  - **"stale block files are left alone and named"** (Review Focus 5): run the 2 x 2 tiles; then run again with `--overwrite --min-piece <above the islet>`: the islet's old file still exists unchanged and `stale:` names it; without `--overwrite` the second run is refused naming the first existing block file, before reading (stdout empty).
  - **"the dry run plans tiles without writing"**: `--dry-run` prints `tiles:`, the `block` lines and `would write:` lines; the directory stays empty.
  - **"window tiles are full squares"**: `--footprint window --tile <12 cells>`: four files, each closed with the rectangle's count, `100% land`.
- [ ] **Step 2: Run** `./build/demdl_tests "[mesh-e2e]~[slow]"`; expect the new cases to fail.
- [ ] **Step 3: Implement**, keeping `run()` readable: a `BlockPlan` built before reading (layout, names, files, selection), a `writeBlocks` function for the loop, and the line builders beside the existing ones.
- [ ] **Step 4: Run** `./build/demdl_tests "[mesh-e2e]~[slow]"`, then the full `ctest --test-dir build --output-on-failure`; expect all passed, the existing cases untouched.
- [ ] **Step 5: Manual check**, outputs under the scratch directory, inputs read-only: the Table Mountain crop with `--footprint data --tolerance 0.05 --scale 0.02 --no-blend` (the coast ends the model); the r3_c1 tile with `--footprint data --tile 100 --tolerance 0.05 --scale 0.02 --no-blend` in the Release tree (`cmake --build build-release`): note the `tiles:` line, the block count and the largest block's size.
- [ ] **Step 6: Commit** `feat: dem2mesh writes footprint blocks as puzzle tiles`.

---

### Task 8: Blender checks every block

Spec 5.4.

**Files:**
- Modify: `src/mesh/BlenderRunner.h`, `src/mesh/BlenderRunner.cpp`, `src/mesh/blender_scene.py`, `src/app/dem2mesh_main.cpp` (the Task 7 placeholder), `tests/FakeBlender.h`
- Test: `tests/test_BlenderRunner.cpp`, `tests/test_Dem2MeshEndToEnd.cpp`

**Interfaces:**
- `BlenderJob::stl` becomes `std::vector<std::filesystem::path> stls;` (one or more). `runBlender` passes `--stl=<path>` for one and, for several, writes them one absolute path per line to a second temporary file beside the script (`dem2mesh-stls-XXXXXX.txt`, removed with it) and passes `--stl-list=<file>`.
- `blender_scene.py`: `--stl` and `--stl-list` are mutually exclusive, one required. With a list, after saving the scene it checks each file in turn as `check_stl` does today and prints one line, `stl check: 146 files, 12345678 triangles, 0 non-manifold edges, 0 inverted faces` (totals; plain integers), failing on the first bad file with its own `stl check:` line for that file preceded by `stl check: <path> failed`; the single-file form prints today's line unchanged.
- `dem2mesh` repeats the last `stl check:` line as today; `makeScene` passes every written block.
- `tests/FakeBlender.h`'s script accepts `--stl-list` and prints the totals line.

- [ ] **Step 1: Write the failing tests**: `[blender]`: `runBlender` with three STLs passes `--stl-list=` to a file holding the three absolute paths and removes it afterwards (the existing argument-capturing fake); the real-Blender case (`[blender][slow]`) checks two small solids from one list and the totals line; `[mesh-e2e]`: the island tiles with the fake Blender print the totals line and `wrote: STEM.blend`; with the real Blender (`[mesh-e2e][slow]`) the same run passes its check.
- [ ] **Step 2: Run** `./build/demdl_tests "[blender]~[slow],[mesh-e2e]~[slow]"`; expect failures.
- [ ] **Step 3: Implement**; remove the Task 7 placeholder.
- [ ] **Step 4: Run** the two tags, then the full `ctest`; expect all passed (the real-Blender cases take about 30 s).
- [ ] **Step 5: Commit** `feat: Blender checks every block's STL in one session`.

---

### Task 9: Documentation and the puzzle run

Spec 5.5, 7 (Part B), 8, 9; CLAUDE.md.

**Files:**
- Modify: `CLAUDE.md`, `docs/superpowers/specs/2026-10-21-simplify-and-tiles-design.md`, `docs/superpowers/plans/2026-10-22-footprints-and-tiles.md` (a follow-ups section)

- [ ] **Step 1: The puzzle run**, Release tree, outputs under the scratch directory: `/usr/bin/time -l ./build-release/dem2mesh -i ~/Downloads/CCT_GLM2019_5m_DEM_1D_v1.tif -o <scratch>/cape-town --scale 0.02 --tolerance 0.05 --footprint data --tile 100`, Blender included, in the background. Record: wall time, maximum resident set size, the `tiles:` line, the number of files, the largest and the total STL size, `corner contacts:`, `dropped pieces:`, and Blender's `stl check:` totals. Then open nothing; instead read two neighbouring blocks' STLs back with the end-to-end test's shared-edge check as a scratch program and record the largest mismatch.
- [ ] **Step 2: Write** CLAUDE.md's `dem2mesh` section: a **Footprints and tiles** bullet (the flags; the cell, closing, enclosed and piece rules in the section's voice; shared edges; the base plane; the local frame; the file names; the lines; `--blocks`; `stale:`; the whole-City figures and the Release advice; what the dry run costs; the one-block limit of deviation 2; the service upload limits the owner checks); the Layout entries for `PolygonTriangulation`, `Footprint`, `Tiles`, `FootprintSolid`, `BlockSimplifier` and the Blender list; the test tags `[polygon]`, `[footprint]`, `[tiles]`, `[footsolid]`; the Part B sentence at the top of the section and line 9 updated from "designed and not built". The spec: the status line (Part B implemented on the date, through this plan); section 9 gains deviations 1 to 5. This plan gains `## Follow-ups deferred at the final review` with whatever the final review leaves, plus the Part A follow-ups that Part B touched (side cache eviction, the `ChunkTin` split) marked done or still open.
- [ ] **Step 3: Commit** `docs: CLAUDE.md and spec for dem2mesh footprints and tiles`.

---

## Self-review notes

- Spec coverage: 5.1 (Task 6; `--blocks` text parsed in Task 3), 5.2 (Tasks 2 and 4; the mask through resampling in Task 2; the outline on grid lines in Task 4), 5.3 (Tasks 3, 4, 5, 7: lattice, shared edges, base plane, walls and base, not written), 5.4 (Tasks 7 and 8), 5.5 (Task 9's run), 6 (Task 7's untouched existing cases), 7 Part B (Task 7's island cases and Task 9's City run), 8 (the files above), 9 (deviations in Task 9).
- Type consistency: `BlockRect` is defined in Task 2 and consumed by Tasks 3, 4 and 7; `Piece`/`BlockFootprint` by Tasks 4 and 7; `PieceOutline`/`BlockResult`/`BlockSimplifier` (Task 4) by Tasks 5 and 7; `TileLayout`/`BlockSelection` (Task 3) by Task 7; `MeshOptions` fields (Task 6) by Task 7; `BlenderJob::stls` (Task 8) by Task 7's placeholder removal.
- Review Focus 1 to 5 are pinned in Task 7 (with Tasks 4 and 5 for 2 and 3).
- Proportion: signatures, test names and assertions, and three algorithm notes (the ear test, the outline walk, the wall winding) that the signatures do not determine; no function bodies.

## Follow-ups deferred at the final review (2026-10-22)

The minors that the task reviews left open, one line each, task by task, as the ledger holds them (line numbers are those the reviewer saw); none changes what `dem2mesh` writes. The whole-branch review's own additions are appended below by the controller.

Task 1, `PolygonTriangulation`:

- Add to the random family a square with many lattice points a side and a scaled sparse-vertex variant.
- No committed test shows `firstProblem` rejecting a bad triangulation.
- `PolygonTriangulation.h:42-44`, "for such a polygon the set can miss an ear", reads as "simple" where it means self-crossing.
- The first ear fill costs about 2x on shapes where the old search found an ear at once (within the stated bound).
- Outcomes on self-crossing input changed with the ear set (unspecified either way).

Task 2, `Footprint`:

- The random run tests re-derive the unit's own rule: add the shared-stretch check or soften the claim at `test_Footprint.cpp:34-36`.
- The header should say that `Piece::loop` is in window coordinates, which can exceed the 16,384 predicate bound (`BlockSimplifier` converts to block-local first).
- `blockFootprint` is about 117 lines.
- Pin a one-cell block, a one-cell piece and an all-out block.

Task 3, `Tiles`:

- Parameters named `blockCells` shadow the function `blockCells` (`Tiles.cpp:49`, `:72`).
- Two header sentences are imprecise: `Tiles.h:10-11`, "apart from one division", and `:30`, "under half a cell".
- The local `quoted` duplicates `CliChecks.h`'s: hoist it into `src/demdl/`.
- The test helper `tilingProblem` would hang on a zero block side: add a guard.
- No named 2 x 2 grid case, and argument-fault precedence is tested only for the width before the height.

Task 4, `BlockSimplifier`:

- The header does not forbid mixing window and footprint blocks in one `BlockSimplifier`; a mixed seam would T-junction silently (`dem2mesh` never mixes).
- The "window blocks equal `simplifyGrid`" test can no longer fail: commit a golden fingerprint of `simplifyGrid` on the fractal.
- The side cache keeps the interior coast runs that no other block uses.
- `BlockSimplifier` is not movable: default the moves (`dem2mesh` constructs it in place).
- No test pins the four-edge re-test in `legaliseAll` (mutant M6 survives).
- Two checks run on a moved-from vector.
- `Simplifier.cpp` is 1063 lines and `test_Simplifier.cpp` 2261: the footprint tests could move to their own file.
- `tracked` is not asserted to be at most 4.

Task 5, `FootprintSolid`:

- `FootprintSolid.h:77-78` says a crossing outline is refused; it is refused only when no ear is left, and the caller must pass a simple polygon.
- Wrapped messages quote the negated-row frame.
- `FootprintSolid.cpp` duplicates `SolidMesh.cpp`'s `formatNumber` and plane checks verbatim.
- The base re-clips the outline the simplifier clipped for its seed: carry the seed's triples in `PieceOutline` if coastal blocks ever dominate (on the City, building, checking and writing the 146 solids took 3.9 s of the 57 s before Blender starts, so they do not).
- No 3-vertex solid case.

The corner-contact join (the Task 2 fix round):

- A stale comment at `test_Simplifier.cpp:1009`: "touch at a vertex all turn up in the footprint".
- The multi-round join path is unreachable through `footprintCells` and untested.
- No test pins that `*joined` is appended to.
- `Simplifier.cpp:81`, `:275`, `:654` and `:995` still describe touching pieces, now unreachable from `footprintCells`.

Task 6, the flags:

- The comment at `MeshCli.cpp:75-77` says `parseBlocks` puts the flag in front; main does.
- The empty-text `--blocks` message differs in wording from `parseBlocks`'s form message.
- The footprint usage-error test does not pin the echoed value, and the "alone and together" case asserts no values.

Task 7, blocks in `dem2mesh`:

- No suite case runs at a scale other than 1, so a lost `* options.scale` in the local frame, the `tiles:` units or `cellAreaUnits2` would pass: add one at `--scale 0.5`.
- "Refused before anything is read" is asserted only through an empty stdout: add `--verbose` and keep `out.empty()`.
- The one-block layout is a hand-built aggregate: use designated initialisers or a `wholeGridLayout` helper.
- First-pass errors carry no block label.
- The tiled dry run names the PLY twice (`would write:` and `output:`), and a `--footprint data` dry run without `--tile` is uncovered.
- A test asserts on its own oracle (`test_Dem2MeshEndToEnd.cpp:1628`, where the library's `solidTriangles` is the expectation).
- `0% land` can print for a tiny kept piece (the City's r02 c06, r12 c13 and r18 c10 do).
- The grid refusal says "with --tolerance" for a Part B run without it.
- `dem2mesh_main.cpp` is 1174 lines with pure mesh work (`blockSurface`, `localNumber`, `renumbered`) that belongs in `src/mesh/` with unit tests at a scale other than 1, and its summary and plan builders could move to their own file; `test_Dem2MeshEndToEnd.cpp` is 1830 lines.

Task 8, Blender over every block:

- `makeScene` repeats the first `stl check:` line, which is correct while a success prints exactly one: take the last, or comment the invariant.
- No `dem2mesh`-level test of a failing tiled Blender run.
- The script's usage errors and its list validation are untested in the suite.
- `solid` in `BlenderRunner.cpp:256` should be `stlArgument`.
- A one-entry list would print "1 files" (never produced).
- A path with a newline in it cannot be listed (it fails loudly, unguarded).
- `blenderMemory` sums every block's triangles where spec 5.4 says the whole-area surface; it is conservative (the City's dry run says about 1.2 GB), and the City run did not measure Blender's own peak.

The Part A follow-ups that Part B touched (from the list of `2026-10-21-simplify.md`):

- The side cache and the vertex map are still never evicted. Still open, and negligible at 1000-cell blocks: the City run, which keeps 662,226 vertices and 146 blocks' runs, peaked at 2.29 GB in all.
- `ChunkTin` was split in Task 4 as far as Part B needed: `Tin` is the shared engine (faces, flips, scan, heap, emission), the number of tracked corners a parameter (4 for a chunk, 0 for a piece) and the outline's samples a bitmap; `ChunkTin` keeps the rectangle's seeding and the new `PieceTin` seeds from `triangulatePolygon`. Done; `Tin` itself still keeps topology, scan and heap together.
- A chunk of 16,384 cells at a near-zero tolerance needs about 30 GB of working state. Still open, and Part B now allows such blocks: a block side may be 16,384 cells, and `--tile` or `--footprint data` without `--tolerance` simplifies at 0. Bound a block's memory before it starts.
- `formatNumber` now exists in eight files, not five (Part B added `Footprint.cpp`, `FootprintSolid.cpp` and `Tiles.cpp`), and `FootprintSolid.cpp` repeats `SolidMesh.cpp`'s frame and plane checks. Still open.
- The spec-citation convention is settled only in the header of `dem2mesh_main.cpp` ("Part B 5.x"); the library headers say "spec 5.x" for the same document.
