# Scene Construction — OBJ Loading and Procedural Meshes

**Milestone:** Phase 2a
**Documented at:** working tree past commit `4a29b52`, 2026-09-21 — nothing under `include/scene/`
or `src/scene/` is committed yet.
**Scope:** `include/scene/obj_loader.hpp`, `src/scene/obj_loader.cpp`,
`include/scene/procedural.hpp`, `src/scene/procedural.cpp`.

Line citations are against the current working tree, not a commit hash, since this code has not
been committed. Nothing here is a benchmark number; any performance-shaped statement is `UNVERIFIED`
unless it cites `benchmarks/results/`, which currently contains no scene-generation results.

## 1. What problem it solves

The BVH and the intersection core (`docs/geometry.md`, `docs/bvh.md`) need meshes to operate on.
This subsystem supplies two independent sources: **real-world geometry** via a Wavefront OBJ parser
(`obj_loader.*`), and **synthetic geometry generated in code** via three shape generators
(`procedural.*`). Both funnel into the same `geom::Mesh` constructor, so both go through the same
validation (non-finite rejection, index-range checking — `docs/geometry.md`'s Phase 2a addendum).

## 2. Why it exists

`include/scene/procedural.hpp:11-13` states the reason directly: there are no `.obj` assets in the
tree yet, and a hand-built 12-triangle cube cannot exercise leaf size or depth — the BVH build and
traversal need meshes at a range of triangle counts and with different spatial structure (curved and
closed, flat and degenerate-axis, spatially incoherent) to be tested and benchmarked meaningfully.
The OBJ loader exists because real assets are the point of an interactive explorer — procedural
shapes are a testing and benchmarking convenience, not the intended content.

## 3. How it works

### OBJ loader (`src/scene/obj_loader.cpp`)

A single-pass, hand-rolled line-oriented parser — no external OBJ library, consistent with
`CLAUDE.md`'s "no external libraries for core algorithms" rule extended here to scene ingestion.
It walks the input by `\n`-delimited lines without copying them into `std::string` (`text.find('\n',
pos)` over a `string_view`, `:104-107`), recognizes `v` and `f` directives, fan-triangulates any
polygon with more than 3 vertices, and accumulates positions/indices into flat vectors that are
handed to the `Mesh` constructor once parsing finishes (`:163`).

- **`v x y z [w]`**: three (or four, the fourth silently unread) scalars parsed with `std::strtof`
  (`:30-40`), not `std::from_chars` — the comment at `:28-29` states this toolchain's C++ standard
  library is missing the floating-point overload (see `CLAUDE.md`'s Toolchain status section: Apple
  clang 11.0.3 lacks C++20 `<charconv>` float support).
- **`f ...`**: each face-vertex token is `v`, `v/vt`, `v//vn`, or `v/vt/vn`
  (`parseFaceIndex`, `:43-54`) — only the leading integer (the position index) is kept; texture and
  normal indices are scanned past and discarded, because this project wants geometry, not shading.
  Indices are 1-based per the OBJ spec and may be negative, meaning "relative to the vertex count so
  far" (`resolveIndex`, `:57-78`) — resolved *at parse time*, against `positions.size()` as it stood
  on that line, so a negative index means what it meant when the exporter wrote it, not what it would
  mean after the whole file is read.
- **Triangulation is fan-from-vertex-0** (`:138-143`): for an n-gon `(v0, v1, ..., vn-1)`, it emits
  triangles `(v0, v1, v2), (v0, v2, v3), ..., (v0, vn-2, vn-1)`. Correct for convex, planar faces,
  which is what mesh exporters emit; the comment at `:136-137` is explicit that a concave n-gon would
  need ear clipping instead, and this loader does not attempt it.
- **Skipped directives**: `vt`, `vn`, `vp`, `o`, `g`, `s`, `usemtl`, `mtllib`, comments, and blank
  lines are all counted (`ObjLoadStats::skippedLines`) but otherwise ignored (`:149-152`).
- **Rejected rather than skipped**: free-form geometry directives (`curv`, `curv2`, `surf`, `deg`)
  throw instead of being silently dropped (`:144-148`). The reasoning is explicit in the source: a
  skip would make the resulting triangle count silently wrong with no way to detect it later, whereas
  a throw surfaces the gap immediately at load time.
- **`ObjLoadStats`** (`obj_loader.hpp:13-20`) records positions, faces, triangles,
  `polygonsTriangulated` (faces that needed fan triangulation), `degenerateTriangles` (computed via
  `Mesh::countDegenerateTriangles()` after construction, `obj_loader.cpp:164`), and `skippedLines` —
  benchmark provenance, per the header comment: a triangle count is only reproducible alongside how
  many triangles came from triangulation versus being authored directly, and how many are degenerate.

### Procedural generators (`src/scene/procedural.cpp`)

Three generators, each returning a `geom::Mesh` built the same way the OBJ loader does — flat
position/index vectors passed to the `Mesh` constructor, so the same finite-vertex and index-range
validation applies uniformly.

- **`uvSphere(targetTriangles, radius)`** (`:34-82`): a closed, indexed UV sphere. Ring count `k` is
  derived from the target triangle count (`2*segments*(rings-1)` triangles, with
  `segments ≈ 2*(rings-1)` chosen to keep quads roughly square, `:36-38`), vertices are shared band-
  to-band, and the poles are single vertices fanned to the first/last ring rather than degenerate
  quads. Header comment (`procedural.hpp:16`): curved and hollow, so most rays either miss the whole
  mesh or cross exactly two surfaces far apart — the adversarial case this shape exercises is the
  **shared-vertex tie**, where dozens of triangles meet at one point (the pole) and each computes a
  slightly different floating-point `t` for the same physical hit. This is exactly the geometry
  `docs/bvh.md`'s `kBestPruneSlack` measurement table is built from (`tests/test_bvh.cpp:927-1024`,
  `poleRays`).
- **`grid(targetTriangles, extent)`** (`:84-111`): a flat, tessellated square in the xz-plane, built
  from a regular `verts × verts` vertex lattice with two triangles per cell. Header comment
  (`procedural.hpp:20`): zero extent along y by construction — the degenerate-axis case for any BVH
  split heuristic that picks "the longest axis," since the geometric y-extent is always exactly 0 and
  `AABB::longestAxis()` (`include/geometry/aabb.hpp:81`) can never legitimately choose it for this
  mesh's root or any node whose bounds still span both x and z.
- **`triangleSoup(triangleCount, seed, extent, triangleSize)`** (`:113-134`): `triangleCount`
  unconnected small triangles at random positions and orientations, no shared vertices at all.
  Header comment (`procedural.hpp:24`): no spatial coherence between neighbouring index-array entries,
  which is the adversarial case for a **median** split specifically — object/centroid median assumes
  index-order is not meaningfully clustered, and this generator guarantees that by construction
  (contrast with a mesh loaded from a real asset, where nearby triangles are often nearby in the
  index buffer because of how modelling tools export faces).
- **Deterministic PRNG mapping** (`:14-30`): `std::mt19937` is used because it is specified
  bit-for-bit and reproducible across standard library implementations, while
  `std::uniform_real_distribution` is explicitly *not* guaranteed reproducible across
  implementations (only its API is standardized, not its algorithm) — so the `[0,1)` mapping is done
  by hand: `unitFloat` takes the top 24 bits of a 32-bit `mt19937` output and scales by `1/2^24`
  (`:26-28`). The comment explains why not all 32 bits: `float`'s mantissa is 24 bits (23 explicit +
  implicit leading 1), and scaling the full 32-bit range would round `2^32-1` up to exactly `2^32` on
  conversion to `float`, producing `1.0` — outside the claimed `[0,1)` range. This is the same
  IEEE-754-awareness discipline as the geometry core's `gamma()`/`kEpsilon` (`docs/geometry.md` §9).

## 4. Data structures used

`geom::Mesh` (positions + indices, see `docs/geometry.md` §4) is the only structure produced.
`ObjLoadStats` is a plain aggregate of `std::size_t` counters. The OBJ parser's only transient
structure is a reused `std::vector<std::size_t> faceVertices` (`obj_loader.cpp:99`), cleared and
refilled per face rather than reallocated, so an n-gon does not allocate on every line.

## 5. Algorithm used

OBJ: single-pass recursive-descent-free line scanner with fan triangulation (a fixed, non-recursive
loop, `:138-143`) — not a general polygon triangulator. Procedural: closed-form parametric generation
(UV sphere via spherical coordinates, grid via a regular lattice) plus PRNG sampling for the soup —
no simulation or iterative refinement in any of the three.

## 6. Time complexity

OBJ loading: O(file size) for the scan plus O(vertices) for finiteness/index validation inside
`Mesh`'s constructor (`docs/geometry.md`'s Phase 2a addendum) — one pass, no backtracking, since
negative/forward-reference indices are resolved and checked against `positions.size()` at the line
that uses them (`:126`, `resolveIndex` throwing immediately on an out-of-range or forward
reference — see `RejectsAForwardReference`, `tests/test_obj_loader.cpp:162`).
Procedural: `uvSphere` and `grid` are O(target triangle count) — each vertex and index is written
exactly once, no loop revisits work. `triangleSoup` is O(triangleCount).

## 7. Space complexity

O(vertices + indices) for all four generators/loader, matching the produced mesh's own storage — no
generator holds an auxiliary structure larger than its output (the OBJ parser's `faceVertices` scratch
vector is O(largest face), not O(mesh size)).

## 8. Important invariants

- Every mesh produced by this subsystem satisfies `Mesh`'s own invariants (`docs/geometry.md` §8):
  `indices.size() % 3 == 0`, every index in range, every vertex finite — enforced by the shared
  `Mesh` constructor, not reimplemented here.
- OBJ: an index is validated against `positions.size()` **as of the line it appears on**, which
  means a face may reference any vertex defined earlier in the file (including by negative index)
  but never one defined later — `resolveIndex` throws on both out-of-range and forward references
  (`:60-78`), the latter case tested explicitly (`tests/test_obj_loader.cpp:162`,
  `RejectsAForwardReference`).
- Procedural: `triangleSoup`'s output count is exact (`SoupHasExactlyTheRequestedTriangleCount`,
  `tests/test_procedural.cpp:57`); `uvSphere`/`grid`'s triangle counts are *targets*, not exact
  values — the header comment says so directly (`procedural.hpp:13`, "Triangle counts are targets —
  ask the returned Mesh for the real count") because ring/segment counts must be integers, so the
  actual count is whatever falls out of rounding `k = isqrtRound(...)`
  (`tests/test_procedural.cpp:23`, `SphereHasRoughlyTheRequestedTriangleCount`, uses a tolerance).

## 9. Numerical assumptions

- OBJ float parsing goes through `std::strtof` into `Scalar` (= `float`), so file precision beyond
  `float` is lost on load — consistent with the geometry core's blanket choice of `float` for
  cache-density reasons (`docs/geometry.md` §9).
- `uvSphere`'s ring/segment derivation (`isqrtRound`, `:14-18`) rounds a `double` square root to the
  nearest `std::uint32_t` — this is shape planning arithmetic, not geometry, so `double` here is fine
  and does not touch the `Scalar` = `float` policy.
- `triangleSoup`'s PRNG-to-float mapping is deliberately hand-rolled rather than using
  `<random>`'s distribution machinery, specifically so a given `seed` reproduces the same mesh
  bit-for-bit across platforms and standard library implementations
  (`tests/test_procedural.cpp:63`, `SoupIsDeterministicForAGivenSeed`) — a property
  `std::uniform_real_distribution` does not guarantee.

## 10. Performance considerations

`UNVERIFIED` — no file under `benchmarks/results/` measures OBJ parse throughput or procedural
generation time. The single-pass, no-copy (`string_view`-based) design of the OBJ scanner and the
closed-form (no iteration-to-convergence) procedural generators are structural choices aimed at
keeping load/generation cost well below the BVH build and query costs they feed, but no number in
this repository currently supports a specific throughput claim.

## 11. Alternative approaches

- **A real OBJ/glTF library** (tinyobjloader, assimp) — rejected by the same project-wide rule that
  keeps geometry math in-house (`CLAUDE.md`): the loader is simple enough to own and explain, and a
  hand-rolled parser is what makes "why does a forward reference throw" an answerable interview
  question rather than "that's library behavior."
- **`std::from_chars` for float parsing** — the natural C++17/20 choice, not available: this
  toolchain's standard library is missing the floating-point overload (`obj_loader.cpp:28-29`,
  cross-referenced in `CLAUDE.md`'s Toolchain status). `strtof` was the fallback, not the first
  choice.
- **Ear-clipping triangulation for n-gons** — not implemented; fan triangulation is correct only for
  convex faces. Accepted because mesh exporters emit convex polygons in practice and the added
  complexity is not exercised by any asset in this project. `UNKNOWN — not yet implemented` if a
  concave n-gon is ever required.
- **`std::uniform_real_distribution` for `triangleSoup`** — rejected specifically because it is not
  guaranteed bit-reproducible across implementations, which would break `SoupIsDeterministicForAGivenSeed`
  as a portable property.

## 12. Why the chosen approach was selected

Simplicity and explainability over generality, per `CLAUDE.md`'s "simple and defensible beats
clever": a line-oriented scanner that only reads what this project actually needs (positions and
triangulated faces, no materials/normals/UVs) is easy to reason about end to end, and every rejection
path (forward reference, index 0, free-form geometry, non-finite vertex) is a deliberate, tested
decision rather than an artifact of a general-purpose library's behavior.

## 13. Known limitations

- No texture coordinates, normals, materials, or groups are retained — this project wants geometry
  only (stated directly, `obj_loader.hpp:26`).
- Concave n-gons triangulate incorrectly (fan-from-vertex-0 produces self-intersecting or wrong-area
  triangles for a non-convex face) — `UNKNOWN — not yet implemented` fix.
- `uvSphere`/`grid` triangle counts are approximate by design; a caller needing an exact count must
  read it back from the returned `Mesh`.
- No streaming/incremental parse — the whole file is read into memory (`loadObj`,
  `obj_loader.cpp:170-179`) before parsing begins.

## 14. Edge cases

Covered by `tests/test_obj_loader.cpp` (336 lines) and `tests/test_procedural.cpp` (78 lines):
CRLF line endings (`:121`), a final line with no trailing newline (`:128`), extra whitespace
(`:133`), the fourth (`w`) vertex component present but unused (`:138`), an empty file producing an
empty mesh (`:201`), vertices defined but never referenced by any face — orphans, not triangles
(`:207`), index 0 (invalid — OBJ is 1-based, `:169`), a negative index reaching before the start of
the vertices defined so far (`:173`), non-finite coordinates (`:326`, routed through `Mesh`'s own
rejection), and a full round-trip of a generated mesh back through the file format (`:279`).
Procedural: sphere points verified to lie exactly on the sphere (`:30`), sphere closure and
non-degeneracy (`:40`), grid flatness in y (`:47`), and every generated vertex checked finite
(`:72`).

## 15. Testing strategy

Example-based tests dominate (readability over property testing here, unlike the intersection
core), because parser correctness is best pinned case-by-case against the OBJ spec's actual grammar
quirks (index forms, negative indices, free-form rejection) rather than generalized. The procedural
generators get a handful of targeted property checks (points lie on the sphere within tolerance,
grid is flat, soup is deterministic and exact-count) rather than full randomized property testing,
consistent with their role as test/benchmark fixtures rather than a subsystem with its own complex
invariants to defend.

## 16. How it interacts with other components

Both paths terminate at `geom::Mesh`'s constructor (`docs/geometry.md`), which is the single point
where non-finite vertices and out-of-range indices are rejected — this subsystem does not duplicate
that validation, it relies on it. `bvh::BVH::build` (`docs/bvh.md`) consumes whichever `Mesh` this
subsystem produces without needing to know whether it came from a file or from code. The pole-ray
edge case that drives `docs/bvh.md`'s `kBestPruneSlack` measurement is generated specifically by
`uvSphere`, and the flat-axis case that guards `AABB::longestAxis()`'s centroid-bounds tie-break
(`docs/bvh.md` §5) is generated specifically by `grid`.
