# Geometry Core

**Milestone:** Phase 1 — C++ Geometry Core
**Commit:** `841777e` "Phase 1: geometry core with CMake and GoogleTest" (baseline `a2a31a9` is scaffolding only)
**Scope of this document:** everything under `include/geometry/` and `src/geometry/` as of `841777e`.

Every behavioural claim below is cited as `file:line` against the source at that commit. Where a
code comment asserts something the code cannot demonstrate — typically a performance argument —
the claim is presented as *reasoning*, and marked `UNVERIFIED` where a measurement would be needed
to settle it. There are no benchmarks in this project yet: `benchmarks/results/` contains only a
format specification (`benchmarks/results/README.md:1-62`).

---

## Contents

1. [What problem it solves](#1-what-problem-it-solves)
2. [Why it exists](#2-why-it-exists)
3. [How it works](#3-how-it-works)
4. [Data structures used](#4-data-structures-used)
5. [Algorithms used](#5-algorithms-used)
6. [Time complexity](#6-time-complexity)
7. [Space complexity](#7-space-complexity)
8. [Important invariants](#8-important-invariants)
9. [Numerical assumptions](#9-numerical-assumptions)
10. [Performance considerations](#10-performance-considerations)
11. [Alternative approaches](#11-alternative-approaches)
12. [Why the chosen approach was selected](#12-why-the-chosen-approach-was-selected)
13. [Known limitations](#13-known-limitations)
14. [Edge cases](#14-edge-cases)
15. [Testing strategy](#15-testing-strategy)
16. [How it interacts with other components](#16-how-it-interacts-with-other-components)

Appendix: [Two bugs fixed in this milestone](#appendix-a--two-bugs-fixed-in-this-milestone),
[Toolchain caveat](#appendix-b--is-this-really-c20),
[What Phase 1 deliberately does not contain](#appendix-c--what-phase-1-deliberately-does-not-contain).

---

## 1. What problem it solves

The geometry core supplies the vocabulary types and exact/robust primitive operations that
everything above it — BVH construction, ray queries, collision queries, and eventually the Vulkan
viewer — is written in terms of. Concretely it answers five families of question:

| Question | Entry point |
|---|---|
| Where is this point / which way does this direction face, after a transform? | `transformPoint`, `transformVector`, `transformNormalWithInverse` (`include/geometry/mat4.hpp:109`, `:114`, `:125`) |
| What is the axis-aligned extent of this primitive / this set of primitives? | `Triangle::bounds` (`include/geometry/triangle.hpp:57`), `AABB::extend` (`include/geometry/aabb.hpp:42`), `Mesh::bounds` (`include/geometry/mesh.hpp:74`) |
| Does this ray hit this box, and at what distance? | `intersectRay` (`include/geometry/aabb.hpp:171`) |
| Do these two boxes overlap, and by how much? | `AABB::intersects`, `intersection` (`include/geometry/aabb.hpp:100`, `:137`) |
| How expensive is this box as a BVH node? | `AABB::surfaceArea` (`include/geometry/aabb.hpp:69`) |

It also solves a second, less obvious problem: **it makes the numerical behaviour of the system
explicit and testable.** The infinities, NaNs, and degenerate inputs that a spatial index will
certainly encounter are handled in named, unit-tested functions here rather than being rediscovered
as mystery bugs during traversal. `safeReciprocal` (`include/geometry/scalar.hpp:58`), `gamma`
(`include/geometry/scalar.hpp:42`), `normalizeSafe` (`include/geometry/vec3.hpp:90`) and
`Triangle::isDegenerate` (`include/geometry/triangle.hpp:51`) all exist for that reason.

## 2. Why it exists

**It is written by hand rather than taken from GLM or Embree.** `CLAUDE.md` makes this a
project-wide rule: geometry math, BVH construction, traversal, intersection and collision are
implemented in-house so the developer owns the explanation; GLM is permitted only on the rendering
side. The project doubles as interview preparation, so "I can't explain that, it's library code" is
a failure mode the constraint is designed to eliminate.

**It is deliberately dependency-free.** `CMakeLists.txt:96-98` states the rule and the build
enforces it: `bvh_geometry` (`CMakeLists.txt:101-104`) links only `bvh_project_options`
(`CMakeLists.txt:111`), and no header under `include/geometry/` includes a Vulkan, GLFW or ImGui
header. The consequence is that the whole algorithmic core is unit-testable on a machine with no
GPU and no window system — which matters a great deal here, because the development machine has an
integrated Iris Plus GPU reached through MoltenVK, and the Vulkan SDK is not yet installable (see
`CLAUDE.md`, Platform section).

**It is the layer where BVH design decisions are already visible.** Three members exist only
because of Phase 2:

- `AABB::offset` maps a point to `[0,1]` per axis and is documented as the SAH-binning bin-index
  function (`include/geometry/aabb.hpp:107-109`).
- `Triangle::centroid` exists because BVH partitioning sorts on centroids, not bounds
  (`include/geometry/triangle.hpp:65-67`).
- `Ray::tMax` is stored in the ray rather than passed alongside it so traversal can shrink it as
  closer hits are found (`include/geometry/ray.hpp:21-24`).

None of the consumers exist yet. See [Appendix C](#appendix-c--what-phase-1-deliberately-does-not-contain).

## 3. How it works

The core is eight headers and two translation units. Dependencies flow strictly downward:

```
scalar.hpp          Scalar typedef, IEEE-754 assertion, gamma(n), safeReciprocal
   |
   +-- vec3.hpp     Vec3: arithmetic, dot, cross, normalize, min/max, maxAxis
   |     |
   |     +-- vec4.hpp    Vec4: homogeneous coordinate
   |     |     |
   |     |     +-- mat4.hpp / mat4.cpp   Mat4: column-major, transforms, Gauss-Jordan inverse
   |     |           |
   |     |           +-- ray.hpp         Ray, invDirection, transformRay
   |     |           |     |
   |     |           |     +-- aabb.hpp  AABB, surfaceArea, slab-method intersectRay
   |     |           |           |
   |     |           |           +-- triangle.hpp   Triangle: bounds, centroid, degeneracy
   |     |           |                 |
   |     |           +-----------------+-- mesh.hpp / mesh.cpp   Mesh: indexed triangles
   |     |
   |     +-- plane.hpp   Plane in constant-normal form
```

Only `mat4.cpp` and `mesh.cpp` are compiled units (`CMakeLists.txt:101-104`); everything else is
header-only `inline`/`constexpr`, so the small operations that appear in inner loops are visible to
the optimiser at every call site.

Three design choices set the character of the whole layer:

**(a) One scalar type, not a template.** `using Scalar = float;`
(`include/geometry/scalar.hpp:20`). Nothing is templated on precision. The header states the reason
(`include/geometry/scalar.hpp:8-10`): a single typedef keeps a double-precision build reachable
without touching call sites, while the common case stays free of template noise. Section 10
discusses the float-vs-double argument itself.

**(b) Points and directions are different things, at the type and the API level.** `Vec4` carries
`w` explicitly (`include/geometry/vec4.hpp:10-14`), and the three transform entry points are
separate functions rather than one function with a convention:
`transformPoint` sets `w = 1` and performs the perspective divide when needed
(`src/geometry/mat4.cpp:172-181`), `transformVector` sets `w = 0`
(`src/geometry/mat4.cpp:183-186`), and `transformNormalWithInverse` applies the inverse transpose
of the upper-left 3×3 (`src/geometry/mat4.cpp:188-195`).

**(c) Degenerate input is a supported case, not an error.** Zero-length vectors, zero-area
triangles, collinear points and flat boxes all have defined, finite behaviour. Section 14 is the
full list.

## 4. Data structures used

| Type | Storage | Size (float `Scalar`) | Declared at |
|---|---|---:|---|
| `Vec3` | three named scalars `x, y, z` | 12 B | `include/geometry/vec3.hpp:23-53` |
| `Vec4` | four named scalars `x, y, z, w` | 16 B | `include/geometry/vec4.hpp:15-40` |
| `Mat4` | `Scalar m[4][4]`, indexed `m[column][row]` | 64 B | `include/geometry/mat4.hpp:21-64` |
| `Ray` | `origin`, `direction`, `tMin`, `tMax` | 32 B | `include/geometry/ray.hpp:25-36` |
| `Plane` | unit `normal` plus scalar `d` | 16 B | `include/geometry/plane.hpp:14-41` |
| `Triangle` | three explicit `Vec3` vertices | 36 B | `include/geometry/triangle.hpp:20-76` |
| `AABB` | `min`, `max` corners | 24 B | `include/geometry/aabb.hpp:27-122` |
| `Mesh` | `vector<Vec3>` positions + `vector<uint32_t>` indices + cached `AABB` | see §7 | `include/geometry/mesh.hpp:31-97` |

Four storage decisions are load-bearing.

**`Vec3` stores named members, and `operator[]` is a conditional chain — not `(&x)[i]`.**

```cpp
constexpr Scalar& operator[](int i) {
    assert(i >= 0 && i < 3);
    return i == 0 ? x : (i == 1 ? y : z);
}
```
(`include/geometry/vec3.hpp:32-35`, const overload at `:36-39`.)

The header explains why (`include/geometry/vec3.hpp:15-22`): `(&x)[i]` is undefined behaviour,
because pointer arithmetic is only defined within a single object and three separate non-array
members are not an array however they happen to be laid out. The conditional form is well-defined,
and because a conditional expression over lvalues is itself an lvalue it still yields a genuine
reference. The claim that optimisers reduce this to the same one or two instructions
(`include/geometry/vec3.hpp:20-22`) is a reasoned expectation — **UNVERIFIED**, no codegen
inspection or measurement is recorded in this repository. The same pattern is used for `Vec4`
(`include/geometry/vec4.hpp:25-32`). Indexed access is not decoration: `intersectRay` reaches
`box.min[axis]` on a runtime axis (`include/geometry/aabb.hpp:177-178`), and BVH split-axis
selection will do the same.

**`Mat4` is column-major.** `m[c][r]` throughout (`include/geometry/mat4.hpp:11-17`), with
translation living in the fourth *column* (`src/geometry/mat4.cpp:11-14`). The stated reason is
GLSL/SPIR-V interop: a `Mat4` can be memcpy'd into a Vulkan uniform buffer and consumed with no
transpose (`include/geometry/mat4.hpp:13-15`). The convention is column-vector, so a transform is
`M * v` and "first A, then B" composes as `B * A` (`include/geometry/mat4.hpp:19-20`), which
`tests/test_mat4.cpp:99` pins down. The admitted cost is that memory order no longer matches how a
matrix is written on paper, which is why the code says `m[c][r]` everywhere rather than using a
flat 16-element array (`include/geometry/mat4.hpp:15-17`).

Note the one place the code deliberately works against its own storage order: `invert` copies into
row-indexed local arrays because elimination is naturally row-oriented, then transposes back
(`src/geometry/mat4.cpp:108-116`, `:159-160`).

**`AABB` default-constructs to an inverted interval.**

```cpp
Vec3 min{kInfinity};
Vec3 max{-kInfinity};
```
(`include/geometry/aabb.hpp:28-29`.)

This is the single most consequential representation choice in the file. It makes `extend`
branch-free in both overloads:

```cpp
void extend(const Vec3& p) { min = minComponents(min, p); max = maxComponents(max, p); }
void extend(const AABB& b) { min = minComponents(min, b.min); max = maxComponents(max, b.max); }
```
(`include/geometry/aabb.hpp:42-52`.)

`min(+inf, p) == p` and `max(-inf, p) == p`, so the first point merged into an empty box produces
exactly that point's degenerate box with no empty-check (`include/geometry/aabb.hpp:22-26`).
Symmetrically, merging an *empty* box into a real one is a no-op for free
(`include/geometry/aabb.hpp:48-50`) — which is precisely what bottom-up bounds propagation in a BVH
needs, and `tests/test_aabb.cpp:48-56` asserts it under that name. A zero-initialised box would
instead wrongly contain the origin, silently inflating every bound in the tree toward `(0,0,0)`.

`isEmpty()` is then "inverted on at least one axis": `min.x > max.x || min.y > max.y || min.z > max.z`
(`include/geometry/aabb.hpp:40`). The header is careful to distinguish *empty* (not a region at
all) from *degenerate* (a point or a flat plane — a real zero-volume region that legitimately
participates in intersection tests) (`include/geometry/aabb.hpp:37-39`). That distinction is
enforced by tests: `tests/test_aabb.cpp:65-72` (flat box has real surface area, zero volume) and
`tests/test_aabb.cpp:235-242` (flat box is still hittable) versus `tests/test_aabb.cpp:244-248`
(empty box is never hit).

**`Mesh` is indexed, not an array of `Triangle`.** Positions and a `uint32_t` index buffer
(`include/geometry/mesh.hpp:94-95`), with triangles materialised on demand
(`include/geometry/mesh.hpp:61-66`). Two reasons are given (`include/geometry/mesh.hpp:18-25`): the
memory/cache argument, and that this is the layout `vkCmdDrawIndexed` wants, so the render path can
upload both buffers with no repacking. The memory arithmetic deserves a correction — see §10.

## 5. Algorithms used

### 5.1 Ray/AABB intersection — the slab method

This is the routine the entire project's performance will rest on, so it gets the most attention
here. Source: `include/geometry/aabb.hpp:171-209`, documented at `:141-170`.

**The idea.** An axis-aligned box is the intersection of three axis-aligned *slabs* (the region
between two parallel planes). For each slab, compute the two parameters at which the ray crosses
its planes, and keep a running `[tEnter, tExit]` interval that is the intersection of the ray's own
`[tMin, tMax]` with all slabs processed so far. The ray hits the box iff that interval is still
non-empty after all three axes.

```cpp
Scalar t0 = tMin;
Scalar t1 = tMax;
for (int axis = 0; axis < 3; ++axis) {
    Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
    Scalar tFar  = (box.max[axis] - origin[axis]) * invDir[axis];
    if (invDir[axis] < Scalar(0)) { const Scalar tmp = tNear; tNear = tFar; tFar = tmp; }
    tFar *= Scalar(1) + Scalar(2) * gamma(3);
    t0 = tNear > t0 ? tNear : t0;
    t1 = tFar  < t1 ? tFar  : t1;
    if (t1 < t0) return false;
}
```
(`include/geometry/aabb.hpp:173-205`.)

**Cost.** The header states 6 multiplies, 6 adds and a handful of comparisons, with no divides
because the caller supplies the reciprocal direction (`include/geometry/aabb.hpp:146-147`). Reading
the loop: 2 subtractions and 2 multiplies per axis for `tNear`/`tFar`, so 6 subtractions and 6
multiplies over three axes — the comment's arithmetic checks out, and the widening multiply
(`:196`) adds one more multiply per axis that the comment does not count. Actual instruction count
and throughput are **UNVERIFIED**.

**Hazard 1 — axis-parallel rays and `0 * inf == NaN`.** A zero direction component makes `invDir`
infinite (`include/geometry/ray.hpp:48-51`), and the products become ±inf, which compare correctly:
the slab is either fully entered or fully missed. But if the origin lies *exactly* on a slab plane
the numerator is zero too, and `0 * inf` is NaN (`include/geometry/aabb.hpp:151-157`). Every
comparison against NaN is false, so the code uses explicit ternaries rather than `std::max`/`std::min`:

```cpp
t0 = tNear > t0 ? tNear : t0;
t1 = tFar  < t1 ? tFar  : t1;
```
(`include/geometry/aabb.hpp:200-201`, with the warning "Do NOT replace with `std::max`/`std::min`:
their NaN behaviour is not guaranteed to match" at `:198-199`.)

When `tNear` is NaN, `tNear > t0` is false and `t0` keeps its existing value. The degenerate axis
becomes a no-op: the test does not constrain the interval on that axis at all. That is conservative
— it can report a hit the exact arithmetic would not — but it can never produce a false *miss*,
which is the failure mode that matters for an acceleration structure. Test:
`tests/test_aabb.cpp:207-216`, whose ray originates exactly on the `y = 1` face with no `y`
component and must still report a finite hit.

**Hazard 2 — rounding at grazing angles, and the `gamma(3)` widening.** Each `t` is a subtract
followed by a multiply, so it carries relative error bounded by `gamma(3)`
(`include/geometry/aabb.hpp:159-164`). `gamma(n)` is Higham's bound as used in PBRT
(`include/geometry/scalar.hpp:37-45`):

```cpp
inline constexpr Scalar gamma(int n) {
    const Scalar e = std::numeric_limits<Scalar>::epsilon() * Scalar(0.5);
    return (Scalar(n) * e) / (Scalar(1) - Scalar(n) * e);
}
```

with the guarantee `|computed - exact| <= gamma(n) * |exact|` (`include/geometry/scalar.hpp:41`).
Left uncorrected, a ray passing exactly through the shared face between two sibling BVH nodes can
be rejected by *both* — a visible crack in the geometry. Widening the exit distance by
`1 + 2*gamma(3)` (`include/geometry/aabb.hpp:196`) makes the test conservative in the safe
direction: it may report a hit fractionally outside the true box, costing one wasted narrow-phase
test, but it will not produce a crack. Note the asymmetry — only `tFar` is widened; `tNear` is left
alone. That is sufficient for watertightness on the exit side, matches the published PBRT form, and
is what the comment describes. **No test in this milestone constructs a grazing ray that would fail
without the widening** — the property is argued, not currently pinned down by a test.

**Why the swap tests the sign of `invDir` and not `tNear > tFar`.** See
[Appendix A](#appendix-a--two-bugs-fixed-in-this-milestone). This is the most interesting single
line in the milestone.

**Why the final comparison is strict.** `if (t1 < t0) return false;` (`include/geometry/aabb.hpp:204`)
rather than `<=`, so a flat box with `min == max` on an axis produces `t1 == t0` and still reports a
hit (`:203`). Pinned by `tests/test_aabb.cpp:235-242`.

**What `tEnter` means.** It receives `t0`, which was initialised to `tMin`
(`include/geometry/aabb.hpp:173`, `:207`). For a ray originating inside the box, that is `tMin` —
not a negative backward distance (`include/geometry/aabb.hpp:166-167`, tested at
`tests/test_aabb.cpp:173-180`).

### 5.2 Reciprocal direction without dividing by zero

```cpp
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}
```
(`include/geometry/scalar.hpp:58-62`, used by `invDirection` at `include/geometry/ray.hpp:48-51`.)

The slab test genuinely wants those infinities, so the goal is not to avoid them but to obtain them
legally. See [Appendix A](#appendix-a--two-bugs-fixed-in-this-milestone) for why `1/0` is not
acceptable in C++ even on IEEE-754 hardware. `copysign` is used rather than a plain `kInfinity`
because it propagates the sign of a *negative* zero, matching `1 / -0.0 == -inf`
(`include/geometry/scalar.hpp:60`). The cost is one predictable branch, paid once per ray rather
than once per node visited (`include/geometry/scalar.hpp:56-57`) — a claim the code structure
supports (it is called from `invDirection`, which the two-argument `intersectRay` overload calls
once at `include/geometry/aabb.hpp:212`), though the traversal loop that would amortise it does not
exist yet.

### 5.3 4×4 inverse — Gauss-Jordan with partial pivoting

`src/geometry/mat4.cpp:105-162`, documented at `include/geometry/mat4.hpp:88-98`.

The algorithm operates on the augmented system `[A | I]`. For each column it selects the pivot row
with the largest magnitude in that column (`src/geometry/mat4.cpp:118-130`), rejects the matrix if
the best pivot is `<= 1e-20` (`:133`), swaps rows if needed (`:135-140`), normalises the pivot row
(`:142-146`), and eliminates the column from every other row (`:148-156`). On success it transposes
the accumulated right-hand side back into column-major storage (`:159-160`) and returns `true`; on
failure it returns `false` and leaves `out` untouched (`include/geometry/mat4.hpp:96-97`,
`src/geometry/mat4.cpp:133`).

Partial pivoting is the whole point: dividing by a near-zero pivot amplifies existing rounding
error (`src/geometry/mat4.cpp:120-122`). The header states the tradeoff plainly — cofactor
expansion is the usual choice for a fixed 4×4 and is faster, but less numerically stable, and this
is not a hot path since transforms are built at most once per frame, not once per ray
(`include/geometry/mat4.hpp:90-94`). The relative speed of the two is **UNVERIFIED**; no
measurement exists.

`inverse()` (`src/geometry/mat4.cpp:164-170`) is the convenience wrapper: asserts in debug, returns
identity for a singular matrix in release. The header explicitly says to prefer `invert()` where
failure is possible (`include/geometry/mat4.hpp:100-101`).

### 5.4 Normal transformation by inverse transpose

```cpp
Vec3 transformNormalWithInverse(const Mat4& inverseTransform, const Vec3& n) {
    const Mat4& i = inverseTransform;
    return {i.m[0][0]*n.x + i.m[0][1]*n.y + i.m[0][2]*n.z,
            i.m[1][0]*n.x + i.m[1][1]*n.y + i.m[1][2]*n.z,
            i.m[2][0]*n.x + i.m[2][1]*n.y + i.m[2][2]*n.z};
}
```
(`src/geometry/mat4.cpp:188-195`.)

Verified against the storage convention: with `i.m[c][r]` meaning column `c`, row `r`, the element
of `i^T` at row `r`, column `c` is `i.m[r][c]`, so `(i^T n)_r = sum_c i.m[r][c] * n_c` — exactly
what the expression computes. The transpose is never materialised (`src/geometry/mat4.cpp:189-190`).

A normal is a covector: under non-uniform scale it does not transform like a direction. Scaling `x`
by 2 *halves* the `x` component of a surface normal rather than doubling it, so using
`transformVector` here is the classic bug that leaves normals non-perpendicular to their surfaces
(`include/geometry/mat4.hpp:118-121`). The function takes the already-inverted matrix so a caller
transforming many normals pays for one inverse, not one per normal
(`include/geometry/mat4.hpp:123-124`). `tests/test_mat4.cpp:158-182` proves the distinction
empirically: it builds a tangent/normal pair, applies a non-uniform scale, and asserts the
inverse-transpose result stays perpendicular to the transformed tangent while the
`transformVector` result does not.

### 5.5 Rotation by Rodrigues' formula

`R = I cos(t) + [k]_x sin(t) + k k^T (1 - cos(t))` (`src/geometry/mat4.cpp:26-51`). The axis is
normalised with `normalizeSafe`, and a zero-length axis short-circuits to identity rather than
producing NaN (`src/geometry/mat4.cpp:30-31`, tested at `tests/test_mat4.cpp:86`).
`rotationX/Y/Z` are thin wrappers (`src/geometry/mat4.cpp:53-55`).

### 5.6 Triangle degeneracy test without a square root

```cpp
bool isDegenerate(Scalar tol = kEpsilon) const {
    return lengthSquared(normalUnnormalized()) <= (Scalar(2) * tol) * (Scalar(2) * tol);
}
```
(`include/geometry/triangle.hpp:51-53`.)

`normalUnnormalized()` is `cross(v1-v0, v2-v0)` (`include/geometry/triangle.hpp:35`) whose magnitude
is twice the triangle area (`:31-32`). Squaring both sides of `2*area <= 2*tol` gives exactly the
expression above, so **`tol` is a tolerance on area**, defaulting to `kEpsilon = 1e-6`
(`include/geometry/scalar.hpp:35`). Avoiding the `sqrt` is stated as the reason for the squared
form (`include/geometry/triangle.hpp:50`).

### 5.7 Mesh index validation at the boundary

`src/geometry/mesh.cpp:8-25`. The constructor rejects an index count that is not a multiple of 3
(`:10-13`) and any index `>= positions_.size()` (`:16-23`), throwing `std::invalid_argument` with
the offending index and slot. The comment states the trade directly: one `O(n)` pass at
construction buys an unchecked inner loop for the lifetime of the mesh
(`src/geometry/mesh.cpp:14-15`), and `Mesh::triangle` relies on that by indexing both vectors with
unchecked `operator[]` (`include/geometry/mesh.hpp:61-66`). Exceptions rather than assertions are
used because this is the boundary where untrusted file data enters the system
(`include/geometry/mesh.hpp:38-40`).

## 6. Time complexity

Everything in the core is either constant-time or a single linear pass. `n` = vertex count,
`m` = triangle count, `i` = index count (`= 3m`).

| Operation | Complexity | Notes / citation |
|---|---|---|
| `Vec3`/`Vec4` arithmetic, `dot`, `cross`, `length` | O(1) | fixed 3/4 components |
| `Vec3::operator[]` | O(1) | 1–2 comparisons; constant index folds at compile time (`include/geometry/vec3.hpp:32-39`) |
| `Mat4 * Mat4` | O(1), 64 multiply-adds | triple loop over fixed 4 (`src/geometry/mat4.cpp:57-68`) |
| `Mat4 * Vec4` | O(1), 16 multiply-adds | `src/geometry/mat4.cpp:70-76` |
| `determinant` | O(1) | 12 2×2 minors then 6 products (`src/geometry/mat4.cpp:85-103`) |
| `invert` | O(1); 4 pivot passes × (pivot search + 4 rows × 4 cols) | `src/geometry/mat4.cpp:118-157` |
| `transformPoint` / `transformVector` / `transformNormalWithInverse` | O(1) | `src/geometry/mat4.cpp:172-195` |
| `AABB::extend`, `merge`, `intersection`, `surfaceArea`, `volume`, `contains`, `intersects`, `offset`, `longestAxis` | O(1) | `include/geometry/aabb.hpp:42-139` |
| `intersectRay` (slab) | O(1) — exactly 3 iterations, early-out at `:204` | `include/geometry/aabb.hpp:171-209` |
| `invDirection` | O(1), 3 branches | `include/geometry/ray.hpp:48-51` |
| `Triangle::bounds`, `centroid`, `normal`, `area`, `isDegenerate` | O(1) | `include/geometry/triangle.hpp:35-70` |
| `Mesh` construction | **O(n + i)** — one validation pass over indices, one bounds pass over positions | `src/geometry/mesh.cpp:10-24` |
| `Mesh::recomputeBounds` | O(n) | `src/geometry/mesh.cpp:27-30` |
| `Mesh::computeTriangleBounds` | O(i) = O(m) | `src/geometry/mesh.cpp:32-36` |
| `Mesh::countDegenerateTriangles` | O(m) | `src/geometry/mesh.cpp:38-45` |
| `Mesh::transform` | O(n) transform + O(n) bounds refresh | `src/geometry/mesh.cpp:47-50` |
| `Mesh::triangle(i)` | O(1), three indirect loads | `include/geometry/mesh.hpp:61-66` |

There is no super-linear algorithm anywhere in Phase 1. Note in particular that `Mesh::bounds()` is
O(1) at query time because the value is cached at construction
(`include/geometry/mesh.hpp:74`, `src/geometry/mesh.cpp:24`, refreshed at `:49`).

BVH construction and traversal complexity: `UNKNOWN — not yet implemented`.

## 7. Space complexity

Per-object, assuming `Scalar = float` (4 B) and 4-byte alignment throughout:

| Type | Bytes | Derivation |
|---|---:|---|
| `Vec3` | 12 | 3 × 4 (`include/geometry/vec3.hpp:24-26`) |
| `Vec4` | 16 | 4 × 4 (`include/geometry/vec4.hpp:16-19`) |
| `Mat4` | 64 | 16 × 4 (`include/geometry/mat4.hpp:23`) |
| `Ray` | 32 | 2 × `Vec3` + 2 scalars (`include/geometry/ray.hpp:26-29`) |
| `Plane` | 16 | `Vec3` + scalar (`include/geometry/plane.hpp:15-16`) |
| `Triangle` | 36 | 3 × `Vec3` (`include/geometry/triangle.hpp:21-23`) |
| `AABB` | **24** | 2 × `Vec3` (`include/geometry/aabb.hpp:28-29`) |

`Mesh` holds `12n + 4i + 24` bytes of payload plus three `std::vector` control blocks
(`include/geometry/mesh.hpp:94-96`). With `i = 3m`, that is `12n + 12m` bytes, and for a closed
manifold where each vertex is shared by about six triangles Euler's relation gives `n ≈ m/2`, so
roughly **`6m + 12m = 18m` bytes**, versus `36m` for an unpacked `vector<Triangle>`. That is a 2×
saving. The source comment at `include/geometry/mesh.hpp:20-22` says "roughly 12 bytes + 12 bytes
of indices per triangle versus 36 bytes unpacked", which understates the win: 12 B/triangle of
vertex data would correspond to `n ≈ m`, whereas the same comment's own "~6 triangles per vertex"
premise gives `n ≈ m/2` and therefore 6 B/triangle. **The comment's conclusion is right and its
arithmetic is slightly conservative.** See §13.

All operations use O(1) auxiliary space. `invert` allocates two 4×4 stack arrays
(`src/geometry/mat4.cpp:111-112`); nothing in the core heap-allocates except the `Mesh` vectors,
which are moved in rather than copied (`include/geometry/mesh.hpp:35-36`,
`src/geometry/mesh.cpp:9`).

## 8. Important invariants

**I1 — `Mesh` index buffer well-formedness.** `indices.size()` is a multiple of 3, and every index
is `< positions.size()`. Stated at `include/geometry/mesh.hpp:28-30`, enforced by throwing at
`src/geometry/mesh.cpp:10-23`, tested at `tests/test_mesh.cpp:84`, `:88`, `:93`. This is what
licenses unchecked indexing in `Mesh::triangle` (`include/geometry/mesh.hpp:61-66`) and in
`computeTriangleBounds` (`src/geometry/mesh.cpp:34`). The mesh is immutable in this respect: no
public member can add or reorder indices, so the invariant holds for the object's lifetime.

**I2 — `Mesh::bounds()` is always current.** Established in the constructor
(`src/geometry/mesh.cpp:24`) and re-established after the only mutating operation
(`src/geometry/mesh.cpp:49`). Tested at `tests/test_mesh.cpp:122`, `:131`.

**I3 — an empty `AABB` has `min > max` on at least one axis, and every measure of it is zero.**
`surfaceArea()` and `volume()` return 0 for an empty box (`include/geometry/aabb.hpp:69-79`) so SAH
cost sums stay finite (`:68`). Tested at `tests/test_aabb.cpp:9-18`.

**I4 — merging an empty `AABB` is the identity.** A direct consequence of the sentinel
(`include/geometry/aabb.hpp:47-52`). Tested under exactly the name that explains why it matters:
`tests/test_aabb.cpp:48-56`, "Relied on by bottom-up bounds propagation in BVH construction".

**I5 — `intersectRay` never reports a false miss.** Both numerical safeguards push in the
conservative direction: the NaN-tolerant ternaries leave the interval unconstrained on a degenerate
axis (`include/geometry/aabb.hpp:200-201`), and the `gamma(3)` widening only ever pushes `tExit`
outward (`:196`). A false positive costs one wasted child visit; a false negative would be a hole
in the image. Not directly testable as a property; argued from the two mechanisms.

**I6 — an empty `AABB` is never hit by any ray.** `include/geometry/aabb.hpp:169-170`, tested at
`tests/test_aabb.cpp:244-248`. This is *not* free — it is exactly what the swap-on-sign fix
protects. See [Appendix A](#appendix-a--two-bugs-fixed-in-this-milestone).

**I7 — `transformRay` preserves `[tMin, tMax]` and does not renormalise the direction.**
`include/geometry/ray.hpp:59-61`, so a point at parameter `t` in one space is the same physical
point at the same `t` in the other. Tested at `tests/test_ray.cpp:71-77` (range preserved) and
`tests/test_ray.cpp:79-91` (hit parameter consistent under scaling).

**I8 — `Plane` factory functions produce a unit normal; the raw constructor does not.** Stated at
`include/geometry/plane.hpp:35-37`. `signedDistance` is only a true distance when the normal is
unit.

**I9 — `maxAxis` ties resolve to the lowest index.** `include/geometry/vec3.hpp:114-117`, stated at
`:111-113` to keep BVH split-axis selection deterministic for symmetric bounds, which matters for
reproducible benchmarks. Tested for the cube case at `tests/test_aabb.cpp:84-85`.

**I10 — `AABB::contains(AABB)` treats empty as trivially contained; `intersects` treats it as
intersecting nothing.** `include/geometry/aabb.hpp:93` and `:101`. These are the correct empty-set
semantics and are deliberately asymmetric; both are tested (`tests/test_aabb.cpp:104`, `:113`).

## 9. Numerical assumptions

**IEEE-754 is required, and the requirement is checked, not assumed.**

```cpp
static_assert(std::numeric_limits<Scalar>::is_iec559,
              "geometry core requires IEEE-754 floating point");
```
(`include/geometry/scalar.hpp:26-27`.)

The specific properties relied upon are named at `include/geometry/scalar.hpp:22-25`: infinities
that compare correctly, NaNs that make every comparison false, and signed zero. Each is used:

- **Infinities that compare correctly** — the empty-`AABB` sentinel (`include/geometry/aabb.hpp:28-29`)
  and axis-parallel `invDir` (`include/geometry/ray.hpp:48-51`).
- **NaNs that falsify every comparison** — the `t0`/`t1` ternaries (`include/geometry/aabb.hpp:200-201`).
- **Signed zero** — `copysign` in `safeReciprocal` (`include/geometry/scalar.hpp:61`), so a
  direction component of `-0.0` yields `-inf` and the slab is entered from the correct side.

**`-ffast-math` must never be enabled.** `CMakeLists.txt:81-84` states this and the build never
adds it; `CLAUDE.md` repeats it as a project rule. Fast-math permits the compiler to assume NaN and
infinity never occur, which does not make the NaN-tolerant slab test faster — it makes it *wrong*.
This is a rare case where an optimisation flag silently converts a correct program into an
incorrect one, and it is worth being able to say so.

**Float carries about 7 decimal digits, and the code is written accordingly.**
`include/geometry/scalar.hpp:14-16` acknowledges this directly: the precision cost of `float` is
why the intersection routines are written to be robust rather than to assume exact arithmetic.

**There is no single global epsilon for intersection.** `kEpsilon = 1e-6`
(`include/geometry/scalar.hpp:35`) exists for "are these two quantities the same" comparisons and
is explicitly *not* used inside intersection tests, because a single global epsilon is meaningless
across different magnitudes (`include/geometry/scalar.hpp:32-34`). The slab test instead uses a
*relative* bound, `gamma(3)` (`include/geometry/aabb.hpp:196`), which scales with the magnitude of
the quantity it corrects.

**Comparison policy.** `Vec3::operator==` is exact bitwise-value comparison, documented as suitable
for exactly-representable test values and for detecting "unchanged", and explicitly *not* for
comparing results of floating-point arithmetic (`include/geometry/vec3.hpp:46-51`). `nearlyEqual`
is the absolute-or-relative alternative: absolute near zero where relative error is meaningless,
relative for large magnitudes where an absolute epsilon is too strict
(`include/geometry/scalar.hpp:67-74`), overloaded for `Vec3`, `Vec4`, `Mat4` and `AABB`
(`include/geometry/vec3.hpp:119`, `include/geometry/vec4.hpp:55`, `include/geometry/mat4.hpp:127`,
`include/geometry/aabb.hpp:215`).

**Overflow avoidance in `AABB::centroid`.** Written as `min + 0.5*(max-min)` rather than
`0.5*(min+max)` specifically to avoid overflowing to infinity when both corners are large and
same-signed (`include/geometry/aabb.hpp:56-59`).

**Singularity threshold.** `invert` rejects a pivot of magnitude `<= 1e-20`
(`src/geometry/mat4.cpp:133`) rather than producing inf/NaN. This is an absolute threshold on a
`float` matrix, so it is conservative — a matrix scaled down by `1e-10` uniformly would be
rejected even though its inverse exists. See §13.

## 10. Performance considerations

> No measurements exist. `benchmarks/results/` contains only the format specification
> (`benchmarks/results/README.md`), which itself states that a number not recorded there "does not
> exist and must be written as `UNVERIFIED`". Everything in this section is an *argument about
> algorithmic or cache behaviour*, not a result.

**Float over double: the memory-bandwidth argument.** `include/geometry/scalar.hpp:12-16`:

> a BVH is memory-bound, not ALU-bound. Every AABB is 6 scalars; halving their width halves the
> bytes pulled through cache during traversal, which is where the time actually goes.

The premise is verifiable from the code: `AABB` is exactly 2 × `Vec3` = 24 B at float, 48 B at
double (`include/geometry/aabb.hpp:28-29`). BVH traversal is a pointer-chasing walk over a node
array whose working set is dominated by bounds, so halving node width roughly doubles the number of
nodes per cache line. The *conclusion* — that this dominates the ALU cost — is standard and
plausible, and is what production renderers do, but for this project it is **UNVERIFIED**: no
double-precision build has been measured against a float one, and the traversal loop that the
argument is about does not exist yet.

The honest interview framing is: "float is chosen for node density, and the precision cost is
bought back by writing the intersection code to be robust — the `gamma(3)` widening and the
NaN-tolerant comparisons are what pay for that choice."

**AABB over tighter bounding volumes.** `include/geometry/aabb.hpp:13-20`:

> the bounding volume is tested far more often than it is built, so the cost that matters is the
> per-test cost, not the tightness.

Three concrete consequences the code demonstrates: (1) the AABB/ray test is a handful of multiplies
and comparisons with no trigonometry and no matrix (`include/geometry/aabb.hpp:171-209`), where an
OBB would need a transform into its local frame first; (2) AABBs are closed under union in a way
that is trivial to compute — component-wise min/max (`include/geometry/aabb.hpp:42-52`) — which is
what makes bottom-up bounds propagation cheap; (3) the admitted cost is a looser fit for diagonal
geometry, appearing as more false-positive node visits (`include/geometry/aabb.hpp:19-20`).
Quantifying (3) needs measurement: **UNVERIFIED**.

**Precomputing the reciprocal direction.** `invDirection` is computed once per ray and reused
across every node visited, turning three divides per AABB into three multiplies
(`include/geometry/ray.hpp:40-41`). Division latency is several times multiply latency on x86, so
the direction of the argument is not in doubt; the magnitude for this workload is **UNVERIFIED**.
Note the `intersectRay` overload at `include/geometry/aabb.hpp:211-213` recomputes `invDirection`
per call — convenient for tests, but a traversal loop must use the five-argument form at `:171` and
hoist the reciprocal itself.

**Indexed mesh storage and cache.** `include/geometry/mesh.hpp:18-25`. The saving is roughly 2×
(see §7). The stated framing is the interesting part: "On a million-triangle mesh that difference
decides whether the vertex data fits in cache during a build"
(`include/geometry/mesh.hpp:21-22`). Whether a given mesh crosses a specific cache threshold is a
function of the machine and the mesh, so as a general claim it is **UNVERIFIED** — but the
structural point (indexed storage is ~half the bytes, and build-time bandwidth is the constraint)
follows from the layout. The cost is honestly stated as one level of indirection per vertex fetch,
which matters during *traversal*, with the mitigation deferred: "Phase 2 can revisit by storing
unpacked triangles in leaf order" (`include/geometry/mesh.hpp:25-26`). That is the classic
build-vs-query layout tradeoff, and it is already flagged in the source.

**Header-only hot primitives.** Everything except `Mat4` and `Mesh` is `inline`/`constexpr` in
headers, so `dot`, `cross`, `minComponents`, `extend`, and `intersectRay` are all visible for
inlining at the call site. Only `src/geometry/mat4.cpp` and `src/geometry/mesh.cpp` are compiled
separately (`CMakeLists.txt:101-104`), and neither contains a per-ray operation.

**Avoided square roots.** `Triangle::isDegenerate` compares squared area against squared tolerance
(`include/geometry/triangle.hpp:50-53`); `normalizeSafe` tests `lengthSquared` before deciding to
take the root (`include/geometry/vec3.hpp:91-93`); `normalUnnormalized` is returned unnormalised so
callers needing only an orientation sign can skip the root entirely
(`include/geometry/triangle.hpp:33-35`).

**Const-correctness and copies.** `Mesh` takes both buffers by value and moves
(`include/geometry/mesh.hpp:35-41`, `src/geometry/mesh.cpp:9`), with the comment noting these may
be tens of megabytes. `Mesh::triangle` returns by value with an explicit justification — 36 bytes
is cheaper to copy than to alias, and returning a value keeps the `Mesh` immutable to callers
(`include/geometry/mesh.hpp:59-60`).

**Warning set as a performance guard.** `-Wconversion`, `-Wsign-conversion` and
`-Wdouble-promotion` are all enabled (`CMakeLists.txt:66-69`), the last specifically to catch an
accidental `float`→`double` promotion in a hot loop — which would silently undo the float decision.
The commit message for `841777e` reports the tree is clean under `-Werror` with the full set.

## 11. Alternative approaches

| Decision | Alternatives considered | Consequence of the alternative |
|---|---|---|
| `Scalar = float` (`include/geometry/scalar.hpp:20`) | `double`; templated precision | `double`: ~7 → ~16 digits, but 48-byte AABBs and half the nodes per cache line. Templates: precision noise at every call site; the typedef keeps a double build reachable without that (`:8-10`). |
| AABB (`include/geometry/aabb.hpp:13-20`) | OBB, sphere, k-DOP | OBB: tighter, but each test needs a transform into local frame, and union is not component-wise. Sphere: cheapest test, but very loose for elongated geometry and the union of two spheres is not a sphere. k-DOP: intermediate on both axes; more planes per test, more storage per node. |
| Surface area as the SAH cost term (`include/geometry/aabb.hpp:62-66`) | Volume weighting; primitive count alone | Volume weighting has no geometric-probability justification for rays; count alone ignores that a large box is hit more often. |
| Inverted-interval empty box (`include/geometry/aabb.hpp:22-26`) | Explicit `bool empty` flag; zero-initialised box | Flag: a branch in `extend`, which is the innermost operation of bounds propagation, plus a wider struct. Zero-init: silently wrong — the box contains the origin. |
| Swap on `sign(invDir)` (`include/geometry/aabb.hpp:180-193`) | Swap on `tNear > tFar` (the widely published form) | Repairs an inverted empty box into `[-inf, +inf]`, so every empty box hits every ray. Same cost either way. See Appendix A. |
| `safeReciprocal` (`include/geometry/scalar.hpp:47-62`) | Plain `1/d`; clamping `d` away from zero; branching in traversal | Plain `1/d`: UB per `[expr.mul]/4`, flagged by UBSan. Clamping: introduces a fictitious direction and a wrong `t`. Branching in traversal: moves a per-ray cost into a per-node loop (`include/geometry/ray.hpp:44-46`). |
| Gauss-Jordan + partial pivoting (`include/geometry/mat4.hpp:88-94`) | Cofactor/adjugate expansion; LU | Cofactor: faster for a fixed 4×4, less stable; the code judges stability more valuable because this inverse brings rays into object space where error becomes a wrong intersection rather than a slightly wrong pixel. |
| Column-major (`include/geometry/mat4.hpp:11-17`) | Row-major | Row-major reads more naturally on paper but requires a transpose on every upload to a GLSL/SPIR-V uniform. |
| Non-normalised ray direction (`include/geometry/ray.hpp:11-19`) | Always unit-length | A `sqrt` per ray, and — the real problem — renormalising after a scaling transform silently changes the meaning of `t`, so local-space and world-space hit distances stop being comparable. |
| Indexed mesh (`include/geometry/mesh.hpp:18-26`) | `vector<Triangle>` | ~2× the memory; no indirection during traversal; would need repacking before `vkCmdDrawIndexed`. |
| Throwing constructor (`include/geometry/mesh.hpp:38-40`) | `assert`; error code; silent clamping | Assertions vanish in release, which is exactly when malformed file data arrives. |
| Separate point/vector/normal transforms (`include/geometry/mat4.hpp:104-125`) | One `transform(Mat4, Vec3)` | A silent convention rather than a checked one; the inverse-transpose bug becomes invisible. |

## 12. Why the chosen approach was selected

The selections above share one criterion, stated in `CLAUDE.md`: *simple and defensible beats
clever*, because every decision must be explainable in an interview. Three threads run through the
choices.

**Optimise the operation that runs most often, not the one that looks most expensive.** AABBs over
OBBs, float over double, precomputed reciprocals, and the branch-free `extend` all follow from
asking "how many times per frame does this execute?" The AABB header states the principle
explicitly (`include/geometry/aabb.hpp:13-15`), and `invert` is the counter-example that proves the
rule: it is allowed to be slower because it runs once per frame, not once per ray
(`include/geometry/mat4.hpp:91-93`).

**Prefer being conservatively wrong to being occasionally wrong.** For an acceleration structure a
false positive costs one wasted narrow-phase test; a false negative is a visible hole. Every
numerical decision in `intersectRay` picks that side: the `gamma(3)` widening
(`include/geometry/aabb.hpp:159-164`), the NaN no-op (`:151-157`), the strict `<` that keeps flat
boxes hittable (`:203-204`). `AABB::intersects` makes the same call for touching boxes, with the
reasoning spelled out (`include/geometry/aabb.hpp:97-99`).

**Make the abstract machine, not just the hardware, correct.** Both bugs fixed in this milestone
are cases where code that "works" on real hardware is undefined by the standard: `(&x)[i]` and
`1/0`. Both were replaced with constructs that produce identical results with defined semantics,
at no cost (`include/geometry/vec3.hpp:17-22`, `include/geometry/scalar.hpp:50-57`). The build is
set up to keep finding these: `BVH_ENABLE_SANITIZERS` wires up ASan and UBSan
(`CMakeLists.txt:86-91`), and the commit message for `841777e` reports the tree clean under UBSan.

## 13. Known limitations

1. **`AABB::centroid()` on an empty box returns NaN.** `min + (max - min) * 0.5`
   (`include/geometry/aabb.hpp:59`) with `min = +inf, max = -inf` gives `(-inf) * 0.5 = -inf` and
   then `+inf + -inf = NaN`. `surfaceArea` and `volume` guard against empty (`:70`, `:76`) but
   `centroid`, `diagonal` and `longestAxis` do not. BVH construction computes centroid bounds over
   primitives, so an empty box reaching that path would poison the bin assignment. No test covers
   it. **Needs a decision from the Implementation Engineer** — guard, or document as a
   precondition.
2. **No `intersectRay` test exercises the `gamma(3)` widening.** The widening is argued for in the
   comment (`include/geometry/aabb.hpp:159-164`) but no test constructs a grazing ray that fails
   without it, so a regression that removed line `:196` would pass the suite.
3. **`Plane::fromPointNormal` uses `normalize`, not `normalizeSafe`**
   (`include/geometry/plane.hpp:23`), so a zero normal asserts in debug and yields inf/NaN in
   release — whereas `fromPoints` handles the same degeneracy gracefully
   (`include/geometry/plane.hpp:31`). The asymmetry is untested and probably unintended.
4. **The singularity threshold in `invert` is absolute, not relative.** `best <= 1e-20`
   (`src/geometry/mat4.cpp:133`) rejects any matrix whose largest pivot is tiny in absolute terms,
   including a well-conditioned matrix that has simply been scaled down. A relative test against
   the matrix norm would be scale-invariant.
5. **The memory arithmetic in the `Mesh` header comment understates the win.**
   `include/geometry/mesh.hpp:20-22` says 12 B + 12 B per triangle; its own "~6 triangles per
   vertex" premise gives 6 B + 12 B. Conclusion unaffected (see §7).
6. **`Mesh::bounds()` bounds the vertex array, not the referenced triangles.** Honestly documented
   (`include/geometry/mesh.hpp:68-73`) and tested (`tests/test_mesh.cpp:101`): a mesh with orphan
   vertices gets a conservative — looser but still correct — root bound.
   `computeTriangleBounds()` gives the tight answer at O(m) (`include/geometry/mesh.hpp:76-78`).
   Whether BVH construction should use the tight version is a Phase 2 decision.
7. **The two-argument `intersectRay` recomputes `invDirection` per call**
   (`include/geometry/aabb.hpp:211-213`). Fine for tests; traversal must use the five-argument form.
8. **`Mesh::transform` applies `transformPoint`, which includes a perspective divide**
   (`src/geometry/mesh.cpp:48`, `src/geometry/mat4.cpp:176-179`), although the member is documented
   as taking an *affine* transform (`include/geometry/mesh.hpp:85`). Harmless — affine matrices
   leave `w == 1` and skip the divide — but the divide is checked per vertex.
9. **No SIMD, no multithreading, no arena allocation.** All deferred; `CLAUDE.md` bans premature
   optimisation without measurement.
10. **`Mesh` has no OBJ loader.** The plan lists OBJ loading (`BVH_Explorer_Project_Plan.md`,
    Technology Stack) but Phase 1 provides only the in-memory representation.
    `UNKNOWN — not yet implemented`.
11. **The project cannot honestly claim C++20.** See [Appendix B](#appendix-b--is-this-really-c20).

## 14. Edge cases

Degenerate input is handled deliberately everywhere. This table is the checklist.

| Input | Behaviour | Citation | Tested |
|---|---|---|---|
| Default-constructed `AABB` | empty; `surfaceArea == 0`, `volume == 0` | `include/geometry/aabb.hpp:28-29`, `:69-79` | `tests/test_aabb.cpp:9` |
| `extend` an empty box with a point | degenerate single-point box, no special case | `include/geometry/aabb.hpp:42-45` | `tests/test_aabb.cpp:20` |
| `merge` with an empty box | identity | `include/geometry/aabb.hpp:47-52` | `tests/test_aabb.cpp:48` |
| Flat (zero-thickness) `AABB` | not empty; real surface area, zero volume; **still hittable** | `include/geometry/aabb.hpp:37-39`, `:203-204` | `tests/test_aabb.cpp:65`, `:235` |
| Empty `AABB` vs any ray | never hit — and this is exactly what the swap-on-sign fix protects | `include/geometry/aabb.hpp:180-193` | `tests/test_aabb.cpp:244` |
| Empty `AABB` in `contains(AABB)` / `intersects` | contained: true; intersects: false | `include/geometry/aabb.hpp:93`, `:101` | `tests/test_aabb.cpp:104`, `:113` |
| Axis-parallel ray, origin outside the slab | ±inf products compare correctly; miss | `include/geometry/aabb.hpp:151-155` | `tests/test_aabb.cpp:199` |
| Axis-parallel ray, origin **exactly on** a slab plane | `0 * inf == NaN`; ternaries make the axis a no-op; finite hit reported | `include/geometry/aabb.hpp:151-157`, `:200-201` | `tests/test_aabb.cpp:207` |
| Ray origin inside the box | hit with `tEnter == tMin`, not a negative distance | `include/geometry/aabb.hpp:166-167` | `tests/test_aabb.cpp:173` |
| Ray pointing away from the box | miss (interval rejected) | `include/geometry/aabb.hpp:204` | `tests/test_aabb.cpp:166` |
| Box outside `[tMin, tMax]` | miss | `include/geometry/aabb.hpp:173-174` | `tests/test_aabb.cpp:182`, `:192` |
| Negative direction components | swap on the sign of `invDir` | `include/geometry/aabb.hpp:189-193` | `tests/test_aabb.cpp:218` |
| `offset()` on a degenerate axis | returns 0 for that axis; no divide by zero | `include/geometry/aabb.hpp:110-115` | `tests/test_aabb.cpp:142` |
| Touching boxes | count as intersecting (deliberate) | `include/geometry/aabb.hpp:97-99` | `tests/test_aabb.cpp:116` |
| Disjoint boxes in `intersection()` | empty result, not garbage | `include/geometry/aabb.hpp:136-139` | `tests/test_aabb.cpp:131` |
| `normalize` of a zero vector | asserts in debug; inf/NaN in release — documented precondition | `include/geometry/vec3.hpp:78-85` | — |
| `normalizeSafe` of a short vector | returns `fallback` (default zero vector) | `include/geometry/vec3.hpp:90-94` | `tests/test_vec3.cpp:104` |
| Zero-area / collinear triangle | `normal()` returns zero, **not NaN**; `area() == 0`; bounds and centroid still valid | `include/geometry/triangle.hpp:37-53` | `tests/test_triangle.cpp:82`, `:91`, `:100` |
| Duplicate triangle vertices | flagged degenerate | `include/geometry/triangle.hpp:51-53` | `tests/test_triangle.cpp:100` |
| Collinear / coincident points in `Plane::fromPoints` | degenerate plane (zero normal), not NaN | `include/geometry/plane.hpp:27-33`, `:40` | `tests/test_plane.cpp:30`, `:36` |
| Zero-length rotation axis | identity matrix, not NaN | `src/geometry/mat4.cpp:30-31` | `tests/test_mat4.cpp:86` |
| Singular matrix in `invert` | returns false, `out` untouched | `src/geometry/mat4.cpp:133` | `tests/test_mat4.cpp:146` |
| Projection matrix in `transformPoint` | perspective divide applied; affine case skips it | `src/geometry/mat4.cpp:174-180` | `tests/test_mat4.cpp:183` |
| Index count not a multiple of 3 | `std::invalid_argument` | `src/geometry/mesh.cpp:10-13` | `tests/test_mesh.cpp:84` |
| Out-of-range index | `std::invalid_argument` naming index and slot | `src/geometry/mesh.cpp:16-23` | `tests/test_mesh.cpp:88`, `:93` |
| Empty mesh | accepted; `empty()` true | `include/geometry/mesh.hpp:45` | `tests/test_mesh.cpp:36`, `:97` |
| Orphan (unreferenced) vertices | `bounds()` conservative; `computeTriangleBounds()` tight | `include/geometry/mesh.hpp:68-78` | `tests/test_mesh.cpp:101` |
| Degenerate triangles in a mesh | counted and reported, never silently dropped | `include/geometry/mesh.hpp:80-83` | `tests/test_mesh.cpp:114` |
| `maxAxis` on a cube / tie | lowest index wins, deterministically | `include/geometry/vec3.hpp:111-117` | `tests/test_aabb.cpp:84` |
| **Empty `AABB` in `centroid()`** | **NaN — unguarded, untested** | `include/geometry/aabb.hpp:56-60` | **none** (see §13.1) |

## 15. Testing strategy

**Framework and wiring.** GoogleTest, preferring an installed copy and otherwise fetching a pinned
v1.15.2 (`tests/CMakeLists.txt:6-22`). The version is pinned rather than tracking a branch so a test
failure is always attributable to our change, never to a silently updated dependency
(`tests/CMakeLists.txt:3-5`). `gtest_discover_tests` registers each `TEST()` individually so a
failure names the case and `ctest` can parallelise (`tests/CMakeLists.txt:46-51`).

**Size.** 105 `TEST()` cases across seven files, counted statically at `841777e`:

| File | Cases | Focus |
|---|---:|---|
| `tests/test_aabb.cpp` | 28 | construction, sentinel, measures, containment, offset, and 12 ray-intersection cases |
| `tests/test_mat4.cpp` | 18 | storage layout, transforms, inverse, normal transform, perspective divide |
| `tests/test_vec3.cpp` | 17 | arithmetic, dot/cross handedness, normalize, min/max, `maxAxis` |
| `tests/test_mesh.cpp` | 15 | counts, accessors, validation rejections, bounds, transform |
| `tests/test_triangle.cpp` | 12 | edges, normal winding, area, centroid, bounds tightness, degeneracy |
| `tests/test_ray.cpp` | 10 | construction, `invDirection`, transform semantics |
| `tests/test_plane.cpp` | 5 | factories, signed distance, degeneracy |

The commit message for `841777e` reports all 105 passing, clean under `-Werror` with the full
strict warning set, and clean under UBSan. **I did not re-run the suite** — there is no `build/`
directory in the tree and a first configure needs network access to fetch GoogleTest
(`tests/CMakeLists.txt:8-21`). The count of 105 is verified statically; the pass/fail status is
quoted from the commit.

**What the strategy actually is.** Four distinguishable layers:

1. *Known-answer tests* for closed-form results: a 1×2×3 box has surface area 22 and volume 6
   (`tests/test_aabb.cpp:58-63`); a ray from `z = -5` hits the unit box at `t = 4`
   (`tests/test_aabb.cpp:152-158`).
2. *Property tests* asserting mathematical relationships rather than constants: cross product is
   orthogonal to both inputs (`tests/test_vec3.cpp:84`); rotation preserves length
   (`tests/test_mat4.cpp:80`); rotation about an axis leaves that axis fixed
   (`tests/test_mat4.cpp:92`); `M * M^-1 == I` (`tests/test_mat4.cpp:129`); world→object→world
   round-trip recovers the original ray (`tests/test_ray.cpp:93-106`); a matrix-vector product
   matches its manual expansion (`tests/test_mat4.cpp:200-208`).
3. *Degeneracy and hazard tests* — the distinctive part of this suite. Nearly every row of §14 has
   a named test, and several are named after the failure they prevent:
   `RayLyingExactlyOnSlabBoundaryIsHandled` (`tests/test_aabb.cpp:207`),
   `EmptyBoxIsNeverHit` (`:244`), `FlatBoxIsStillHittable` (`:235`),
   `OffsetOnDegenerateAxisDoesNotDivideByZero` (`:142`),
   `CollinearPointsGiveDegeneratePlaneNotNaN` (`tests/test_plane.cpp:30`),
   `RotationAboutDegenerateAxisIsIdentity` (`tests/test_mat4.cpp:86`).
4. *Convention-pinning tests* that would otherwise be undetectable drift:
   `ColumnMajorStorageLayout` (`tests/test_mat4.cpp:28`),
   `MultiplicationAppliesRightmostFirst` (`:99`),
   `CrossProductIsRightHanded` (`tests/test_vec3.cpp:67`),
   `NormalFollowsCounterClockwiseWinding` (`tests/test_triangle.cpp:20`),
   `DirectionIsNotRequiredToBeNormalized` (`tests/test_ray.cpp:35`).

**Two tests are worth reading in full as documentation.**
`tests/test_mat4.cpp:158-182` (`NormalTransformUnderNonUniformScale`) does not merely check the
inverse-transpose result; it constructs a tangent/normal pair, applies a non-uniform scale, then
asserts the correct normal stays perpendicular to the transformed tangent *and* that the naive
`transformVector` answer does not — so the test fails if someone "simplifies" the function. And
`tests/test_ray.cpp:79-91` (`TransformKeepsHitParameterConsistentUnderScaling`) asserts
`transformPoint(m, r.at(t)) == transformRay(m, r).at(t)`, which is exactly the property that would
break if `transformRay` renormalised the direction.

**Comparison discipline in tests.** Exact `EXPECT_EQ` on `Vec3` is used only where values are
exactly representable — sentinels, integral coordinates, unchanged components
(`tests/test_aabb.cpp:13-14`, `:36-37`) — and `nearlyEqual`/`EXPECT_NEAR` everywhere arithmetic has
occurred (`tests/test_aabb.cpp:76`, `tests/test_ray.cpp:104-105`). This mirrors the policy
documented at `include/geometry/vec3.hpp:46-48`.

**Gaps.** No property-based/fuzz testing (a randomised ray-vs-brute-force cross-check becomes
possible in Phase 2). No test for the `gamma(3)` widening (§13.2). No test for
`AABB::centroid()` on an empty box (§13.1). No benchmark harness — Phase 9.

## 16. How it interacts with other components

**Downward: nothing.** The geometry core depends only on the C++ standard library —
`<cmath>`, `<limits>`, `<cassert>`, `<algorithm>`, `<cstddef>`, `<cstdint>`, `<vector>`,
`<stdexcept>`, `<string>`, `<utility>`. No third-party headers, no Vulkan, no GLFW, no ImGui. This
is the project's "single most important structural rule" (`CLAUDE.md`, Layering), restated in the
build at `CMakeLists.txt:96-98`.

**Upward: everything.** Consumers, present and planned:

| Consumer | Status | What it will use |
|---|---|---|
| `tests/` | **exists** | the whole surface; links `bvh::geometry` (`tests/CMakeLists.txt:34-38`) |
| BVH construction (Phase 2) | `UNKNOWN — not yet implemented` | `AABB::extend`/`merge` for bottom-up bounds; `Triangle::centroid` for partitioning; `AABB::longestAxis` for the default split axis; `AABB::surfaceArea` for SAH cost; `AABB::offset` for SAH bin indices |
| BVH traversal (Phase 2) | `UNKNOWN — not yet implemented` | the five-argument `intersectRay` with a hoisted `invDirection`; `Ray::tMax` shrinking as closer hits are found |
| Ray/triangle intersection | `UNKNOWN — not yet implemented` | **deliberately not in Phase 1**; no Möller–Trumbore or equivalent exists in this tree |
| Collision / clearance (Phase 6) | `UNKNOWN — not yet implemented` | `AABB::intersects`, `intersection`, `Plane::signedDistance` |
| Vulkan rendering (Phase 7) | `UNKNOWN — not yet implemented` | `Mat4` uploaded untransposed; `Mesh` positions and indices uploaded for `vkCmdDrawIndexed`; `transformNormalWithInverse` for shading normals |
| GPU compute (Phase 8) | `UNKNOWN — not yet implemented` | — |
| Benchmarks (Phase 9) | `UNKNOWN — not yet implemented` | `Mesh::countDegenerateTriangles` so mesh quality is visible rather than quietly changing the primitive count (`include/geometry/mesh.hpp:80-82`) |

**The seams already cut for Phase 2.** Four places where Phase 1 shaped itself around a consumer
that does not exist yet, each documented in the source at the point of the decision:

- `AABB::offset` — SAH bin index (`include/geometry/aabb.hpp:107-109`).
- `Triangle::centroid` — "BVH construction partitions on centroids rather than on bounds, because a
  centroid gives each primitive exactly one position to sort by, whereas overlapping bounds do not
  induce an ordering" (`include/geometry/triangle.hpp:65-67`).
- `Ray::tMax` carried in the ray — "traversal can shrink tMax as closer hits are found... once a
  hit at t is known, every subtree whose entry distance exceeds t can be skipped outright"
  (`include/geometry/ray.hpp:21-24`).
- `AABB::surfaceArea` — the SAH cost term, with the ray-hit-probability justification stated inline
  (`include/geometry/aabb.hpp:62-66`).

**The seam already cut for the renderer:** column-major `Mat4` so uniform upload needs no transpose
(`include/geometry/mat4.hpp:13-15`), and indexed `Mesh` so buffers upload without repacking
(`include/geometry/mesh.hpp:23-24`). Neither can be exercised until the Vulkan SDK is installable
on this machine (`CLAUDE.md`, Toolchain status).

---

## Appendix A — two bugs fixed in this milestone

Both were found while writing the tests, both are documented in the `841777e` commit message and in
the source comments, and — because Phase 1 landed as a single commit — **neither appears as a diff
in git history.** The buggy versions never reached a commit. The evidence for them is the commit
message, the code comments that explain what was avoided, and the tests written to pin the fixes.

### A.1 The slab-test swap: `tNear > tFar` vs `sign(invDir)`

The widely published form of the slab test swaps when the near value exceeds the far value:

```cpp
if (tNear > tFar) std::swap(tNear, tFar);   // the bug
```

The code swaps on the sign of the reciprocal direction instead:

```cpp
if (invDir[axis] < Scalar(0)) { const Scalar tmp = tNear; tNear = tFar; tFar = tmp; }
```
(`include/geometry/aabb.hpp:189-193`, reasoning at `:180-188`.)

**Why the comparison form is wrong here.** It interacts with the inverted-interval empty box. Take
a default-constructed `AABB` — `min = +inf`, `max = -inf` (`include/geometry/aabb.hpp:28-29`) —
and a ray with a positive `x` direction, so `invDir.x > 0`:

```
tNear = (+inf - origin.x) * invDir.x = +inf
tFar  = (-inf - origin.x) * invDir.x = -inf
```

`tNear > tFar` is true, so the comparison form swaps, producing `tNear = -inf`, `tFar = +inf`. The
interval `[-inf, +inf]` constrains nothing. All three axes do the same, `t1 < t0` never fires, and
**the function returns true: an empty box hits every ray.** The `if` that was supposed to repair a
negative direction has instead repaired the sentinel that encodes "this box is not a region at all".

The sign form leaves the inverted interval inverted: `t0` becomes `+inf`, `t1` becomes `-inf`,
`t1 < t0` fires on the first axis, and the box is correctly rejected
(`include/geometry/aabb.hpp:185-188`). The same holds for a negative direction — then `tNear = -inf`
and `tFar = +inf`, the sign test swaps them back to `+inf`/`-inf`, and the rejection still fires —
and for an axis-parallel direction where `invDir` is `+inf`, giving the same `+inf`/`-inf` pair.

**Why it matters in a BVH.** Empty bounds are not exotic there: a node with no primitives, a
partially built tree, a child slot that has not been filled. If every empty node reports a hit,
traversal descends into it and the acceleration structure quietly stops accelerating — no crash, no
wrong pixel, just a performance cliff that is very hard to attribute. `tests/test_aabb.cpp:244-248`
(`EmptyBoxIsNeverHit`) is the regression test.

**Why it is a good interview story.** The two forms look equivalent, the buggy one is the one most
people have seen, the failure is silent and performance-only, and the fix costs exactly nothing —
one comparison either way, on a value already in a register. The comment ends with the right
summary: "Same cost, one fewer way to be wrong" (`include/geometry/aabb.hpp:188`).

### A.2 `1/0` is undefined behaviour even when IEEE-754 defines it

The slab test wants `invDir` to be `±inf` for an axis-parallel ray — that is load-bearing, not
accidental (`include/geometry/ray.hpp:43-47`). The obvious implementation is `1.0f / d`. On any
IEEE-754 machine that yields exactly the desired infinity with the correct sign.

It is still undefined behaviour. `[expr.mul]/4` makes division by zero UB *regardless of operand
type*: the IEEE guarantee is a property of the hardware, not of the C++ abstract machine
(`include/geometry/scalar.hpp:50-53`). UBSan's `float-divide-by-zero` check flags it, and per the
commit message that is how it was found.

The response was not to suppress the check but to produce the same value by construction
(`include/geometry/scalar.hpp:53-54`):

```cpp
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}
```
(`include/geometry/scalar.hpp:58-62`.)

`copysign` rather than a bare `kInfinity` because `-0.0` must give `-inf`, matching `1 / -0.0`
(`include/geometry/scalar.hpp:60`). The cost is one predictable branch per component, paid once per
ray in `invDirection` (`include/geometry/ray.hpp:48-51`) rather than once per node visited
(`include/geometry/scalar.hpp:56-57`). `tests/test_ray.cpp:42-51` asserts the infinities appear with
the right sign; `tests/test_ray.cpp:53-59` asserts finite components are unaffected.

**The interview point** is the distinction itself: "the hardware defines this" and "the abstract
machine defines this" are different claims, and only the second one survives an optimiser. A
compiler entitled to assume the divisor is non-zero may propagate that assumption backwards and
delete a later zero-check, or constant-fold the expression differently. The same distinction is
behind the `(&x)[i]` decision (`include/geometry/vec3.hpp:17-19`) and behind the ban on
`-ffast-math` (`CMakeLists.txt:81-84`) — three instances of one principle, which makes it a
genuinely defensible theme rather than a memorised trivium.

## Appendix B — is this really C++20?

No, not fully, and the build says so out loud.

`CMakeLists.txt:12-14` requests C++20 with `CMAKE_CXX_STANDARD_REQUIRED ON` and extensions off. But
`CMAKE_CXX_STANDARD_REQUIRED` does not guarantee a real C++20: for a compiler that only knows the
pre-release draft flag, CMake silently emits `-std=c++2a` and still reports "20"
(`CMakeLists.txt:27-31`). The installed toolchain is Apple clang 11.0.3 (2020), which has no C++20
standard library — `<concepts>`, `<span>`, `<ranges>`, `<numbers>` are all missing
(`CLAUDE.md`, Toolchain status).

Rather than let the build look conformant when it is not, `CMakeLists.txt:32-49` compiles a probe
that includes `<concepts>` and `<span>` and uses a `std::floating_point` constrained template. On
failure it emits a `message(WARNING)` naming the compiler and pointing at `CLAUDE.md`
(`:42-48`), and the configuration summary prints `C++ standard ....... 20 (DRAFT -- no C++20 library)`
(`CMakeLists.txt:132-136`).

The geometry core is unaffected in practice: nothing in `include/geometry/` or `src/geometry/` uses
a C++20 library facility. The features it does rely on — `constexpr` member functions, default
member initialisers, `inline constexpr` variables at namespace scope, `friend` hidden operators —
are all C++17 or earlier. **So the honest statement is: the geometry core is C++17-compatible code
compiled under a draft C++20 flag, and the project cannot claim C++20 until the Command Line Tools
are updated.** Homebrew already refuses to compile on this configuration, which will block GLFW,
the Vulkan SDK and ImGui (`CLAUDE.md`, Toolchain status).

That is a good answer to give an interviewer who asks, because the interesting part is not the
version number — it is that the build *detects and reports* the discrepancy instead of inheriting
it silently.

## Appendix C — what Phase 1 deliberately does not contain

Marked here so nothing in this document is mistaken for a description of code that exists.

- **Ray/triangle intersection** — no Möller–Trumbore, no Watertight, nothing. Deliberately out of
  scope for this milestone. `UNKNOWN — not yet implemented`.
- **BVH node type, construction, partitioning, traversal** — Phase 2.
  `UNKNOWN — not yet implemented`.
- **SAH** — only the cost *term* exists (`AABB::surfaceArea`,
  `include/geometry/aabb.hpp:69`) and the binning *helper* (`AABB::offset`, `:110`). The heuristic
  itself, bin sweeping, and cost evaluation are Phase 3. `UNKNOWN — not yet implemented`.
- **Configurable leaf size, max depth, split strategy** — Phase 3.
  `UNKNOWN — not yet implemented`.
- **Vulkan instance/device/pipelines/descriptor sets/compute** — Phases 7–8, and currently blocked
  on the toolchain. `UNKNOWN — not yet implemented`.
- **OBJ loading** — no parser in the tree. `UNKNOWN — not yet implemented`.
- **Benchmarks** — `benchmarks/results/` contains only `README.md`, a format and methodology
  specification. No measurement of any kind exists. Every performance statement in this document is
  therefore an argument, and is labelled as one.
