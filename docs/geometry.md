# Geometry Core

**Milestone:** Phase 1 — C++ Geometry Core
**Documented at:** `935c354` "Phase 1 review fixes: UB, preconditions, scale-invariance"
(built on `841777e` "Phase 1: geometry core with CMake and GoogleTest"; `a2a31a9` is scaffolding only)
**Scope:** everything under `include/geometry/` and `src/geometry/`.

Every behavioural claim is cited as `file:line` against the source at `935c354`. Where a code
comment asserts something the code cannot demonstrate — typically a performance argument — the
claim is presented as *reasoning* and marked `UNVERIFIED`. There are no benchmarks in this project:
`benchmarks/results/` contains only a format specification (`benchmarks/results/README.md:1-62`),
whose own rule is that a number not recorded there "does not exist and must be written as
`UNVERIFIED`".

> **One claim in the source violates that rule and needs the Implementation Engineer's attention.**
> `include/geometry/aabb.hpp:196-198` states that "a sweep of 3 million rays aimed at shared sibling
> faces found no difference with it removed." That is an empirical result with no file under
> `benchmarks/results/` behind it, and no committed test runs 3 million rays — the largest is a
> 20,000-ray property test (`tests/test_aabb_property.cpp:120`). Treated as `UNVERIFIED` throughout.

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

Appendices: [A — two bugs fixed](#appendix-a--two-bugs-fixed-in-this-milestone) ·
[B — is this really C++20?](#appendix-b--is-this-really-c20) ·
[C — what Phase 1 does not contain](#appendix-c--what-phase-1-deliberately-does-not-contain) ·
[D — changelog](#appendix-d--changelog)

---

## 1. What problem it solves

The geometry core supplies the vocabulary types and robust primitive operations that everything
above it — BVH construction, ray queries, collision queries, and eventually the Vulkan viewer — is
written in terms of. Concretely it answers five families of question:

| Question | Entry point |
|---|---|
| Where is this point / which way does this direction face, after a transform? | `transformPoint`, `transformVector`, `transformNormalWithInverse` (`include/geometry/mat4.hpp:119`, `:124`, `:140`) |
| What is the axis-aligned extent of this primitive / this set of primitives? | `Triangle::bounds` (`include/geometry/triangle.hpp:76`), `AABB::extend` (`include/geometry/aabb.hpp:58`), `Mesh::bounds` (`include/geometry/mesh.hpp:86`) |
| Does this ray hit this box, and at what distance? | `intersectRay` (`include/geometry/aabb.hpp:211`) |
| Do these two boxes overlap, and by how much? | `AABB::intersects`, `intersection` (`include/geometry/aabb.hpp:128`, `:166`) |
| How expensive is this box as a BVH node? | `AABB::surfaceArea` (`include/geometry/aabb.hpp:93`) |

It also solves a second, less obvious problem: **it makes the numerical behaviour of the system
explicit, testable, and — where it is a judgement call — switchable.** The infinities, NaNs,
undefined-behaviour traps and degenerate inputs that a spatial index will certainly encounter are
handled in named, unit-tested functions here rather than being rediscovered as mystery bugs during
traversal: `safeReciprocal` (`include/geometry/scalar.hpp:58`), the guarded `Vec3::operator/`
(`include/geometry/vec3.hpp:70-76`), `gamma` (`include/geometry/scalar.hpp:42`), `normalizeSafe`
(`include/geometry/vec3.hpp:117`), `Triangle::isDegenerate` (`include/geometry/triangle.hpp:64-72`).

## 2. Why it exists

**It is written by hand rather than taken from GLM or Embree.** `CLAUDE.md` makes this a
project-wide rule: geometry math, BVH construction, traversal, intersection and collision are
implemented in-house so the developer owns the explanation; GLM is permitted only on the rendering
side. The project doubles as interview preparation, so "I can't explain that, it's library code" is
a failure mode the constraint is designed to eliminate.

**It is deliberately dependency-free.** `CMakeLists.txt:107-109` states the rule and the build
enforces it: `bvh_geometry` (`CMakeLists.txt:112-115`) links only `bvh_project_options`
(`:125`), and no header under `include/geometry/` includes a Vulkan, GLFW or ImGui header. The
whole algorithmic core is therefore unit-testable on a machine with no GPU and no window system —
which matters here, because the development machine has an integrated Iris Plus GPU reached through
MoltenVK, and the Vulkan SDK is not yet installable (`CLAUDE.md`, Platform).

**It is the layer where BVH design decisions are already visible.** Several members exist only
because of Phase 2, each documented at the point of the decision:

- `AABB::offset` maps a point to `[0,1]` per axis and is documented as the SAH-binning bin-index
  function (`include/geometry/aabb.hpp:135-137`).
- `Triangle::centroid` exists because BVH partitioning sorts on centroids, not bounds
  (`include/geometry/triangle.hpp:84-86`).
- `Ray::tMax` is stored in the ray so traversal can shrink it as closer hits are found
  (`include/geometry/ray.hpp:21-24`).
- `Mesh::bounds()` is the *tight* bound specifically because "the SAH normalises each child's
  surface area against its parent's, so a root inflated by unreferenced vertices would shift every
  split decision" (`include/geometry/mesh.hpp:80-85`).
- `AABB::centroid`'s precondition is justified by a Phase 2 failure mode: "an empty range at a
  recursion boundary is exactly how you reach here" (`include/geometry/aabb.hpp:72-78`).
- `tests/test_aabb_property.cpp` names itself the future Phase 2 correctness oracle (`:6-7`).

None of those consumers exist yet. See [Appendix C](#appendix-c--what-phase-1-deliberately-does-not-contain).

## 3. How it works

Eight headers and two translation units. Dependencies flow strictly downward:

```
scalar.hpp          Scalar typedef, IEEE-754 assertion, gamma(n), safeReciprocal
   |
   +-- vec3.hpp     Vec3: arithmetic, guarded operator/, dot, cross, normalize, maxAxis
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

Only `mat4.cpp` and `mesh.cpp` are compiled units (`CMakeLists.txt:112-115`); everything else is
header-only `inline`/`constexpr`, so the small operations that appear in inner loops are visible to
the optimiser at every call site.

Five design choices set the character of the layer:

**(a) One scalar type, not a template.** `using Scalar = float;`
(`include/geometry/scalar.hpp:20`). A single typedef keeps a double-precision build reachable
without touching call sites, while the common case stays free of template noise
(`:8-10`). §10 discusses the float-vs-double argument itself.

**(b) Points, directions and normals are different things, at the type and the API level.** `Vec4`
carries `w` explicitly (`include/geometry/vec4.hpp:10-14`), and the three transform entry points are
separate functions rather than one function with a convention: `transformPoint` sets `w = 1` and
handles the perspective divide (`src/geometry/mat4.cpp:197-206`), `transformVector` sets `w = 0`
(`:208-211`), `transformNormalWithInverse` applies the inverse transpose of the upper-left 3×3
(`:213-220`).

**(c) Degenerate input is a supported case, not an error.** Zero-length vectors, zero-area
triangles, collinear points and flat boxes all have defined, finite behaviour. §14 is the full list.

**(d) Every tolerance is scale-invariant.** There is no absolute epsilon anywhere in a geometric
predicate. The ray/box error bound is relative (`gamma(3)`, `include/geometry/aabb.hpp:22`), the
triangle degeneracy test is a dimensionless aspect ratio
(`include/geometry/triangle.hpp:52-63`), and the matrix singularity test is a per-row ratio
(`src/geometry/mat4.cpp:127-142`). §9 collects the argument.

**(e) A two-tier error policy, with a build configuration that actually exercises both tiers.**
`CLAUDE.md` states it: the `Mesh` constructor **throws**, because that is the trust boundary where
untrusted file data enters; everywhere else a violated precondition is a programming error and
**asserts**. Since `-DNDEBUG` strips every assert, a Release-only test run never exercises a single
precondition — so both Debug and Release must pass before a milestone is done (`CLAUDE.md`, Build
and test; `README.md`, Building), the configuration summary prints whether asserts are live
(`CMakeLists.txt:150-154`), and `tests/test_preconditions.cpp` covers the asserting half.

## 4. Data structures used

| Type | Storage | Size (float `Scalar`) | Declared at |
|---|---|---:|---|
| `Vec3` | three named scalars `x, y, z` | 12 B | `include/geometry/vec3.hpp:23-53` |
| `Vec4` | four named scalars `x, y, z, w` | 16 B | `include/geometry/vec4.hpp:15-40` |
| `Mat4` | `Scalar m[4][4]`, indexed `m[column][row]` | 64 B | `include/geometry/mat4.hpp:21-64` |
| `Ray` | `origin`, `direction`, `tMin`, `tMax` | 32 B | `include/geometry/ray.hpp:25-36` |
| `Plane` | unit-or-zero `normal` plus scalar `d` | 16 B | `include/geometry/plane.hpp:14-45` |
| `Triangle` | three explicit `Vec3` vertices | 36 B | `include/geometry/triangle.hpp:22-95` |
| `AABB` | `min`, `max` corners | 24 B | `include/geometry/aabb.hpp:43-151` |
| `Mesh` | `vector<Vec3>` positions + `vector<uint32_t>` indices + **two** cached `AABB`s | see §7 | `include/geometry/mesh.hpp:32-110` |

Five storage decisions are load-bearing.

**`Vec3` stores named members, and `operator[]` is a conditional chain — not `(&x)[i]`.**

```cpp
constexpr Scalar& operator[](int i) {
    assert(i >= 0 && i < 3);
    return i == 0 ? x : (i == 1 ? y : z);
}
```
(`include/geometry/vec3.hpp:32-35`, const overload at `:36-39`.)

`(&x)[i]` is undefined behaviour: pointer arithmetic is only defined within a single object, and
three separate non-array members are not an array however they are laid out. The conditional form
is well-defined, and because a conditional expression over lvalues is itself an lvalue it still
yields a genuine reference (`include/geometry/vec3.hpp:15-22`). The claim that optimisers reduce
this to the same one or two instructions (`:20-22`) is a reasoned expectation — **UNVERIFIED**, no
codegen inspection is recorded. The same pattern is used for `Vec4`
(`include/geometry/vec4.hpp:25-32`). Indexed access is not decoration: `intersectRay` reaches
`box.min[axis]` on a runtime axis (`include/geometry/aabb.hpp:217-218`), and BVH split-axis
selection will do the same.

**`Mat4` is column-major.** `m[c][r]` throughout (`include/geometry/mat4.hpp:11-17`), with
translation in the fourth *column* (`src/geometry/mat4.cpp:10-16`). The stated reason is GLSL/SPIR-V
interop: a `Mat4` can be memcpy'd into a Vulkan uniform buffer and consumed with no transpose
(`include/geometry/mat4.hpp:13-15`). The convention is column-vector, so a transform is `M * v` and
"first A, then B" composes as `B * A` (`:19-20`), which `tests/test_mat4.cpp:99` pins down. The
admitted cost is that memory order no longer matches how a matrix is written on paper, which is why
the code says `m[c][r]` everywhere rather than using a flat 16-element array (`:15-17`). The one
place the code works against its own storage order is `invert`, which copies into row-indexed
locals because elimination is naturally row-oriented, then transposes back
(`src/geometry/mat4.cpp:109-125`, `:184-185`).

**`AABB` default-constructs to an inverted interval.**

```cpp
Vec3 min{kInfinity};
Vec3 max{-kInfinity};
```
(`include/geometry/aabb.hpp:44-45`.)

This is the single most consequential representation choice in the file. It makes `extend`
branch-free in both overloads (`:58-68`): `min(+inf, p) == p` and `max(-inf, p) == p`, so the first
point merged into an empty box produces exactly that point's degenerate box with no empty-check
(`:38-42`). Symmetrically, merging an *empty* box into a real one is a no-op for free (`:64-66`) —
precisely what bottom-up bounds propagation in a BVH needs, and `tests/test_aabb.cpp:48-56` asserts
it under that name. A zero-initialised box would instead wrongly contain the origin, silently
inflating every bound in the tree toward `(0,0,0)`.

`isEmpty()` is then "inverted on at least one axis" (`include/geometry/aabb.hpp:56`). The header
distinguishes *empty* (not a region at all) from *degenerate* (a point or flat plane — a real
zero-volume region that legitimately participates in intersection tests) at `:53-55`. Tests enforce
the distinction: `tests/test_aabb.cpp:65-72` (flat box has real surface area, zero volume) and
`:235-242` (flat box is still hittable) versus `:244-248` (empty box is never hit).

The sentinel is not free — it interacts with arithmetic that assumes finite corners, and all three
such places are handled explicitly. `surfaceArea` and `volume` guard and return 0 (`:93-103`);
`centroid` and `offset` declare a non-empty **precondition** and assert it (`:72-84`, `:138-139`),
with `centroid`'s comment naming the exact Phase 2 failure it prevents — "a NaN centroid then
propagates silently into a bin index and corrupts the tree instead of crashing" (`:74-78`). Both
asserts have death tests (`tests/test_preconditions.cpp:50-59`).

**`Mesh` carries two bounds, and they mean different things.**

```cpp
AABB bounds_;        // referenced geometry (tight)
AABB vertexBounds_;  // whole vertex array (conservative)
```
(`include/geometry/mesh.hpp:108-109`, both computed together at `src/geometry/mesh.cpp:28-34`.)

`bounds()` is the tight bound over vertices actually *referenced* by the index buffer, and the
header explains why that is the default rather than the cheaper vertex-array bound: "The SAH
normalises each child's surface area against its parent's, so a root inflated by unreferenced
vertices would shift every split decision and make benchmark numbers depend on how clean the input
file happens to be" (`include/geometry/mesh.hpp:80-85`). `vertexBounds()` is the conservative
vertex-array extent, offered for uses like sizing a GPU vertex buffer (`:88-90`). The relationship —
conservative always contains tight — is asserted directly at `tests/test_mesh.cpp:101-116`.

**`Mesh` is indexed, not an array of `Triangle`.** Positions and a `uint32_t` index buffer
(`include/geometry/mesh.hpp:106-107`), with triangles materialised on demand (`:69-75`). Two reasons
are given (`:19-27`): the memory/cache argument, and that this is the layout `vkCmdDrawIndexed`
wants, so the render path can upload both buffers with no repacking. The memory arithmetic in that
comment deserves a correction — see §7.

## 5. Algorithms used

### 5.1 Ray/AABB intersection — the slab method

Source: `include/geometry/aabb.hpp:211-250`, documented at `:170-210`.

**The idea.** An axis-aligned box is the intersection of three axis-aligned *slabs*. For each slab,
compute the two parameters at which the ray crosses its planes and intersect a running
`[tEnter, tExit]` interval with them. The ray hits iff that interval is still non-empty after all
three axes.

```cpp
Scalar t0 = tMin;
Scalar t1 = tMax;
for (int axis = 0; axis < 3; ++axis) {
    Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
    Scalar tFar  = (box.max[axis] - origin[axis]) * invDir[axis];
    if (invDir[axis] < Scalar(0)) { /* swap */ }
    tFar *= kRayBoxWidening;
    t0 = tNear > t0 ? tNear : t0;
    t1 = tFar  < t1 ? tFar  : t1;
    if (t1 < t0) return false;
}
```
(`include/geometry/aabb.hpp:213-246`.)

`t0` starts at the ray's own `tMin` and `t1` at its `tMax` (`:213-214`), so the ray's valid range is
just a fourth interval in the same intersection — no separate range check. On success `t0` is
returned as the entry distance (`:248`), which for a ray originating inside the box is `tMin`, not a
negative backward distance (`:206-207`).

**Cost.** The header states 6 multiplies, 6 adds and a handful of comparisons, with no divides
because the caller supplies the reciprocal direction (`:175-176`). Reading the loop: 2 subtractions
and 2 multiplies per axis for the `t` values — 6 and 6 over three axes, so the comment's arithmetic
checks out — plus the widening multiply at `:237`, which the comment block counts separately: "it is
3 of the 9 multiplies in this function" (`:202`). Actual instruction count and throughput are
**UNVERIFIED**.

**Hazard 1 — axis-parallel rays and `0 * inf == NaN`.** A zero direction component makes `invDir`
infinite (`include/geometry/ray.hpp:48-51`) and the products become ±inf, which compare correctly.
But if the origin lies *exactly* on a slab plane the numerator is zero too, and `0 * inf` is NaN
(`include/geometry/aabb.hpp:180-186`). Every comparison against NaN is false, so the code uses
explicit ternaries rather than `std::max`/`std::min`:

```cpp
t0 = tNear > t0 ? tNear : t0;
t1 = tFar  < t1 ? tFar  : t1;
```
(`:241-242`, with "Do NOT replace with `std::max`/`std::min`: their NaN behaviour is not guaranteed
to match" at `:239-240`.)

When `tNear` is NaN, `tNear > t0` is false and `t0` keeps its existing value: the degenerate axis
becomes a no-op and does not constrain the interval. That is conservative — it can report a hit the
exact arithmetic would not — but it can never produce a false *miss*, the failure mode that matters
for an acceleration structure. Test: `tests/test_aabb.cpp:207-216`, whose ray originates exactly on
the `y = 1` face with no `y` component.

**Hazard 2 — rounding at grazing angles, and the `gamma(3)` widening.** Each `t` is a subtract
followed by a multiply, so it carries relative error bounded by `gamma(3)`
(`include/geometry/aabb.hpp:188-192`). `gamma(n)` is Higham's bound as used in PBRT
(`include/geometry/scalar.hpp:37-45`); for float, `gamma(3) ≈ 1.8e-7`. Left uncorrected, a ray
passing exactly through the shared face between two sibling BVH nodes could be rejected by both — a
visible crack. Widening the exit distance makes the test err toward reporting a hit.

The widening is a **named, build-switchable constant**:

```cpp
#if BVH_CONSERVATIVE_RAY_BOX
inline constexpr Scalar kRayBoxWidening = Scalar(1) + Scalar(2) * gamma(3);
#else
inline constexpr Scalar kRayBoxWidening = Scalar(1);
#endif
```
(`include/geometry/aabb.hpp:17-25`, applied at `:237`.) Defining it as a constant rather than an
`#if` at the use site means disabling it "changes one value and nothing else" (`:13-14`), and
because the disabled value is exactly 1 "the multiply folds away entirely" (`:235-236`). The macro
is set on the shared options target so every translation unit agrees — a header-inline function
compiled with different values in different TUs would be an ODR violation
(`CMakeLists.txt:88-95`), which is a genuinely subtle build-correctness point. The option is exposed
as `BVH_CONSERVATIVE_RAY_BOX`, default ON (`CMakeLists.txt:52-53`), and reported in the
configuration summary (`:149`).

**The source is unusually honest about this widening's status** (`include/geometry/aabb.hpp:194-204`):
it follows PBRT's error analysis (3rd ed., §3.9), which is sound; but no test in the repository
demonstrates a failure it prevents; its real justification arrives in Phase 2, where watertightness
matters between a node's bound and the triangle test inside it, and there is no triangle test yet.
It is kept as the safe default and made switchable so Phase 9 can price it. The comment closes with
"Do not restate the PBRT rationale as a measured result until there is a benchmark behind it."
One sentence inside that block — the 3-million-ray sweep at `:196-198` — is itself an unrecorded
measurement; see §13.1.

Only `tFar` is widened; `tNear` is left alone. That closes the exit side, where the sibling-crack
arises, and matches the published PBRT form.

**Why the swap tests the sign of `invDir` and not `tNear > tFar`.** See
[Appendix A](#appendix-a--two-bugs-fixed-in-this-milestone). This is the most interesting single
line in the milestone.

**Why the final comparison is strict.** `if (t1 < t0) return false;` (`:245`) rather than `<=`, so a
flat box with `min == max` on an axis produces `t1 == t0` and still reports a hit (`:244`). Pinned
by `tests/test_aabb.cpp:235-242`.

### 5.2 Division that cannot be undefined behaviour

Two functions produce IEEE-754 infinities without ever executing a division by zero.

```cpp
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}
```
(`include/geometry/scalar.hpp:58-62`, used by `invDirection` at `include/geometry/ray.hpp:48-51`.)

```cpp
inline Vec3 operator/(const Vec3& v, Scalar s) {
    if (s == Scalar(0)) {
        const Scalar inf = safeReciprocal(s);
        return {v.x * inf, v.y * inf, v.z * inf};
    }
    return {v.x / s, v.y / s, v.z / s};
}
```
(`include/geometry/vec3.hpp:70-76`.)

`[expr.mul]/4` makes `x / 0` undefined behaviour regardless of operand type — the IEEE guarantee is
a property of the hardware, not of the abstract machine (`include/geometry/scalar.hpp:50-53`). The
slab test genuinely wants those infinities, so the goal is not to avoid them but to obtain them
legally (`:53-54`). `copysign` rather than a bare `kInfinity` because `-0.0` must give `-inf`,
matching `1 / -0.0` (`:60`) — and the sign determines which slab plane is the entry plane, so
getting it wrong on a negative-zero direction component would flip entry and exit for that axis.

`Vec3::operator/` reproduces the same semantics component-wise: `x * (±inf)` is `±inf` with the
correct sign, and `0 * inf` is NaN — exactly what `x/0` and `0/0` produce
(`include/geometry/vec3.hpp:60-64`). It also deliberately keeps a *true divide* on the non-zero
path rather than multiplying by a reciprocal: multiply-by-reciprocal is faster but adds a second
rounding step, and overflows to infinity for a denormal divisor where the true quotient is finite
(`:66-69`). This is not a hot path, so correctness wins. The consequence is that `operator/` and the
now out-of-line `operator/=` (`:44`, defined at `:80-83`) are no longer `constexpr`; nothing in the
core needed them to be.

This is what makes `normalize`'s release-build behaviour well-defined: the assert is the debug
guard, but with `NDEBUG` the division must still not be UB, and `operator/` guarantees that
(`include/geometry/vec3.hpp:101-112`). Both halves are tested — the assert by a death test
(`tests/test_preconditions.cpp:61-63`) and the release fallback by an `#ifdef NDEBUG`-guarded test
asserting NaN rather than UB (`:78-90`), plus a direct IEEE-conformance test of the operator
(`:92-100`).

### 5.3 4×4 inverse — Gauss-Jordan with **scaled** partial pivoting

`src/geometry/mat4.cpp:106-188`, documented at `include/geometry/mat4.hpp:88-102`. This is the
subtlest numerical decision in the core, and it is not the obvious one.

The algorithm operates on the augmented system `[A | I]`, in row-indexed local copies because
elimination is naturally row-oriented (`src/geometry/mat4.cpp:107-111`). Before eliminating, it
records the largest absolute entry of **each row** and rejects an all-zero row outright as
rank-deficient at any scale (`:115-125`):

```cpp
Scalar rowScale[4];
for (int r = 0; r < 4; ++r) {
    Scalar rowMax = Scalar(0);
    for (int c = 0; c < 4; ++c) { lhs[r][c] = a.m[c][r]; rowMax = std::fmax(rowMax, std::fabs(lhs[r][c])); }
    if (rowMax == Scalar(0)) return false;
    rowScale[r] = rowMax;
    rhs[r][r] = Scalar(1);
}
```

Each candidate pivot is then judged as a **ratio against the largest entry in its own row**, and the
tolerance is a pure precision figure with no units:

```cpp
constexpr Scalar kPivotTolerance = std::numeric_limits<Scalar>::epsilon() * Scalar(8);
...
const Scalar ratio = std::fabs(lhs[r][col]) / rowScale[r];
if (ratio > best) { best = ratio; pivot = r; }
...
if (pivot < 0 || best <= kPivotTolerance) return false;
```
(`:142`, `:147-156`.) When rows are swapped, their scales travel with them — "The scales are a
property of the rows, so they travel with them" (`:163-164`).

**Why scaled and not plain partial pivoting.** Plain partial pivoting compares raw magnitudes, which
makes the choice depend on how each row happens to be scaled: multiply one equation through by
`1e6` and it wins every pivot contest without being any better conditioned
(`src/geometry/mat4.cpp:131-134`).

**Why not a whole-matrix (norm-relative) threshold — the interesting part.** The obvious fix for an
absolute cutoff is to scale it by the matrix's largest entry. That is wrong, and *specifically*
wrong for the matrices this project uses: a homogeneous transform always carries `m[3][3] == 1`, so
`scaling(1e-21)` has a global maximum of 1 while every pivot that matters is `1e-21`. A matrix-norm
threshold rejects it, even though it is perfectly invertible
(`src/geometry/mat4.cpp:134-138`; header summary at `include/geometry/mat4.hpp:96-101`).
Row-relative ratios are dimensionless and in `[0, 1]`, so the tolerance has no units to get wrong
(`:140-141`). The regression test is `tests/test_mat4.cpp:241-254`
(`InvertAcceptsUniformlyTinyButInvertibleMatrices`), which accepts `scaling(1e-21)` and still
rejects genuinely rank-deficient matrices at both `1e-21` and `1e21` scale.

**Why Gauss-Jordan at all.** The header states the tradeoff: cofactor expansion is the usual choice
for a fixed 4×4 and is faster, but less numerically stable, and this is not a hot path since
transforms are built at most once per frame, not once per ray
(`include/geometry/mat4.hpp:90-94`). Stability matters more here because the inverse brings rays
into object space, where error becomes a wrong intersection rather than a slightly wrong pixel.
The relative speed of the two is **UNVERIFIED**.

`inverse()` (`src/geometry/mat4.cpp:189-195`) is the convenience wrapper: asserts in debug, returns
identity for a singular matrix in release. The header says to prefer `invert()` where failure is
possible (`include/geometry/mat4.hpp:104-105`).

### 5.4 Normal transformation by inverse transpose

```cpp
Vec3 transformNormalWithInverse(const Mat4& inverseTransform, const Vec3& n) {
    const Mat4& i = inverseTransform;
    return {i.m[0][0]*n.x + i.m[0][1]*n.y + i.m[0][2]*n.z,
            i.m[1][0]*n.x + i.m[1][1]*n.y + i.m[1][2]*n.z,
            i.m[2][0]*n.x + i.m[2][1]*n.y + i.m[2][2]*n.z};
}
```
(`src/geometry/mat4.cpp:213-220`.)

Verified against the storage convention: with `i.m[c][r]` meaning column `c`, row `r`, the element
of `iᵀ` at row `r`, column `c` is `i.m[r][c]`, so `(iᵀn)_r = Σ_c i.m[r][c]·n_c` — exactly what the
expression computes. The transpose is never materialised (`:214-215`).

A normal is a covector: under non-uniform scale it does not transform like a direction. Scaling `x`
by 2 *halves* the `x` component of a surface normal rather than doubling it, so using
`transformVector` here is the classic bug that leaves normals non-perpendicular to their surfaces
(`include/geometry/mat4.hpp:126-131`). The function takes the already-inverted matrix so a caller
transforming many normals pays for one inverse, not one per normal (`:133-134`). The header also
warns that **the result is not unit length even for a unit input** — the inverse transpose preserves
perpendicularity, not magnitude, and a unit normal through the inverse of a uniform 4× scale comes
back with length 1/4 (`:136-139`), which `tests/test_mat4.cpp:256-266` pins numerically.
`tests/test_mat4.cpp:158-182` proves the core distinction: it builds a tangent/normal pair, applies
a non-uniform scale, and asserts the inverse-transpose result stays perpendicular to the transformed
tangent while the `transformVector` result does not.

### 5.5 Point transformation — three cases, not two

```cpp
if (r.w != Scalar(1) && r.w != Scalar(0)) { /* perspective divide */ }
return r.xyz();
```
(`src/geometry/mat4.cpp:197-206`, condition at `:201`.) The header enumerates the cases
(`include/geometry/mat4.hpp:114-118`): `w == 1` skips the divide (the affine fast path), and `w == 0`
also skips it and returns the raw xyz, because a point projecting to `w == 0` lies on the eye plane
and has no finite projected position — so the raw direction-like vector is at least finite where
dividing would give infinities or NaN. Tested from `Mat4` (`tests/test_mat4.cpp:183-198`) and from
`Mesh` (`tests/test_mesh.cpp:151-168`).

### 5.6 Rotation by Rodrigues' formula

`R = I cos(t) + [k]_x sin(t) + k kᵀ (1 - cos(t))` (`src/geometry/mat4.cpp:27-52`). The axis is
normalised with `normalizeSafe`, and a zero-length axis short-circuits to identity rather than
producing NaN (`:31-32`, tested at `tests/test_mat4.cpp:86`). `rotationX/Y/Z` are thin wrappers
(`:54-56`).

### 5.7 Triangle degeneracy — a dimensionless aspect-ratio test

```cpp
bool isDegenerate(Scalar tol = kEpsilon) const {
    const Scalar longestEdgeSq =
        std::fmax(lengthSquared(edge01()),
                  std::fmax(lengthSquared(edge02()), lengthSquared(v2 - v1)));
    if (longestEdgeSq == Scalar(0)) return true;
    const Scalar limit = tol * longestEdgeSq;
    return lengthSquared(normalUnnormalized()) <= limit * limit;
}
```
(`include/geometry/triangle.hpp:64-72`, reasoning at `:52-63`.)

**Derivation, verified against the code.** `normalUnnormalized()` is `cross(v1-v0, v2-v0)` (`:37`)
whose magnitude is twice the triangle area (`:33-34`). Let `L` be the longest edge length and `h`
the triangle's height above that edge. Then `|cross| = 2·area = L·h`. The predicate compares
`|cross|² ≤ (tol·L²)²`, i.e. `|cross| ≤ tol·L²`, i.e. `L·h ≤ tol·L²`, i.e.

```
h / L  ≤  tol
```

**a pure aspect ratio, dimensionless and invariant under uniform scaling.** Squaring both sides is
what avoids the square root (`:63`). The all-vertices-coincident case has no edge at all and is
short-circuited to `true` (`:68-69`), which also prevents `limit` being zero.

**Why an absolute-area form would be wrong.** The header spells it out (`:52-58`): with an absolute
tolerance of `1e-6`, a perfectly healthy triangle with `1e-3` edges has area `5e-7` and would be
flagged, and a unit-scale mesh of a million triangles has a mean triangle area near `6e-6` — right
at the threshold. "A degeneracy predicate that fires on a tenth of a valid mesh is worse than none."

Three tests pin the property directly: a small-but-healthy triangle of area `5e-7` is *not* flagged
and the same shape agrees at unit and `1e5` scale (`tests/test_triangle.cpp:110-123`); a needle-thin
sliver *is* flagged at `1e-3`, `1` and `1e5` scale (`:125-133`); and for an arbitrary triangle the
verdict is unchanged across five uniform scalings spanning eight orders of magnitude (`:135-145`).

**Consequence for callers:** `tol` is not an area. It is a shape ratio, and
`Mesh::countDegenerateTriangles(Scalar tol = kEpsilon)` passes it straight through
(`include/geometry/mesh.hpp:95`, `src/geometry/mesh.cpp:36-43`).

### 5.8 Mesh index validation at the boundary

`src/geometry/mesh.cpp:8-26`. The constructor rejects an index count that is not a multiple of 3
(`:10-13`) and any index `>= positions_.size()` (`:16-24`), throwing `std::invalid_argument` naming
the offending index and slot (`:20-22`). One `O(n)` pass at construction buys an unchecked inner
loop for the lifetime of the mesh (`:14-15`), and `Mesh::triangle` relies on that by indexing both
vectors with unchecked `operator[]` (`include/geometry/mesh.hpp:69-75`). Exceptions rather than
assertions are used because this is the boundary where untrusted file data enters (`:39-41`).

The comparison **widens the index rather than narrowing the count** —
`static_cast<std::size_t>(indices_[i]) >= positions_.size()` — because narrowing would wrap for a
vertex buffer larger than 2³² and silently accept bad indices (`src/geometry/mesh.cpp:17-19`). A
small but real correctness detail on a 64-bit platform.

The caller-supplied triangle index `i` is *outside* that invariant, so `triangle(i)` and
`triangleIndices(i, …)` assert it independently (`include/geometry/mesh.hpp:60`, `:70`), justified by
"Phase 2 will call this from a loop bounded by a node's primitive range, which is exactly where an
off-by-one would land" (`:53-57`). Three death tests cover it (`tests/test_preconditions.cpp:32-48`).

## 6. Time complexity

Everything in the core is either constant-time or a single linear pass. `n` = vertex count,
`m` = triangle count, `i` = index count (`= 3m`).

| Operation | Complexity | Notes / citation |
|---|---|---|
| `Vec3`/`Vec4` arithmetic, `dot`, `cross`, `length` | O(1) | fixed 3/4 components |
| `Vec3::operator/` | O(1), one branch | `include/geometry/vec3.hpp:70-76` |
| `Vec3::operator[]` | O(1) | 1–2 comparisons; constant index folds at compile time (`:32-39`) |
| `Mat4 * Mat4` | O(1), 64 multiply-adds | `src/geometry/mat4.cpp:58-69` |
| `Mat4 * Vec4` | O(1), 16 multiply-adds | `src/geometry/mat4.cpp:71-77` |
| `determinant` | O(1) | 12 2×2 minors then 6 products (`:86-104`) |
| `invert` | O(1); a row-scale pass, then 4 pivot passes × (pivot search + 4 rows × 4 cols) | `src/geometry/mat4.cpp:112-185` |
| `transformPoint` / `transformVector` / `transformNormalWithInverse` | O(1) | `src/geometry/mat4.cpp:197-220` |
| `AABB::extend`, `merge`, `intersection`, `surfaceArea`, `volume`, `contains`, `intersects`, `offset`, `longestAxis`, `centroid` | O(1) | `include/geometry/aabb.hpp:58-168` |
| `intersectRay` (slab) | O(1) — exactly 3 iterations, early-out at `:245` | `include/geometry/aabb.hpp:211-250` |
| `invDirection` | O(1), 3 branches | `include/geometry/ray.hpp:48-51` |
| `Triangle::bounds`, `centroid`, `normal`, `area` | O(1) | `include/geometry/triangle.hpp:37-89` |
| `Triangle::isDegenerate` | O(1); 3 `lengthSquared`, 2 `fmax`, 1 cross, no `sqrt` | `include/geometry/triangle.hpp:64-72` |
| `Mesh` construction | **O(n + i)** — one validation pass over indices, then `recomputeBounds` | `src/geometry/mesh.cpp:10-25` |
| `Mesh::recomputeBounds` | **O(n + i)** — one pass over indices for the tight bound, one over positions for the conservative one | `src/geometry/mesh.cpp:28-34` |
| `Mesh::countDegenerateTriangles` | O(m) | `src/geometry/mesh.cpp:36-43` |
| `Mesh::transform` | O(n) transform + O(n + i) bounds refresh | `src/geometry/mesh.cpp:45-48` |
| `Mesh::triangle(i)` | O(1), three indirect loads | `include/geometry/mesh.hpp:69-75` |
| `Mesh::bounds()` / `vertexBounds()` | **O(1)** — both cached | `include/geometry/mesh.hpp:86`, `:90` |

There is no super-linear algorithm anywhere in Phase 1. Note that `recomputeBounds` is two passes
over two different arrays — still linear, but not one traversal.

BVH construction and traversal complexity: `UNKNOWN — not yet implemented`.

## 7. Space complexity

Per-object, assuming `Scalar = float` (4 B) and 4-byte alignment:

| Type | Bytes | Derivation |
|---|---:|---|
| `Vec3` | 12 | 3 × 4 (`include/geometry/vec3.hpp:24-26`) |
| `Vec4` | 16 | 4 × 4 (`include/geometry/vec4.hpp:16-19`) |
| `Mat4` | 64 | 16 × 4 (`include/geometry/mat4.hpp:23`) |
| `Ray` | 32 | 2 × `Vec3` + 2 scalars (`include/geometry/ray.hpp:26-29`) |
| `Plane` | 16 | `Vec3` + scalar (`include/geometry/plane.hpp:15-16`) |
| `Triangle` | 36 | 3 × `Vec3` (`include/geometry/triangle.hpp:23-25`) |
| `AABB` | **24** | 2 × `Vec3` (`include/geometry/aabb.hpp:44-45`) |

`Mesh` holds `12n + 4i + 48` bytes of payload — the `+48` being two cached `AABB`s
(`include/geometry/mesh.hpp:106-109`) — plus three `std::vector` control blocks. With `i = 3m` that
is `12n + 12m` bytes of buffer data. For a closed manifold where each vertex is shared by about six
triangles, Euler's relation gives `n ≈ m/2`, so roughly **`6m + 12m = 18m` bytes**, versus `36m` for
an unpacked `vector<Triangle>` — a **2× saving**.

The source comment at `include/geometry/mesh.hpp:20-23` says "roughly 12 bytes + 12 bytes of indices
per triangle versus 36 bytes unpacked", which understates the win: 12 B/triangle of vertex data
corresponds to `n ≈ m`, whereas the same comment's own "~6 triangles per vertex" premise gives
`n ≈ m/2` and therefore 6 B/triangle. **The comment's conclusion is right and its arithmetic is
conservative** (1.5× stated, 2× actual). See §13.

All operations use O(1) auxiliary space. `invert` allocates two 4×4 stack arrays plus a 4-element
row-scale array (`src/geometry/mat4.cpp:112-114`); nothing in the core heap-allocates except the
`Mesh` vectors, which are moved in rather than copied (`include/geometry/mesh.hpp:36-37`,
`src/geometry/mesh.cpp:9`).

## 8. Important invariants

**I1 — `Mesh` index buffer well-formedness.** `indices.size()` is a multiple of 3, and every index
is `< positions.size()`. Stated at `include/geometry/mesh.hpp:29-31`, enforced by throwing at
`src/geometry/mesh.cpp:10-24`, tested at `tests/test_mesh.cpp:84`, `:88`, `:93`. This licenses
unchecked indexing in `Mesh::triangle` (`include/geometry/mesh.hpp:69-75`) and in `recomputeBounds`
(`src/geometry/mesh.cpp:30`). No public member can add or reorder indices, so it holds for the
object's lifetime.

**I2 — both cached bounds are always current, and `vertexBounds()` always contains `bounds()`.**
Established in the constructor (`src/geometry/mesh.cpp:25`), re-established after the only mutating
operation (`:47`), both computed together (`:28-34`). Containment asserted at
`tests/test_mesh.cpp:115`.

**I3 — an empty `AABB` has `min > max` on at least one axis, and every *defined* measure of it is
zero.** `surfaceArea()` and `volume()` return 0 (`include/geometry/aabb.hpp:93-103`) so SAH cost
sums stay finite (`:92`). `centroid()` and `offset()` are *undefined* on an empty box and say so as
a precondition (`:72-80`, `:138-139`).

**I4 — merging an empty `AABB` is the identity.** A direct consequence of the sentinel
(`include/geometry/aabb.hpp:63-68`). Tested under the name that explains why it matters:
`tests/test_aabb.cpp:48-56`, "Relied on by bottom-up bounds propagation in BVH construction".

**I5 — `intersectRay` never reports a false miss.** Both numerical safeguards push in the
conservative direction: the NaN-tolerant ternaries leave the interval unconstrained on a degenerate
axis (`include/geometry/aabb.hpp:241-242`), and the widening only pushes `tExit` outward (`:237`).
This is a *tested* property, not only an argued one: a randomised sweep asserts that any ray
demonstrably passing through a box is reported as a hit (`tests/test_aabb_property.cpp:81-111`), and
a second asserts that the widened test never removes a hit the exact test found (`:113-143`).

**I6 — an empty `AABB` is never hit by any ray.** `include/geometry/aabb.hpp:209-210`. Tested for one
direction at `tests/test_aabb.cpp:244-248` and over 5,000 randomised directions including
axis-aligned and negative ones at `tests/test_aabb_property.cpp:159-169`. This is *not* free — see
[Appendix A](#appendix-a--two-bugs-fixed-in-this-milestone).

**I7 — `transformRay` preserves `[tMin, tMax]` and does not renormalise the direction.**
`include/geometry/ray.hpp:59-61`, so a point at parameter `t` in one space is the same physical
point at the same `t` in the other. Tested at `tests/test_ray.cpp:71-77` and `:79-91`.

**I8 — every `Plane` factory produces a unit-or-zero normal; the raw constructor guarantees
nothing.** Both `fromPointNormal` and `fromPoints` use `normalizeSafe`
(`include/geometry/plane.hpp:26-29`, `:34-37`), so a degenerate input gives a zero normal rather than
NaN, and `isDegenerate()` (`:44`) is the way to ask. `signedDistance` is only a true distance when
the normal is unit (`:39-41`).

**I9 — `maxAxis` ties resolve to the lowest index.** `include/geometry/vec3.hpp:141-144`, stated at
`:138-140` to keep BVH split-axis selection deterministic for symmetric bounds, which matters for
reproducible benchmarks. Tested for the cube case at `tests/test_aabb.cpp:84-85`.

**I10 — set semantics for the empty box are consistent, not arbitrary.** `AABB::contains(AABB)`
returns true for an empty argument and `intersects` returns false; both follow the same rule —
"(empty subset A) is true, while (empty intersect A) is empty"
(`include/geometry/aabb.hpp:116-119`, `:125-127`). Both tested (`tests/test_aabb.cpp:104`, `:113`).

**I11 — `tEnter` is always within `[tMin, tMax]` and finite when a hit is reported.** Asserted over
5,000 randomised rays at `tests/test_aabb_property.cpp:145-157`.

**I12 — `invert` is scale-invariant in both directions.** It accepts a uniformly tiny but invertible
matrix and rejects a rank-deficient one at any scale, because the pivot test is a per-row ratio
(`src/geometry/mat4.cpp:147-156`). Tested at `tests/test_mat4.cpp:241-254`.

**I13 — error-reporting policy.** Throw at the trust boundary (`Mesh`'s constructor), assert
everywhere else. Stated in `CLAUDE.md` (Build and test) and in `tests/test_preconditions.cpp:1-11`;
the asserting half is only real if the Debug configuration is actually run, which is why both
configurations are required before a milestone is done.

## 9. Numerical assumptions

**IEEE-754 is required, and the requirement is checked, not assumed.**

```cpp
static_assert(std::numeric_limits<Scalar>::is_iec559,
              "geometry core requires IEEE-754 floating point");
```
(`include/geometry/scalar.hpp:26-27`.)

The specific properties relied upon are named at `:22-25`: infinities that compare correctly, NaNs
that make every comparison false, and signed zero. Each is used:

- **Infinities that compare correctly** — the empty-`AABB` sentinel
  (`include/geometry/aabb.hpp:44-45`) and axis-parallel `invDir` (`include/geometry/ray.hpp:48-51`).
- **NaNs that falsify every comparison** — the `t0`/`t1` ternaries
  (`include/geometry/aabb.hpp:241-242`).
- **Signed zero** — `copysign` in `safeReciprocal` (`include/geometry/scalar.hpp:61`), so a direction
  component of `-0.0` yields `-inf` and the slab is entered from the correct side. Tested at
  `tests/test_preconditions.cpp:92-100`.

**`-ffast-math` must never be enabled.** `CMakeLists.txt:83-86` states this and the build never adds
it; `CLAUDE.md` repeats it as a project rule. Fast-math permits the compiler to assume NaN and
infinity never occur, which does not make the NaN-tolerant slab test faster — it makes it *wrong*.

**Float carries about 7 decimal digits, and the code is written accordingly.**
`include/geometry/scalar.hpp:14-16` says so directly: the precision cost of `float` is why the
intersection routines are written to be robust rather than to assume exact arithmetic.

**Every geometric tolerance is scale-invariant. This is the through-line of the numerical design.**
`kEpsilon = 1e-6` (`include/geometry/scalar.hpp:35`) exists for "are these two quantities the same"
comparisons and is explicitly *not* used inside intersection tests, because a single global epsilon
is meaningless across magnitudes (`:32-34`). The three geometric predicates that need a threshold
each use a relative one:

| Predicate | Threshold | Citation |
|---|---|---|
| Ray/box error bound | relative — `gamma(3)`, a bound on relative error after 3 ops | `include/geometry/scalar.hpp:37-45`; `include/geometry/aabb.hpp:22` |
| Triangle degeneracy | dimensionless — height / longest edge | `include/geometry/triangle.hpp:52-63` |
| Matrix singularity | per-row ratio in `[0,1]` — *not* absolute, and deliberately *not* matrix-norm-relative | `src/geometry/mat4.cpp:127-142` |

The last is the one worth dwelling on: a whole-matrix threshold looks like the obvious
scale-invariant fix and is wrong for homogeneous transforms, because `m[3][3] == 1` dominates the
norm (§5.3).

**Comparison policy.** `Vec3::operator==` is exact bitwise-value comparison, documented as suitable
for exactly-representable test values and for detecting "unchanged", and explicitly *not* for
comparing results of floating-point arithmetic (`include/geometry/vec3.hpp:46-48`). `nearlyEqual` is
the absolute-or-relative alternative (`include/geometry/scalar.hpp:67-74`), overloaded for `Vec3`,
`Vec4`, `Mat4` and `AABB` (`include/geometry/vec3.hpp:146`, `include/geometry/vec4.hpp:55`,
`include/geometry/mat4.hpp:142`, `include/geometry/aabb.hpp:256`).

**Overflow avoidance in `AABB::centroid`.** Written as `min + 0.5*(max-min)` rather than
`0.5*(min+max)` specifically to avoid overflowing to infinity when both corners are large and
same-signed (`include/geometry/aabb.hpp:81-82`).

**Division never has undefined behaviour.** `safeReciprocal` and `Vec3::operator/` together
guarantee that no division by a zero scalar is ever executed, in any build configuration (§5.2). The
true-divide-over-reciprocal-multiply choice in `operator/` also avoids a second rounding step and a
denormal-divisor overflow (`include/geometry/vec3.hpp:66-69`).

## 10. Performance considerations

> No measurements exist. `benchmarks/results/` contains only the format specification. Everything in
> this section is an *argument about algorithmic or cache behaviour*, not a result. One code comment
> states a measured result and is flagged in §5.1 and §13.1.

**Float over double: the memory-bandwidth argument.** `include/geometry/scalar.hpp:12-16`:

> a BVH is memory-bound, not ALU-bound. Every AABB is 6 scalars; halving their width halves the
> bytes pulled through cache during traversal, which is where the time actually goes.

The premise is verifiable from the code: `AABB` is exactly 2 × `Vec3` = 24 B at float, 48 at double
(`include/geometry/aabb.hpp:44-45`). BVH traversal is a pointer-chasing walk whose working set is
dominated by bounds, so halving node width roughly doubles nodes per cache line. The *conclusion* —
that this dominates ALU cost — is standard and is what production renderers do, but for this project
it is **UNVERIFIED**: no double-precision build has been measured, and the traversal loop the
argument is about does not exist yet.

**AABB over tighter bounding volumes.** `include/geometry/aabb.hpp:29-36`: "the bounding volume is
tested far more often than it is built, so the cost that matters is the per-test cost, not the
tightness." Three consequences the code demonstrates: the AABB/ray test needs no matrix and no
trigonometry (`:211-250`); AABBs are closed under union via component-wise min/max (`:58-68`), which
makes bottom-up bounds propagation cheap; and the admitted cost is a looser fit for diagonal
geometry, appearing as more false-positive node visits (`:35-36`). Quantifying the last needs
measurement: **UNVERIFIED**.

**Precomputing the reciprocal direction.** Computed once per ray and reused across every node
visited, turning three divides per AABB into three multiplies (`include/geometry/ray.hpp:40-41`).
Division latency exceeds multiply latency on x86, so the direction of the argument is not in doubt;
the magnitude is **UNVERIFIED**. The convenience overload at `include/geometry/aabb.hpp:252-254`
recomputes `invDirection` per call — fine for tests, but a traversal loop must use the five-argument
form at `:211` and hoist the reciprocal.

**The widening is now priceable rather than merely asserted.** Making it a build option
(`CMakeLists.txt:52-53`, `:91-95`) means Phase 9 can measure what it costs instead of arguing about
it, and the source quantifies the structural upper bound: "it is 3 of the 9 multiplies in this
function, the innermost loop of the whole system" (`include/geometry/aabb.hpp:202`). That is a
count, not a timing — the timing is **UNVERIFIED**. Defining it as a constant that is exactly 1 when
disabled means the multiply folds away rather than becoming a branch (`:235-236`).

**Indexed mesh storage and cache.** `include/geometry/mesh.hpp:19-27`. The saving is roughly 2× (§7).
The framing — "on a million-triangle mesh that difference decides whether the vertex data fits in
cache during a build" (`:22-23`) — is the right *shape* of argument, but whether a given mesh crosses
a specific machine's threshold is **UNVERIFIED**. The cost is honestly stated as one level of
indirection per vertex fetch, which matters during *traversal*, with the mitigation deferred: "Phase
2 can revisit by storing unpacked triangles in leaf order" (`:26-27`). That is the classic
build-vs-query layout tradeoff, already flagged in the source.

**Three places the code deliberately chose the slower option, and said so.** `Vec3::operator/` keeps
a true divide rather than multiplying by a reciprocal, because the reciprocal adds a rounding step
and mishandles denormal divisors, and this is not a hot path (`include/geometry/vec3.hpp:66-69`).
`invert` uses Gauss-Jordan rather than cofactor expansion for stability, because it runs once per
frame (`include/geometry/mat4.hpp:90-94`). And `invert`'s pivot search divides by a row scale
(`src/geometry/mat4.cpp:148`) rather than comparing raw magnitudes. Being able to point at cases
where the same "how often does this run?" criterion pointed the *other* way is a stronger position
than optimising uniformly.

**Header-only hot primitives.** Everything except `Mat4` and `Mesh` is `inline`/`constexpr` in
headers, so `dot`, `cross`, `minComponents`, `extend` and `intersectRay` are visible for inlining at
the call site. Only `src/geometry/mat4.cpp` and `src/geometry/mesh.cpp` compile separately
(`CMakeLists.txt:112-115`), and neither contains a per-ray operation.

**Avoided square roots.** `Triangle::isDegenerate` squares both sides of the aspect-ratio test
(`include/geometry/triangle.hpp:63-71`); `normalizeSafe` tests `lengthSquared` before deciding to
take the root (`include/geometry/vec3.hpp:118-120`); `normalUnnormalized` is returned unnormalised so
callers needing only an orientation sign can skip the root (`include/geometry/triangle.hpp:35-37`).

**Const-correctness and copies.** `Mesh` takes both buffers by value and moves
(`include/geometry/mesh.hpp:36-42`, `src/geometry/mesh.cpp:9`), with the comment noting these may be
tens of megabytes. `Mesh::triangle` returns by value with an explicit justification — 36 bytes is
cheaper to copy than to alias, and returning a value keeps the `Mesh` immutable to callers
(`include/geometry/mesh.hpp:67-68`).

**Warning set as a performance guard.** `-Wconversion`, `-Wsign-conversion` and `-Wdouble-promotion`
are all enabled (`CMakeLists.txt:67-71`), the last specifically to catch an accidental
`float`→`double` promotion in a hot loop — which would silently undo the float decision. The tree is
clean under `-Werror` with the full set (§15).

## 11. Alternative approaches

| Decision | Alternatives considered | Consequence of the alternative |
|---|---|---|
| `Scalar = float` (`include/geometry/scalar.hpp:20`) | `double`; templated precision | `double`: ~7 → ~16 digits, but 48-byte AABBs and half the nodes per cache line. Templates: precision noise at every call site. |
| AABB (`include/geometry/aabb.hpp:29-36`) | OBB, sphere, k-DOP | OBB: tighter, but each test needs a transform into local frame, and union is not component-wise. Sphere: cheapest test, very loose for elongated geometry. k-DOP: intermediate on both axes; more planes per test, more storage per node. |
| Surface area as the SAH cost term (`include/geometry/aabb.hpp:86-90`) | Volume weighting; primitive count alone | Volume has no geometric-probability justification for rays, and gives a flat box zero cost. Count alone ignores that a large box is hit more often. |
| Inverted-interval empty box (`include/geometry/aabb.hpp:38-42`) | `bool empty` flag; zero-initialised box | Flag: a branch in `extend`, the innermost operation of bounds propagation, plus a wider struct. Zero-init: silently wrong — the box contains the origin. |
| Swap on `sign(invDir)` (`include/geometry/aabb.hpp:220-228`) | Swap on `tNear > tFar` (the widely published form) | Repairs an inverted empty box into `[-inf, +inf]`, so every empty box hits every ray. Same cost either way. Appendix A. |
| Widening as a switchable constant (`include/geometry/aabb.hpp:11-25`) | Hard-coded expression; `#if` at the use site; no widening | Hard-coded: unmeasurable. `#if` at the use site: more than one thing changes when you flip it, and risks an ODR violation if it differs per TU. None: the PBRT crack risk, unproven but real from Phase 2 onward. |
| `safeReciprocal` + guarded `Vec3::operator/` (`include/geometry/scalar.hpp:47-62`, `include/geometry/vec3.hpp:60-76`) | Plain `1/d`; clamping away from zero; suppressing the sanitizer | Plain: UB per `[expr.mul]/4`. Clamping: a fictitious direction and a wrong `t`. Suppressing: hides the report and loses the check elsewhere. |
| True divide in `operator/` (`include/geometry/vec3.hpp:66-69`) | Multiply by reciprocal | Faster, but a second rounding step and overflow for a denormal divisor. |
| Gauss-Jordan (`include/geometry/mat4.hpp:88-94`) | Cofactor/adjugate; LU; affine special case | Cofactor: faster for a fixed 4×4, less stable. Affine special case: much faster but needs a validity check and a second codepath. |
| **Scaled** partial pivoting (`src/geometry/mat4.cpp:127-142`) | No pivoting; plain partial pivoting; absolute cutoff; matrix-norm-relative cutoff | No pivoting: a near-zero pivot amplifies error. Plain: a row scaled by `1e6` wins every contest without being better conditioned. Absolute: rejects a uniformly tiny invertible matrix. **Matrix-norm-relative: rejects `scaling(1e-21)`, because a homogeneous transform's `m[3][3] == 1` dominates the norm.** |
| Dimensionless degeneracy test (`include/geometry/triangle.hpp:52-63`) | Absolute area threshold | Scale-dependent; flags healthy small triangles and mis-fires on unit-scale million-triangle meshes. |
| Column-major (`include/geometry/mat4.hpp:11-17`) | Row-major | Reads more naturally on paper but requires a transpose on every GLSL/SPIR-V upload. |
| Non-normalised ray direction (`include/geometry/ray.hpp:11-19`) | Always unit-length | A `sqrt` per ray, and renormalising after a scaling transform silently changes the meaning of `t`. |
| Indexed mesh (`include/geometry/mesh.hpp:19-27`) | `vector<Triangle>` | ~2× the memory; no indirection during traversal; would need repacking before `vkCmdDrawIndexed`. |
| Tight `bounds()` + separate `vertexBounds()` (`include/geometry/mesh.hpp:77-90`) | One conservative bound | An inflated root shifts every SAH split decision and makes benchmarks depend on file hygiene. |
| Throw at the boundary, assert elsewhere (`CLAUDE.md`; `include/geometry/mesh.hpp:39-41`) | Assert everywhere; error codes; silent clamping | Assertions vanish in release, exactly when malformed file data arrives. |
| Separate point/vector/normal transforms (`include/geometry/mat4.hpp:108-140`) | One `transform(Mat4, Vec3)` | A silent convention rather than a checked one; the inverse-transpose bug becomes invisible. |

## 12. Why the chosen approach was selected

The selections share one criterion, stated in `CLAUDE.md`: *simple and defensible beats clever*.
Four threads run through them.

**Optimise the operation that runs most often, not the one that looks most expensive.** AABBs over
OBBs, float over double, precomputed reciprocals, and the branch-free `extend` all follow from
asking "how many times per frame does this execute?" The AABB header states the principle
(`include/geometry/aabb.hpp:29-31`), and `invert` and `Vec3::operator/` are the counter-examples that
prove the rule: both are allowed to be slower because they are not hot
(`include/geometry/mat4.hpp:91-93`, `include/geometry/vec3.hpp:69`).

**Prefer being conservatively wrong to being occasionally wrong.** For an acceleration structure a
false positive costs one wasted narrow-phase test; a false negative is a visible hole. Every
numerical decision in `intersectRay` picks that side: the widening
(`include/geometry/aabb.hpp:188-192`), the NaN no-op (`:180-186`), the strict `<` that keeps flat
boxes hittable (`:244-245`). `AABB::intersects` makes the same call for touching boxes (`:125-127`).

**Make thresholds scale-invariant — and check that the invariance is the *right* invariance.** Three
tolerances are relative rather than absolute (§9). The matrix one is the instructive case: the first
obvious fix, scaling by the whole matrix's largest entry, is *also* scale-invariant and is *still*
wrong for this project's matrices, because a homogeneous transform's `m[3][3] == 1` dominates the
norm (`src/geometry/mat4.cpp:134-138`). A per-row ratio is the invariance that actually matches the
problem, and a test exists precisely to keep the wrong fix out
(`tests/test_mat4.cpp:241-254`).

**Make the abstract machine, not just the hardware, correct.** The two bugs fixed in this milestone
are cases where code that "works" on real hardware is undefined by the standard: `(&x)[i]` and
`1/0`. Both were replaced with constructs that produce identical results with defined semantics, at
no meaningful cost (`include/geometry/vec3.hpp:17-22`, `include/geometry/scalar.hpp:50-57`), and the
second fix was later extended from `safeReciprocal` to `Vec3::operator/`
(`include/geometry/vec3.hpp:60-76`) so that even the release-build fallback path of `normalize` is
defined. UBSan is clean in both configurations (§15).

## 13. Known limitations

1. **An unrecorded measurement in a code comment.** `include/geometry/aabb.hpp:196-198` states that
   "a sweep of 3 million rays aimed at shared sibling faces found no difference with it removed."
   There is no file under `benchmarks/results/`, and the largest committed sweep is 20,000 rays
   (`tests/test_aabb_property.cpp:120`). Per `CLAUDE.md`'s absolute rule and
   `benchmarks/results/README.md:3-5`, this must either be recorded as a result or restated as
   `UNVERIFIED`. **Needs the Implementation Engineer's input.** Everything else in that comment
   block — including its own closing instruction not to restate PBRT's rationale as a measured
   result — is exemplary.
2. **No memory-safety sanitizer coverage on this machine.** `BVH_ENABLE_SANITIZERS` wires up both
   ASan and UBSan (`CMakeLists.txt:97-102`), and UBSan runs clean in Debug and Release. **ASan
   cannot run at all here**: a hello-world built with `-fsanitize=address` exits 139, verified in
   isolation, so the failure is environmental rather than a defect in this code. The practical
   consequence is that use-after-free, buffer-overflow and leak classes are currently unchecked. The
   geometry core's exposure is small — it heap-allocates only through `std::vector` inside `Mesh`
   (`include/geometry/mesh.hpp:106-107`) and has no raw owning pointers — but "no ASan coverage" is
   an honest gap, and it will matter more once Vulkan resource handles arrive in Phase 7.
3. **The `Mesh` header's memory arithmetic understates the win.** `include/geometry/mesh.hpp:20-23`
   says 12 B + 12 B per triangle; its own "~6 triangles per vertex" premise gives 6 B + 12 B.
   Conclusion unaffected; see §7.
4. **`Triangle::isDegenerate`'s default tolerance is very permissive as an aspect ratio.**
   `kEpsilon = 1e-6` now means `h/L ≤ 1e-6` (`include/geometry/triangle.hpp:64`), so a sliver with a
   height one part in a million of its longest edge is flagged, but anything thicker is not. That is
   the correct *form* of predicate; whether `1e-6` is the right *value* for BVH purposes is an open
   question a Phase 2 measurement would settle. Note the default is shared with `kEpsilon`'s other,
   unrelated role as a "these quantities are the same" tolerance.
5. **`Mesh::transform` applies `transformPoint`, which includes the perspective-divide check**
   (`src/geometry/mesh.cpp:46`, `src/geometry/mat4.cpp:201`), although the member is documented as
   taking an *affine* transform (`include/geometry/mesh.hpp:97`). Harmless — affine matrices leave
   `w == 1` and skip the divide — and the non-affine path is deliberately tested
   (`tests/test_mesh.cpp:151-168`), so the documented contract and the tested behaviour disagree
   slightly. Worth reconciling the comment.
6. **The two-argument `intersectRay` recomputes `invDirection` per call**
   (`include/geometry/aabb.hpp:252-254`). Fine for tests; traversal must use the five-argument form.
7. **`Vec3::operator/` and `operator/=` are no longer `constexpr`** (`include/geometry/vec3.hpp:70`,
   `:80`) as the price of the UB guard. Nothing in the core needs them to be, but a future
   `constexpr` bounds computation would hit this.
8. **No SIMD, no multithreading, no arena allocation.** All deferred; `CLAUDE.md` bans premature
   optimisation without measurement.
9. **`Mesh` has no OBJ loader** and **stores positions only** — no normals, no UVs. Adding attributes
   raises an AoS-vs-SoA question, since the BVH build wants positions alone and the renderer wants
   them interleaved. `UNKNOWN — not yet implemented`.
10. **The project cannot honestly claim C++20.** See [Appendix B](#appendix-b--is-this-really-c20).
11. **No benchmark of any kind exists.** Every performance statement in this document is an argument
    from layout or complexity.

## 14. Edge cases

| Input | Behaviour | Citation | Tested |
|---|---|---|---|
| Default-constructed `AABB` | empty; `surfaceArea == 0`, `volume == 0` | `include/geometry/aabb.hpp:44-45`, `:93-103` | `tests/test_aabb.cpp:9` |
| `extend` an empty box with a point | degenerate single-point box, no special case | `include/geometry/aabb.hpp:58-61` | `tests/test_aabb.cpp:20` |
| `merge` with an empty box | identity | `include/geometry/aabb.hpp:63-68` | `tests/test_aabb.cpp:48` |
| Flat (zero-thickness) `AABB` | not empty; real surface area, zero volume; **still hittable** | `include/geometry/aabb.hpp:53-55`, `:244-245` | `tests/test_aabb.cpp:65`, `:235` |
| Empty `AABB` vs any ray | never hit — what the swap-on-sign fix protects | `include/geometry/aabb.hpp:220-228` | `tests/test_aabb.cpp:244`; randomised `tests/test_aabb_property.cpp:159` |
| Empty `AABB` in `centroid()` / `offset()` | precondition violation — **asserts in Debug** (would be NaN) | `include/geometry/aabb.hpp:72-80`, `:138-139` | `tests/test_preconditions.cpp:50`, `:56` |
| Empty `AABB` in `contains(AABB)` / `intersects` | contained: true; intersects: false — same set semantics | `include/geometry/aabb.hpp:116-121`, `:128-129` | `tests/test_aabb.cpp:104`, `:113` |
| Axis-parallel ray, origin outside the slab | ±inf products compare correctly; miss | `include/geometry/aabb.hpp:180-184` | `tests/test_aabb.cpp:199` |
| Axis-parallel ray, origin **exactly on** a slab plane | `0 * inf == NaN`; ternaries make the axis a no-op; finite hit | `include/geometry/aabb.hpp:180-186`, `:241-242` | `tests/test_aabb.cpp:207` |
| Ray origin inside the box | hit with `tEnter == tMin` | `include/geometry/aabb.hpp:206-207` | `tests/test_aabb.cpp:173`; randomised `tests/test_aabb_property.cpp:171` |
| Ray pointing away from the box | miss | `include/geometry/aabb.hpp:245` | `tests/test_aabb.cpp:166` |
| Box outside `[tMin, tMax]` | miss | `include/geometry/aabb.hpp:213-214` | `tests/test_aabb.cpp:182`, `:192` |
| Negative direction components | swap on the sign of `invDir` | `include/geometry/aabb.hpp:229-233` | `tests/test_aabb.cpp:218` |
| `offset()` on a degenerate axis | returns 0 for that axis; no divide by zero | `include/geometry/aabb.hpp:141-143` | `tests/test_aabb.cpp:142` |
| Touching boxes | count as intersecting (deliberate) | `include/geometry/aabb.hpp:125-127` | `tests/test_aabb.cpp:116` |
| Disjoint boxes in `intersection()` | empty result, not garbage | `include/geometry/aabb.hpp:165-168` | `tests/test_aabb.cpp:131` |
| `Vec3 / 0` | IEEE result by construction: ±inf, NaN for `0/0`; **never UB** | `include/geometry/vec3.hpp:70-76` | `tests/test_preconditions.cpp:92` |
| `normalize` of a zero vector | precondition — asserts in Debug; NaN (not UB) in Release | `include/geometry/vec3.hpp:101-112` | `tests/test_preconditions.cpp:61`, `:78` |
| `normalizeSafe` of a short vector | returns `fallback` (default zero vector) | `include/geometry/vec3.hpp:114-121` | `tests/test_vec3.cpp:104`; `tests/test_preconditions.cpp:102` |
| Zero-area / collinear triangle | `normal()` returns zero, **not NaN**; `area() == 0`; bounds and centroid still valid | `include/geometry/triangle.hpp:39-72` | `tests/test_triangle.cpp:82`, `:91`, `:100` |
| Triangle with all three vertices coincident | short-circuits to degenerate before any division | `include/geometry/triangle.hpp:68-69` | `tests/test_triangle.cpp:100` |
| Healthy but *small* triangle (area `5e-7`) | **not** flagged — the predicate is a scale-invariant aspect ratio | `include/geometry/triangle.hpp:52-63` | `tests/test_triangle.cpp:110` |
| Needle-thin sliver at any scale | flagged at `1e-3`, `1`, `1e5` | `include/geometry/triangle.hpp:60-62` | `tests/test_triangle.cpp:125` |
| Same triangle uniformly rescaled | verdict unchanged across 8 orders of magnitude | `include/geometry/triangle.hpp:62` | `tests/test_triangle.cpp:135` |
| Collinear / coincident points in either `Plane` factory | degenerate plane (zero normal), not NaN | `include/geometry/plane.hpp:22-37`, `:44` | `tests/test_plane.cpp:30`, `:36` |
| Zero-length rotation axis | identity matrix, not NaN | `src/geometry/mat4.cpp:31-32` | `tests/test_mat4.cpp:86` |
| All-zero row in `invert` | rejected before any division | `src/geometry/mat4.cpp:121-122` | `tests/test_mat4.cpp:146` |
| Uniformly tiny but invertible matrix (`scaling(1e-21)`) | **accepted** — the pivot test is a per-row ratio | `src/geometry/mat4.cpp:127-142` | `tests/test_mat4.cpp:241` |
| Rank-deficient matrix at extreme scale (`1e-21`, `1e21`) | rejected at both | `src/geometry/mat4.cpp:156` | `tests/test_mat4.cpp:250-253` |
| Projection matrix in `transformPoint`, `w ∉ {0,1}` | perspective divide applied | `src/geometry/mat4.cpp:201-204` | `tests/test_mat4.cpp:183`; via `Mesh` `tests/test_mesh.cpp:151` |
| Point projecting to `w == 0` | divide skipped; raw finite xyz returned | `include/geometry/mat4.hpp:114-118` | — (documented, untested) |
| Normal through a uniform 4× scale | direction preserved, **length 1/4** | `include/geometry/mat4.hpp:136-139` | `tests/test_mat4.cpp:256` |
| Index count not a multiple of 3 | `std::invalid_argument` | `src/geometry/mesh.cpp:10-13` | `tests/test_mesh.cpp:84` |
| Out-of-range index | `std::invalid_argument` naming index and slot; index widened, not count narrowed | `src/geometry/mesh.cpp:16-24` | `tests/test_mesh.cpp:88`, `:93` |
| Out-of-range *triangle* index | precondition — asserts in Debug | `include/geometry/mesh.hpp:60`, `:70` | `tests/test_preconditions.cpp:32`, `:38`, `:44` |
| Empty mesh | accepted; `empty()` true; `triangle(0)` asserts | `include/geometry/mesh.hpp:46` | `tests/test_mesh.cpp:36`, `:97`; `tests/test_preconditions.cpp:38` |
| Orphan (unreferenced) vertices | `bounds()` stays tight; `vertexBounds()` covers them | `include/geometry/mesh.hpp:77-90` | `tests/test_mesh.cpp:101` |
| Degenerate triangles in a mesh | counted and reported, never silently dropped | `include/geometry/mesh.hpp:92-95` | `tests/test_mesh.cpp:119` |
| `maxAxis` on a cube / tie | lowest index wins, deterministically | `include/geometry/vec3.hpp:138-144` | `tests/test_aabb.cpp:84` |

## 15. Testing strategy

**Reported status at `935c354`** (measured by the Implementation Engineer, not by me — I did not
build; there is no `build/` directory in the tree and a first configure needs network access to
fetch GoogleTest, `tests/CMakeLists.txt:8-21`):

| Configuration | Result |
|---|---|
| Release (`ctest`) | **122 / 122 pass** |
| Debug (`ctest`) | **128 / 128 pass** |
| `-Werror`, full strict warning set (`CMakeLists.txt:63-81`) | clean, zero warnings |
| UBSan, **both** configurations | clean |
| ASan | **cannot run on this machine** — a hello-world built with `-fsanitize=address` exits 139; environmental, verified in isolation. See §13.2. |

**Why the two counts differ.** 129 `TEST()` macros exist in the source, but
`tests/test_preconditions.cpp` is conditionally compiled: 7 death tests only exist when `NDEBUG` is
undefined (`:30-70`), a single explanatory skip-stub only exists when it is defined (`:23-28`), and
3 release-safety tests compile in both (`:78-105`). So Release registers 118 + 1 + 3 = 122 and Debug
registers 118 + 7 + 3 = 128. One test skips in each configuration: the stub in Release, and
`NormalizeZeroVectorIsNotUndefinedBehaviour` in Debug, where the assert fires first (`:87-89`).

**Framework and wiring.** GoogleTest, preferring an installed copy and otherwise fetching a pinned
v1.15.2 (`tests/CMakeLists.txt:6-22`). The version is pinned rather than tracking a branch so a test
failure is always attributable to our change, never to a silently updated dependency (`:3-5`).
`gtest_discover_tests` (`:51`) registers each `TEST()` individually so a failure names the case and
`ctest` can parallelise.

**Both build configurations are required**, because `-DNDEBUG` strips every `assert` and a
Release-only run never exercises a precondition (`CLAUDE.md`, Build and test; `README.md`,
Building). The configuration summary prints whether asserts are live
(`CMakeLists.txt:150-154`).

**Distribution.** 129 `TEST()` cases across nine files:

| File | Cases | Focus |
|---|---:|---|
| `tests/test_aabb.cpp` | 28 | construction, sentinel, measures, containment, offset, 12 ray-intersection cases |
| `tests/test_mat4.cpp` | 22 | storage layout, `fromColumns`, transforms, inverse (incl. scale-invariance), normal transform, perspective divide |
| `tests/test_vec3.cpp` | 17 | arithmetic, dot/cross handedness, normalize, min/max, `maxAxis` |
| `tests/test_mesh.cpp` | 16 | counts, accessors, validation rejections, tight vs conservative bounds, transform |
| `tests/test_triangle.cpp` | 15 | edges, normal winding, area, centroid, bounds tightness, scale-invariant degeneracy |
| `tests/test_preconditions.cpp` | 11 | 7 death tests + 1 Release skip-stub + 3 release-safety tests |
| `tests/test_ray.cpp` | 10 | construction, `invDirection`, transform semantics |
| `tests/test_aabb_property.cpp` | 5 | randomised invariants over the slab test |
| `tests/test_plane.cpp` | 5 | factories, signed distance, degeneracy |

**Six distinguishable layers.**

1. *Known-answer tests* for closed-form results: a 1×2×3 box has surface area 22 and volume 6
   (`tests/test_aabb.cpp:58-63`); a ray from `z = -5` hits the unit box at `t = 4` (`:152-158`).
2. *Property tests* asserting mathematical relationships: cross product orthogonal to both inputs
   (`tests/test_vec3.cpp:84`); rotation preserves length (`tests/test_mat4.cpp:80`); rotation leaves
   its axis fixed (`:92`); `M·M⁻¹ = I` (`:129`); world→object→world round-trip recovers the ray
   (`tests/test_ray.cpp:93-106`); matrix-vector product matches manual expansion
   (`tests/test_mat4.cpp:200-209`).
3. *Randomised property tests* — the strongest part of the suite. `tests/test_aabb_property.cpp`
   seeds `std::mt19937` with a fixed constant so a failure is reproducible ("A randomised test that
   cannot be replayed is a flaky test", `:9-10`), generates directions that are *sometimes exactly
   axis-aligned* to keep the infinite-reciprocal paths in the sample (`:64-73`), and checks five
   invariants: any sampled point inside the box implies a reported hit (`:81-111`), the widening
   never removes a hit (`:113-143`), `tEnter` stays within the ray range and finite (`:145-157`), an
   empty box is never hit from any direction (`:159-169`), and a ray originating inside always hits
   with `tEnter == tMin` (`:171-186`). Three details make this a good suite rather than a token one:
   the soundness test is deliberately **one-directional** — sampling can prove a hit but never a
   miss, so there are no false alarms (`:82-85`); it guards against silently proving nothing by
   requiring a minimum number of proven hits (`:109-110`); and the widening comparison **logs rather
   than asserts** the disagreement count, because "if a future change makes the widening start
   mattering, that is information rather than a failure" (`:137-142`). It also keeps a deliberately
   separate unwidened reference implementation so the two can be compared (`:24-44`), and names its
   future role: the oracle for Phase 2, since BVH traversal is correct only if it reports the same
   hits as testing every primitive directly (`:6-7`).
4. *Death tests for preconditions.* `tests/test_preconditions.cpp` covers the asserting half of the
   error policy: `Mesh::triangle` / `triangleIndices` out of range (`:32-48`), `AABB::centroid` and
   `offset` on an empty box (`:50-59`), `normalize` of a zero vector (`:61-63`),
   `Vec3::operator[]` out of range (`:65-68`). The file is careful about its own validity: under
   `NDEBUG` it compiles to a single `GTEST_SKIP` with an explanatory message (`:23-28`), because
   "if this file were not guarded, the tests would 'pass' in Release by doing nothing, which is
   worse than not having them" (`:8-11`). Its last three cases run the other way — release-safe
   behaviour that must hold in *every* build, including that `normalize(Vec3(0))` yields NaN rather
   than UB (`:78-90`) and that `Vec3 / 0` matches IEEE component-wise (`:92-100`).
5. *Degeneracy and hazard tests* named after the failure they prevent:
   `RayLyingExactlyOnSlabBoundaryIsHandled` (`tests/test_aabb.cpp:207`), `EmptyBoxIsNeverHit`
   (`:244`), `FlatBoxIsStillHittable` (`:235`), `OffsetOnDegenerateAxisDoesNotDivideByZero` (`:142`),
   `CollinearPointsGiveDegeneratePlaneNotNaN` (`tests/test_plane.cpp:30`),
   `RotationAboutDegenerateAxisIsIdentity` (`tests/test_mat4.cpp:86`).
6. *Convention-pinning tests*: `ColumnMajorStorageLayout` (`tests/test_mat4.cpp:28`),
   `FromColumnsMatchesStorageOrder` (`:211`), `MultiplicationAppliesRightmostFirst` (`:99`),
   `CrossProductIsRightHanded` (`tests/test_vec3.cpp:67`), `NormalFollowsCounterClockwiseWinding`
   (`tests/test_triangle.cpp:20`), `DirectionIsNotRequiredToBeNormalized` (`tests/test_ray.cpp:35`).

**Four tests are worth reading in full as documentation.** `tests/test_mat4.cpp:158-182` asserts both
that the inverse-transpose result stays perpendicular to the transformed tangent *and* that the
naive `transformVector` answer does not — so the test fails if someone "simplifies" the function.
`tests/test_ray.cpp:79-91` asserts `transformPoint(m, r.at(t)) == transformRay(m, r).at(t)`, exactly
the property that would break if `transformRay` renormalised the direction.
`tests/test_mesh.cpp:101-116` asserts the tight bound excludes an orphan vertex, the conservative
bound includes it, and the second contains the first — three assertions pinning the whole two-bounds
design. And `tests/test_mat4.cpp:241-254` exists specifically to reject a plausible-but-wrong fix to
the singularity threshold (§5.3).

**Comparison discipline in tests.** Exact `EXPECT_EQ` on `Vec3` only where values are exactly
representable — sentinels, integral coordinates, unchanged components (`tests/test_aabb.cpp:13-14`,
`:36-37`) — and `nearlyEqual`/`EXPECT_NEAR` wherever arithmetic has occurred (`:76`,
`tests/test_ray.cpp:104-105`). This mirrors the policy documented at
`include/geometry/vec3.hpp:46-48`.

**Gaps.** No ASan coverage (§13.2). No test for `transformPoint` on a point projecting to exactly
`w == 0`. No test for the ODR hazard the `BVH_CONSERVATIVE_RAY_BOX` macro placement guards against —
that would need a two-TU build with mismatched definitions, which is arguably a build-system test
rather than a unit test. No fuzzing beyond the seeded property suite. And **no benchmarks at all**.

## 16. How it interacts with other components

**Downward: nothing.** The geometry core depends only on the C++ standard library — `<cmath>`,
`<limits>`, `<cassert>`, `<cstddef>`, `<cstdint>`, `<vector>`, `<stdexcept>`, `<string>`,
`<utility>`. No third-party headers, no Vulkan, no GLFW, no ImGui. This is the project's "single
most important structural rule" (`CLAUDE.md`, Layering), restated in the build at
`CMakeLists.txt:107-109`.

**Upward: everything.** Consumers, present and planned:

| Consumer | Status | What it will use |
|---|---|---|
| `tests/` | **exists** | the whole surface; links `bvh::geometry` (`tests/CMakeLists.txt:36-40`) |
| BVH construction (Phase 2) | `UNKNOWN — not yet implemented` | `AABB::extend`/`merge` for bottom-up bounds; `Triangle::centroid` for partitioning; `AABB::longestAxis` for the default split axis; `AABB::surfaceArea` for SAH cost; `AABB::offset` for SAH bin indices; `Mesh::bounds()` (tight) for the root |
| BVH traversal (Phase 2) | `UNKNOWN — not yet implemented` | the five-argument `intersectRay` with a hoisted `invDirection`; `Ray::tMax` shrinking as closer hits are found |
| Ray/triangle intersection | `UNKNOWN — not yet implemented` | **deliberately not in Phase 1**; no Möller–Trumbore or equivalent exists |
| Collision / clearance (Phase 6) | `UNKNOWN — not yet implemented` | `AABB::intersects`, `intersection`, `Plane::signedDistance` |
| Vulkan rendering (Phase 7) | `UNKNOWN — not yet implemented` | `Mat4` uploaded untransposed; `Mesh` positions and indices for `vkCmdDrawIndexed`; `vertexBounds()` for sizing a vertex buffer; `transformNormalWithInverse` for shading normals (renormalised — see §5.4) |
| GPU compute (Phase 8) | `UNKNOWN — not yet implemented` | — |
| Benchmarks (Phase 9) | `UNKNOWN — not yet implemented` | `Mesh::countDegenerateTriangles` so mesh quality is visible rather than quietly changing the primitive count (`include/geometry/mesh.hpp:92-94`); `BVH_CONSERVATIVE_RAY_BOX=0` to price the widening (`CMakeLists.txt:52-53`) |

**Seams already cut for Phase 2**, each documented at the point of the decision:

- `AABB::offset` — SAH bin index (`include/geometry/aabb.hpp:135-137`).
- `AABB::centroid`'s precondition — named after the Phase 2 failure it prevents, a NaN centroid
  corrupting a bin index (`:74-78`).
- `Triangle::centroid` — "a centroid gives each primitive exactly one position to sort by, whereas
  overlapping bounds do not induce an ordering" (`include/geometry/triangle.hpp:84-86`).
- `Ray::tMax` carried in the ray — "once a hit at t is known, every subtree whose entry distance
  exceeds t can be skipped outright" (`include/geometry/ray.hpp:21-24`).
- `AABB::surfaceArea` — the SAH cost term with the ray-hit-probability justification inline
  (`include/geometry/aabb.hpp:86-90`).
- `Mesh::bounds()` tight by default — so the SAH's parent-area normalisation is not skewed by orphan
  vertices (`include/geometry/mesh.hpp:80-85`).
- `Mesh::triangle`'s index assert — "Phase 2 will call this from a loop bounded by a node's
  primitive range, which is exactly where an off-by-one would land"
  (`include/geometry/mesh.hpp:53-57`).
- `tests/test_aabb_property.cpp` — explicitly designed to become the Phase 2 correctness oracle
  (`:6-7`).

**Seams cut for the renderer:** column-major `Mat4` so uniform upload needs no transpose
(`include/geometry/mat4.hpp:13-15`), indexed `Mesh` so buffers upload without repacking
(`include/geometry/mesh.hpp:24-25`), and `vertexBounds()` for vertex-buffer sizing (`:88-90`). None
can be exercised until the Vulkan SDK is installable on this machine (`CLAUDE.md`, Toolchain status).

---

## Appendix A — two bugs fixed in this milestone

Both were found while writing the tests, both are documented in the `841777e` commit message and in
the source comments, and — because Phase 1 landed as a single commit — **neither appears as a diff in
git history.** The buggy versions never reached a commit. The evidence is the commit message, the
comments that explain what was avoided, and the tests written to pin the fixes.

### A.1 The slab-test swap: `tNear > tFar` vs `sign(invDir)`

The widely published form swaps when the near value exceeds the far value:

```cpp
if (tNear > tFar) std::swap(tNear, tFar);   // the bug
```

The code swaps on the sign of the reciprocal direction instead
(`include/geometry/aabb.hpp:229-233`, reasoning at `:220-228`).

**Why the comparison form is wrong here.** Take a default-constructed `AABB` — `min = +inf`,
`max = -inf` (`:44-45`) — and a ray with positive `x` direction:

```
tNear = (+inf - origin.x) * invDir.x = +inf
tFar  = (-inf - origin.x) * invDir.x = -inf
```

`tNear > tFar` is true, so the comparison form swaps, producing `[-inf, +inf]`, which constrains
nothing. All three axes do the same, `t1 < t0` never fires, and **the function returns true: an
empty box hits every ray.** The `if` that was supposed to repair a negative direction has instead
repaired the sentinel that encodes "this box is not a region at all".

The sign form leaves the inverted interval inverted: `t0` becomes `+inf`, `t1` becomes `-inf`, and
`t1 < t0` fires on the first axis (`:225-228`). The same holds for a negative direction (the sign
test swaps `-inf`/`+inf` back to `+inf`/`-inf`) and for an axis-parallel direction where `invDir` is
`+inf`. The sign is the *cause* of the crossing order; `tNear > tFar` is only a *symptom*, and a
symptom can be produced by a degenerate input.

**Why it matters in a BVH.** Empty bounds are routine: a node with no primitives after a partition,
a partially built tree, an unfilled child slot, a conservatively initialised SAH bin. If every empty
node reports a hit, traversal descends into it and the acceleration structure quietly stops
accelerating — no crash, no wrong pixel, just a performance cliff that is very hard to attribute.
`tests/test_aabb.cpp:244-248` is the regression test; `tests/test_aabb_property.cpp:159-169` now
sweeps 5,000 random directions including axis-aligned and negative ones, with a comment naming this
bug as its reason (`:160-162`).

**Why it is a good interview story.** The two forms look equivalent, the buggy one is the one most
people have seen, the failure is silent and performance-only, and the fix costs exactly nothing. The
comment's own summary is right: "Same cost, one fewer way to be wrong" (`:228`).

### A.2 `1/0` is undefined behaviour even when IEEE-754 defines it

The slab test wants `invDir` to be `±inf` for an axis-parallel ray — load-bearing, not accidental
(`include/geometry/ray.hpp:43-47`). The obvious implementation is `1.0f / d`, which on any IEEE-754
machine yields exactly the desired infinity.

It is still undefined behaviour. `[expr.mul]/4` makes division by zero UB *regardless of operand
type*: the IEEE guarantee is a property of the hardware, not of the C++ abstract machine
(`include/geometry/scalar.hpp:50-53`). UBSan's `float-divide-by-zero` check flags it, and per the
commit message that is how it was found. The response was not to suppress the check but to produce
the same value by construction, with `copysign` so that `-0.0` gives `-inf`
(`include/geometry/scalar.hpp:58-62`).

**The fix was later extended.** `Vec3::operator/` had the same latent problem — reachable through
`normalize` in a release build, where the assert is gone — and now branches on a zero divisor and
reproduces IEEE semantics by multiplying by the signed infinity
(`include/geometry/vec3.hpp:60-76`). `tests/test_preconditions.cpp:78-100` covers both halves: that
`normalize(Vec3(0))` produces NaN rather than UB in a Release build, and that `Vec3 / 0` matches
IEEE component-wise including `0/0 → NaN`.

**The interview point** is the distinction: "the hardware defines this" and "the abstract machine
defines this" are different claims, and only the second survives an optimiser. A compiler entitled
to assume the divisor is non-zero may propagate that backwards and delete a later zero-check. The
same distinction is behind the `(&x)[i]` decision (`include/geometry/vec3.hpp:17-19`) and the ban on
`-ffast-math` (`CMakeLists.txt:83-86`) — three instances of one principle, which makes it a
defensible theme rather than a memorised trivium.

## Appendix B — is this really C++20?

No, not fully, and the build says so out loud.

`CMakeLists.txt:12-14` requests C++20 with `CMAKE_CXX_STANDARD_REQUIRED ON` and extensions off. But
`CMAKE_CXX_STANDARD_REQUIRED` does not guarantee a real C++20: for a compiler that only knows the
pre-release draft flag, CMake silently emits `-std=c++2a` and still reports "20"
(`CMakeLists.txt:27-31`). The installed toolchain is Apple clang 11.0.3 (2020), which has no C++20
standard library — `<concepts>`, `<span>`, `<ranges>`, `<numbers>` are all missing (`CLAUDE.md`,
Toolchain status).

Rather than let the build look conformant, `CMakeLists.txt:32-40` compiles a probe that includes
`<concepts>` and `<span>` and uses a `std::floating_point` constrained template. On failure it emits
a `message(WARNING)` naming the compiler and pointing at `CLAUDE.md` (`:42-48`), and the
configuration summary prints `C++ standard ....... 20 (DRAFT -- no C++20 library)` (`:145`).
`README.md` carries a "Toolchain caveat" section saying the same to anyone who never runs CMake.

The geometry core is unaffected in practice: nothing in `include/geometry/` or `src/geometry/` uses
a C++20 library facility. The features it relies on — `constexpr` member functions, default member
initialisers, `inline constexpr` variables at namespace scope, hidden friend operators — are C++17
or earlier. **The honest statement is: this is C++17-compatible code compiled under a draft C++20
flag, and the project cannot claim C++20 until the Command Line Tools are updated.** Homebrew
already refuses to compile on this configuration, which will block GLFW, the Vulkan SDK and ImGui.

That is a good answer to give an interviewer who asks, because the interesting part is not the
version number — it is that the build *detects and reports* the discrepancy instead of inheriting it
silently.

## Appendix C — what Phase 1 deliberately does not contain

- **Ray/triangle intersection** — no Möller–Trumbore, no watertight variant, nothing. Deliberately
  out of scope. `UNKNOWN — not yet implemented`.
- **BVH node type, construction, partitioning, traversal** — Phase 2.
  `UNKNOWN — not yet implemented`.
- **SAH** — only the cost *term* exists (`AABB::surfaceArea`, `include/geometry/aabb.hpp:93`) and the
  binning *helper* (`AABB::offset`, `:138`). The heuristic itself, bin sweeping, and cost evaluation
  are Phase 3. `UNKNOWN — not yet implemented`.
- **Configurable leaf size, max depth, split strategy** — Phase 3.
  `UNKNOWN — not yet implemented`.
- **Vulkan instance/device/pipelines/descriptor sets/compute** — Phases 7–8, currently blocked on the
  toolchain. `UNKNOWN — not yet implemented`.
- **OBJ loading** — no parser in the tree. `UNKNOWN — not yet implemented`.
- **Benchmarks** — `benchmarks/results/` contains only `README.md`, a format and methodology
  specification. No measurement of any kind exists, so every performance statement in this document
  is an argument and is labelled as one.

## Appendix D — changelog

`935c354` "Phase 1 review fixes: UB, preconditions, scale-invariance" reworked several decisions
from `841777e`. The body above describes only the current state; this table records what moved, for
anyone reading an older review thread.

| Change | Files |
|---|---|
| `AABB::centroid` and `offset` gain non-empty preconditions with asserts (previously returned NaN silently) | `include/geometry/aabb.hpp:72-84`, `:138-139` |
| The `gamma(3)` widening became `kRayBoxWidening`, gated by `BVH_CONSERVATIVE_RAY_BOX`, with an ODR-safe macro placement | `include/geometry/aabb.hpp:11-25`, `:237`; `CMakeLists.txt:52-53`, `:88-95` |
| "HONEST STATUS" note added on the widening — mostly exemplary, but introduces the unrecorded 3-million-ray claim (§13.1) | `include/geometry/aabb.hpp:194-204` |
| `Triangle::isDegenerate` rewritten from an absolute-area test to a dimensionless aspect ratio; `tol` changed meaning | `include/geometry/triangle.hpp:52-72` |
| `invert`'s singularity test went from an absolute `1e-20` cutoff to **scaled partial pivoting** (per-row ratios), after a matrix-norm-relative intermediate was rejected for breaking homogeneous transforms | `src/geometry/mat4.cpp:112-156`; `include/geometry/mat4.hpp:96-101` |
| `Vec3::operator/` guards division by zero; `operator/=` moved out of line; both lose `constexpr` | `include/geometry/vec3.hpp:60-83` |
| `Plane::fromPointNormal` switched from `normalize` to `normalizeSafe`, matching `fromPoints` | `include/geometry/plane.hpp:22-29` |
| `Mesh::bounds()` became tight; `computeTriangleBounds()` replaced by a cached `vertexBounds()`; `Mesh` grew 24 bytes | `include/geometry/mesh.hpp:77-90`, `:106-109`; `src/geometry/mesh.cpp:28-34` |
| `Mesh::triangle` / `triangleIndices` assert their caller-supplied index | `include/geometry/mesh.hpp:53-60`, `:70` |
| Index validation widens the index instead of narrowing the count | `src/geometry/mesh.cpp:17-19` |
| `transformPoint`'s `w == 0` case documented; `transformNormalWithInverse` documented as not length-preserving | `include/geometry/mat4.hpp:114-118`, `:136-139` |
| New `tests/test_aabb_property.cpp` (5 randomised property tests) and `tests/test_preconditions.cpp` (11 cases) | whole files |
| New coverage for `fromColumns`, scale-invariant inversion, normal length, scale-invariant degeneracy, non-affine mesh transform | `tests/test_mat4.cpp:211-266`; `tests/test_triangle.cpp:110-145`; `tests/test_mesh.cpp:151-168` |
| Debug **and** Release runs both required before a milestone is done | `CLAUDE.md`; `README.md`; `CMakeLists.txt:150-154` |

Test count went from 105 to 122 (Release) / 128 (Debug).
