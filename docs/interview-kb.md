# Interview Knowledge Base

Questions this project can currently answer *from its own source*, with the answer you would give,
the detail behind it, and the exact code that proves it.

**Scope.** Started at Phase 1 (`841777e`, "Phase 1: geometry core with CMake and GoogleTest"). Only
the geometry core exists. Questions about BVH construction, traversal, SAH implementation, Vulkan,
GPU compute, and measured performance are listed at the end as
[not yet answerable](#not-yet-answerable), so you do not walk into an interview thinking you can
defend them.

**Rule this file follows.** There are no benchmarks — `benchmarks/results/` contains only a format
specification. No answer below quotes a number as measured. Where a code comment makes a
performance argument, the answer presents it as an argument and says what would have to be measured
to close it. Saying "I argued it from the memory layout but haven't measured it" is a strong answer;
inventing a speedup is a fatal one.

Companion document: [`docs/geometry.md`](geometry.md).

---

## Index

**Numerical and C++ correctness**
1. [Why `float` and not `double`?](#q1--why-float-and-not-double)
2. [Why does `Vec3::operator[]` use a conditional chain instead of `(&x)[i]`?](#q2--why-does-vec3operator-use-a-conditional-chain-instead-of-xi)
3. [You divide by zero deliberately. Isn't that undefined behaviour?](#q3--you-divide-by-zero-deliberately-isnt-that-undefined-behaviour)
4. [How do floating-point errors affect intersection tests?](#q4--how-do-floating-point-errors-affect-intersection-tests)
5. [What happens with degenerate geometry?](#q5--what-happens-with-degenerate-geometry)

**Bounding volumes**

6. [Why AABBs, and not OBBs, spheres, or k-DOPs?](#q6--why-aabbs-and-not-obbs-spheres-or-k-dops)
7. [Why is surface area the cost term in the SAH?](#q7--why-is-surface-area-the-cost-term-in-the-sah)
8. [Why is an empty AABB represented as `min = +inf, max = -inf`?](#q8--why-is-an-empty-aabb-represented-as-min--inf-max---inf)

**Ray/box intersection**

9. [Walk me through the slab method.](#q9--walk-me-through-the-slab-method)
10. [Tell me about a bug you found and fixed.](#q10--tell-me-about-a-bug-you-found-and-fixed)

**Linear algebra and transforms**

11. [Why is `Mat4` column-major?](#q11--why-is-mat4-column-major)
12. [Why Gauss-Jordan with partial pivoting instead of cofactor expansion?](#q12--why-gauss-jordan-with-partial-pivoting-instead-of-cofactor-expansion)
13. [Why do normals need the inverse transpose when directions don't?](#q13--why-do-normals-need-the-inverse-transpose-when-directions-dont)
14. [Why aren't ray directions normalized?](#q14--why-arent-ray-directions-normalized)

**Data layout**

15. [Why is the mesh indexed rather than an array of triangles?](#q15--why-is-the-mesh-indexed-rather-than-an-array-of-triangles)

**Process and engineering**

16. [Is this really C++20?](#q16--is-this-really-c20)
17. [How do you know any of this is correct?](#q17--how-do-you-know-any-of-this-is-correct)

[Not yet answerable](#not-yet-answerable)

---

## Q1 — Why `float` and not `double`?

### Short Answer

A BVH is memory-bound, not ALU-bound. Halving the width of every AABB doubles the number of nodes
per cache line during traversal, which is where the time goes. I pay for the lost precision by
writing the intersection code to be robust — conservative interval widening and NaN-tolerant
comparisons — rather than assuming exact arithmetic.

### Deep Answer

The tradeoff is roughly 16 decimal digits for 7, against half the bytes. Which side wins depends
entirely on whether the workload is limited by arithmetic throughput or by memory traffic, and BVH
traversal is firmly the second: it is a pointer-chasing walk over a node array where almost every
byte fetched is bounds data, and the arithmetic per node is a handful of multiplies and compares
that a modern core will issue faster than the cache can feed it.

An `AABB` is exactly 2 × `Vec3` = 24 bytes at float, 48 at double. On a 64-byte cache line that is
two-and-a-bit nodes versus one-and-a-bit. The choice is really about node density, and node density
is the lever on traversal time.

The honest part of the answer is the second half: 7 digits is not much, and the code has to earn
it back. That is why the slab test widens the exit distance by a relative error bound instead of
trusting `t` to be exact, and why every comparison in it is written to be NaN-safe. Those two
mechanisms are the price of the float decision, and I can point at both.

I kept the door open cheaply: `Scalar` is a single typedef, so a double-precision build is a
one-line change, and nothing is templated on precision — which would have put template noise at
every call site to buy a configurability nobody has asked for yet.

### Code Connection

- `using Scalar = float;` — `include/geometry/scalar.hpp:20`, with the memory-bound rationale at
  `:12-16` and the "typedef not template" rationale at `:8-10`.
- The 24-byte `AABB`: `include/geometry/aabb.hpp:28-29` (two `Vec3`, each three floats at
  `include/geometry/vec3.hpp:24-26`).
- The robustness that pays for it: `gamma(3)` widening at `include/geometry/aabb.hpp:196`,
  NaN-tolerant narrowing at `:200-201`.
- `-Wdouble-promotion` is enabled specifically so an accidental `float`→`double` promotion in a hot
  loop cannot silently undo the decision — `CMakeLists.txt:69`.

### Tradeoffs

| Choice | Gain | Cost |
|---|---|---|
| `float` (chosen) | 24-byte AABB, double node density per cache line | ~7 digits; intersection code must be written defensively |
| `double` | ~16 digits; naive intersection code is usually fine | 48-byte AABB; half the nodes per cache line |
| Templated on precision | Both available | Template noise at every call site; two codepaths to test |
| Mixed (float storage, double arithmetic) | Compact storage, accurate math | Conversion per access; `-Wdouble-promotion` exists to catch exactly this happening by accident |

**Be explicit about what is unmeasured.** The claim that memory traffic dominates ALU work *for
this code on this machine* is `UNVERIFIED`: no float-vs-double build has been benchmarked, and the
traversal loop the argument is about does not exist yet. The reasoning is standard and matches what
production renderers do, but I would want a measurement before putting a number on it.

### Follow-up Questions

- *"How would you verify the memory-bound claim?"* Build the same scene at both precisions, hold
  the ray set and seed fixed, and compare traversal time and L1/L2 miss rate — a hardware counter
  profile is what actually distinguishes "memory-bound" from "I assumed it was".
- *"Where would float precision actually bite you?"* Large scenes far from the origin: absolute
  spacing between representable floats grows with magnitude, so a scene at coordinate 10⁵ has
  roughly 10⁻² resolution. That is where you would move to doubles, or re-origin the scene.
- *"Would you use half precision for node bounds?"* Only with directed rounding — round min down and
  max up so bounds stay conservative. Without that, quantisation produces false misses.
- *"What's the AVX angle?"* Float gives 8 lanes per 256-bit register versus 4 for double, so a
  packet or 8-wide-BVH traversal doubles its throughput too. Not implemented here.

---

## Q2 — Why does `Vec3::operator[]` use a conditional chain instead of `(&x)[i]`?

### Short Answer

`(&x)[i]` is undefined behaviour — pointer arithmetic is only defined *within* a single object, and
three separate members are not an array however they happen to be laid out. The conditional chain
is well-defined, still returns a real reference because a ternary over lvalues is an lvalue, and
compiles to the same thing.

### Deep Answer

The `(&x)[i]` trick is everywhere in graphics code and it does work on every compiler anyone has
tried. That is exactly what makes it worth talking about: it is a case where "it works" and "it is
correct" come apart.

The standard's rule is that `p + n` is only defined when `p` points into an array object (or one
past its end), and a scalar is treated as an array of length 1 for this purpose. `&x` points at a
single `Scalar`, so `(&x)[1]` is already past the end of the only object it is allowed to walk. The
layout of `Vec3` may well be three contiguous floats — it almost certainly is — but layout is not
the question; the abstract machine's object model is. A compiler entitled to assume the access stays
in bounds may reorder it against a store to `y`, or fold an alias analysis result that assumes
`&x + 1` cannot be `&y`.

The alternative costs nothing:

```cpp
constexpr Scalar& operator[](int i) {
    assert(i >= 0 && i < 3);
    return i == 0 ? x : (i == 1 ? y : z);
}
```

Two subtleties make this work. First, a conditional expression whose second and third operands are
lvalues of the same type is itself an lvalue, so the function genuinely returns a reference to the
member and `v[i] = 3.0f` works. Second, when `i` is a compile-time constant the whole chain folds
away; for a runtime index the compiler emits a cmov or a small branch.

The reason `Vec3` needs indexed access at all is worth saying, because otherwise the whole question
looks academic: the slab test loops over axes and reads `box.min[axis]` with a runtime `axis`, and
BVH split-axis selection will do the same. Named members are what you want for readability
(`v.x` beats `v[0]`), indexed access is what algorithms need, and this is how you get both.

### Code Connection

- The implementation and both overloads: `include/geometry/vec3.hpp:32-39`; the reasoning,
  including the lvalue-ternary point and the codegen expectation, at `:11-22`.
- Same pattern for `Vec4`: `include/geometry/vec4.hpp:25-32`.
- Why indexed access is required at all: `intersectRay` reads `box.min[axis]` / `box.max[axis]` on a
  runtime axis — `include/geometry/aabb.hpp:177-178`.
- `maxAxis` returns exactly such a runtime index — `include/geometry/vec3.hpp:114-117`.
- Tested against named members at `tests/test_vec3.cpp:20`.

### Tradeoffs

| Approach | Defined? | Notes |
|---|---|---|
| `(&x)[i]` | **No** | Works in practice; UB in principle; UBSan and strict aliasing can both object |
| Conditional chain (chosen) | Yes | Same codegen expected; slightly more source |
| `Scalar v[3]` storage + named accessors | Yes | Loses `v.x` as a plain member; accessors everywhere |
| Anonymous union of struct + array | **No** (in C++) | Type-punning through a union is UB in C++, unlike C; the most popular "fix" is also not a fix |
| `std::array<Scalar,3>` + `x()`/`y()`/`z()` | Yes | Clean, but every call site gains parentheses |

The codegen-equivalence claim is `UNVERIFIED` — no disassembly is recorded in the repository. If
pressed, the correct answer is "I expect identical codegen and I would check with `-S` before
claiming it."

### Follow-up Questions

- *"Does the anonymous-union trick fix it?"* Not in C++ — reading a union member other than the one
  last written is UB. It is legal in C, which is why so much graphics code inherited it.
- *"What about `std::launder` / `reinterpret_cast`?"* Neither creates an array object that wasn't
  there. `std::launder` reinterprets a pointer to an object that exists; it does not make three
  members into an array.
- *"Have you seen this actually break?"* Not in this project. I treat it the way I treat the
  `1/0` case — the standard's guarantee is what survives an optimiser, not the observed behaviour.
- *"Is the `assert` free in release?"* Yes, `NDEBUG` removes it; it exists to catch a bad index
  during development, and out-of-range still falls through to `z`.

---

## Q3 — You divide by zero deliberately. Isn't that undefined behaviour?

### Short Answer

It is, which is why I don't. The slab test genuinely wants `±inf` for an axis-parallel ray, but
`x / 0` is UB per `[expr.mul]/4` regardless of operand type — the IEEE-754 guarantee is a property
of the hardware, not of the C++ abstract machine. `safeReciprocal` constructs the identical value
with `copysign` instead.

### Deep Answer

Start with why the infinity is wanted. When a ray has a zero direction component, its reciprocal is
infinite, and the slab products become `±inf`. Those compare correctly: the slab is either fully
entered or fully missed, and the interval arithmetic gets the right answer with no branch. Branching
to special-case axis-parallel rays inside traversal would cost more than it saves, because it is
a per-node cost to avoid a per-ray one.

So the goal is not to avoid infinity. The goal is to obtain it legally.

`[expr.mul]/4` says: *"If the second operand of `/` or `%` is zero, the behavior is undefined."* No
exception for floating point. IEEE-754 does define `1.0/0.0` as `+inf` and your CPU implements
that — but UB in C++ is not about what the hardware does, it is about what the optimiser is
entitled to assume. A compiler that sees `1.0f / d` may assume `d != 0` from that point forward and
delete a later check, or constant-fold the expression under that assumption. UBSan's
`float-divide-by-zero` check flags it, and that is how this was found in this project.

The fix produces the same bits without the division:

```cpp
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}
```

`copysign` rather than a bare `kInfinity` because `-0.0` must give `-inf`, matching `1 / -0.0`. That
matters: the sign determines which slab plane is the entry plane, so getting it wrong on a negative
zero direction flips the entry and exit for that axis.

The cost is one predictable branch per component, three per ray, computed once in `invDirection`
and reused across every node the ray visits. It never enters the traversal inner loop.

The general principle is the thing worth carrying into an interview: **"the hardware defines this"
and "the abstract machine defines this" are different claims, and only the second survives an
optimiser.** The same principle explains two other decisions in this codebase — the `(&x)[i]`
avoidance (Q2) and the absolute ban on `-ffast-math`, which would let the compiler assume the NaNs
and infinities the slab test depends on never occur. Three instances of one idea is a theme; one
instance is trivia.

### Code Connection

- `safeReciprocal`: `include/geometry/scalar.hpp:58-62`; the `[expr.mul]/4` reasoning at `:50-54`;
  the `copysign`-for-negative-zero note at `:60`; the cost analysis at `:56-57`.
- Called once per ray: `invDirection` — `include/geometry/ray.hpp:48-51`, with the "branching in
  traversal would cost more than it saves" argument at `:43-47`.
- The slab test consuming the infinities: `include/geometry/aabb.hpp:151-155`.
- `-ffast-math` ban: `CMakeLists.txt:81-84`, restated as a project rule in `CLAUDE.md`.
- The IEEE requirement is asserted, not assumed:
  `static_assert(std::numeric_limits<Scalar>::is_iec559, ...)` — `include/geometry/scalar.hpp:26-27`.
- Tests: infinities with the right sign at `tests/test_ray.cpp:42-51`; finite components unaffected
  at `:53-59`.

### Tradeoffs

| Approach | Verdict |
|---|---|
| `1.0f / d` | Simplest, correct on hardware, UB in C++, flagged by UBSan |
| Suppress the sanitizer (`__attribute__((no_sanitize))`) | Hides the report without fixing the UB — and loses the check for genuine bugs elsewhere |
| Clamp `d` away from zero (`d = max(d, 1e-8)`) | Removes the UB but introduces a *fictitious direction*: `t` values become subtly wrong instead of correctly infinite |
| Branch inside the traversal loop | Moves a 3-per-ray cost to a 3-per-node cost |
| `safeReciprocal` (chosen) | Same value, defined semantics, one predictable branch per ray |

### Follow-up Questions

- *"Is integer division by zero different?"* It is UB too, but it traps on x86 (`#DE`) rather than
  producing a value, so it fails loudly. Float UB is worse precisely because it looks like it works.
- *"Is `0.0/0.0` also UB?"* Yes, same clause, and the "correct" IEEE answer would be NaN.
- *"What else does `-ffast-math` break here?"* It permits assuming no NaN and no inf, which turns
  the NaN-tolerant ternaries at `include/geometry/aabb.hpp:200-201` into dead code the compiler may
  simplify — and it can reassociate the subtract-then-multiply, invalidating the `gamma(3)` bound.
  This is the rare flag that turns a correct program into an incorrect one rather than a faster one.
- *"How did you find it?"* UBSan, via `BVH_ENABLE_SANITIZERS` (`CMakeLists.txt:86-91`), while
  writing the axis-parallel ray tests.

---

## Q4 — How do floating-point errors affect intersection tests?

### Short Answer

Two ways, and both are handled explicitly. Axis-parallel rays can produce `0 * inf = NaN`, so every
comparison in the slab loop is written NaN-tolerant — a NaN leaves the interval unconstrained rather
than rejecting the box. And each `t` is a subtract plus a multiply, so it carries relative error up
to `gamma(3)`; the exit distance is widened by that bound so a ray through a shared face between
sibling nodes cannot miss both.

### Deep Answer

**Hazard 1: NaN from `0 * inf`.** A zero direction component gives an infinite reciprocal. Normally
fine — the products are `±inf` and compare correctly. But if the ray origin lies *exactly* on a slab
plane, the numerator `box.min[axis] - origin[axis]` is exactly zero too, and `0 * inf` is NaN.

NaN makes every comparison false, which is either a disaster or a tool depending on how you write
the code. Here it is used as a tool:

```cpp
t0 = tNear > t0 ? tNear : t0;
t1 = tFar  < t1 ? tFar  : t1;
```

If `tNear` is NaN, `tNear > t0` is false and `t0` keeps its previous value. The degenerate axis
contributes nothing — the test simply does not constrain the interval there. That is conservative:
it may report a hit that exact arithmetic would not, which costs one wasted narrow-phase test, but
it can never produce a false miss.

This is why `std::max` and `std::min` are explicitly banned at that line. Their NaN behaviour is not
guaranteed to match: `std::max(a, b)` returns `a < b ? b : a`, whose result with a NaN operand
depends on which operand is NaN, and an implementation is free to differ. The ternary makes the
intent — "on NaN, keep what I had" — explicit and unambiguous.

**Hazard 2: rounding at grazing angles.** `t = (plane - origin) * invDir` is two operations, so by
Higham's standard bound the relative error is at most `gamma(3)`, where

```cpp
gamma(n) = (n * e) / (1 - n * e),   e = epsilon/2
```

For float that is about `1.8e-7`. Tiny — but the failure mode it causes is not tiny. Consider two
sibling BVH nodes that share a face, and a ray passing exactly through that face. Rounding can push
the computed exit of the left child fractionally short and the computed entry of the right child
fractionally long, so the ray is rejected by *both* and the geometry behind the face is never
tested. That is a visible crack in the rendered image, and it is the classic watertightness bug.

The fix is to widen the interval in the safe direction:

```cpp
tFar *= Scalar(1) + Scalar(2) * gamma(3);
```

Now the test errs toward reporting a hit. Wasted work is recoverable; a hole is not.

Two honest caveats. First, only `tFar` is widened — `tNear` is left alone. That is the published
PBRT form and it closes the exit side, which is where the sibling-crack arises. Second, **no test in
this milestone exercises the widening**: removing that line would still pass all 105 tests. I know
this because I looked; it is a gap, not a claim.

**The general policy** is that there is no single global epsilon in intersection code. `kEpsilon`
(`1e-6`) exists for "are these two quantities the same" comparisons and is explicitly kept out of the
intersection routines, because an absolute epsilon is meaningless across magnitudes — too loose near
the origin, too strict at coordinate 10⁵. `gamma(3)` is *relative*, so it scales with the quantity
it corrects. That distinction is the real answer to this question.

### Code Connection

- Hazard analysis written out in full: `include/geometry/aabb.hpp:149-164`.
- NaN-tolerant comparisons and the `std::max`/`std::min` ban:
  `include/geometry/aabb.hpp:198-201`.
- `gamma(n)` with the Higham bound stated: `include/geometry/scalar.hpp:37-45`.
- The widening: `include/geometry/aabb.hpp:196`.
- "No single global epsilon" policy: `include/geometry/scalar.hpp:32-34`.
- IEEE requirement asserted at compile time: `include/geometry/scalar.hpp:22-27`.
- Absolute-or-relative `nearlyEqual` for non-hot-path comparisons:
  `include/geometry/scalar.hpp:67-74`.
- Test for the exact `0 * inf` case — origin on the `y = 1` face, no `y` direction:
  `tests/test_aabb.cpp:207-216`.
- Overflow avoidance elsewhere: `AABB::centroid` is `min + 0.5*(max-min)` rather than
  `0.5*(min+max)` — `include/geometry/aabb.hpp:56-59`.

### Tradeoffs

| Approach | Consequence |
|---|---|
| Ignore both hazards | Occasional false misses; cracks between siblings; NaN-dependent behaviour |
| Widen the exit by `gamma(3)` (chosen) | Slightly conservative; a few wasted narrow-phase tests |
| Widen both ends | More conservative still; more wasted tests for no additional safety on the crack case |
| Global absolute epsilon | Wrong at both ends of the magnitude range |
| Exact / robust predicates (adaptive precision) | Exact answers, much slower; the right tool for mesh booleans, overkill for an acceleration structure |
| Double precision | Shrinks the error but does not eliminate it, and costs the node density from Q1 |

### Follow-up Questions

- *"Why widen only `tFar`?"* It closes the sibling-crack case, which arises on the exit side, and
  it is the published form. Widening `tNear` too would be more conservative at the cost of more
  false positives — I would want a measurement before changing it.
- *"What if the ray is exactly on an edge between three boxes?"* Same mechanism; the widening is
  per-axis, so a corner case is covered on each axis independently.
- *"Would a robust/watertight ray-triangle test be needed too?"* Yes, and that is a separate
  problem — the shared-edge case between two triangles. Not implemented; ray/triangle intersection
  is not in Phase 1.
- *"How would you test the widening?"* Construct two adjacent boxes sharing a face and a ray whose
  entry lands exactly on that plane at a magnitude where the float spacing exceeds the error, then
  assert at least one box reports a hit. Worth adding.

---

## Q5 — What happens with degenerate geometry?

### Short Answer

Every degenerate case has a defined, finite result and a named test. Zero-area triangles return a
zero normal rather than NaN and still have valid bounds and centroids, so a BVH can carry them
without special-casing; flat AABBs are treated as real zero-volume regions and remain hittable; and
`AABB::offset` returns zero on a degenerate axis rather than dividing by zero.

### Deep Answer

Degenerate input is not hypothetical. Exported meshes routinely contain zero-area triangles from
welded or duplicated vertices, and the code says so at the point where it handles them. The design
rule is: *never return NaN, and never silently drop data.*

**Zero-area triangles.** `normal()` uses `normalizeSafe`, which checks squared length against a
squared tolerance and returns a fallback (the zero vector) rather than dividing by a zero length.
The degeneracy test avoids a square root by working on the squared cross-product magnitude:

```cpp
bool isDegenerate(Scalar tol = kEpsilon) const {
    return lengthSquared(normalUnnormalized()) <= (Scalar(2) * tol) * (Scalar(2) * tol);
}
```

Worth being able to derive on a whiteboard: `normalUnnormalized()` is `cross(v1-v0, v2-v0)`, whose
magnitude is *twice* the triangle area. So the condition is `(2·area)² ≤ (2·tol)²`, i.e.
`area ≤ tol`. **`tol` is a tolerance on area**, defaulting to `1e-6`.

The crucial design point is what the comment says next: such triangles "cannot be hit by a ray in
any meaningful sense, but they still have valid bounds and a valid centroid, so a BVH can carry
them without special-casing — they simply never report a hit." That is a much better answer than
"I filter them out": filtering changes the primitive count, which silently changes every benchmark
you then compare against. `Mesh::countDegenerateTriangles` exists so the quality of the input is
*visible* rather than quietly corrected.

**Flat AABBs.** A zero-thickness box is a real region, unlike an empty box, and the code keeps that
distinction sharp. `surfaceArea()` of a 2×3×0 box is 12 — twice the face — while its volume is 0,
and the slab test's final comparison is strict (`if (t1 < t0) return false`) specifically so a flat
box with `t1 == t0` still reports a hit. If that were `<=`, every axis-aligned planar primitive
would become invisible.

**Degenerate axes in binning.** `AABB::offset` normalises a point to `[0,1]` per axis, and guards
each divide:

```cpp
if (max.x > min.x) o.x /= (max.x - min.x);
```

SAH binning hits this the moment all centroids share a coordinate — a planar mesh, or the last few
primitives in a subdivision. The test is named for exactly that situation.

**Elsewhere:** a zero-length rotation axis yields identity rather than NaN; collinear or coincident
points give a degenerate plane with a zero normal rather than NaN; a singular matrix makes `invert`
return `false` with the output untouched rather than producing inf.

**One gap I know about.** `AABB::centroid()` on an *empty* box returns NaN: `min + (max - min)*0.5`
with `min = +inf, max = -inf` gives `+inf + (-inf) = NaN`. `surfaceArea` and `volume` guard against
empty; `centroid` does not, and no test covers it. BVH construction computes centroid bounds over
primitives, so this is worth fixing before Phase 2. I would rather state that than claim complete
coverage.

### Code Connection

- `Triangle::isDegenerate` and the "a BVH can carry them" rationale:
  `include/geometry/triangle.hpp:44-53`; `normal()` via `normalizeSafe` at `:37-40`;
  `normalUnnormalized` magnitude note at `:31-35`.
- `normalizeSafe`: `include/geometry/vec3.hpp:87-94`; the asserting `normalize` and its documented
  precondition at `:78-85`.
- Degenerate vs empty box distinction: `include/geometry/aabb.hpp:37-39`.
- Strict `<` so flat boxes stay hittable: `include/geometry/aabb.hpp:203-204`.
- `AABB::offset` guards: `include/geometry/aabb.hpp:110-115`, with the SAH-binning motivation at
  `:107-109`.
- Degenerate rotation axis → identity: `src/geometry/mat4.cpp:30-31`.
- Degenerate plane, not NaN: `include/geometry/plane.hpp:27-33`.
- Singular matrix rejected rather than producing inf/NaN: `src/geometry/mat4.cpp:132-133`.
- Report-don't-drop: `Mesh::countDegenerateTriangles` — `include/geometry/mesh.hpp:80-83`,
  `src/geometry/mesh.cpp:38-45`.
- Tests: `tests/test_triangle.cpp:82` (bounds still valid), `:91` (normal zero not NaN), `:100`
  (duplicate vertices); `tests/test_aabb.cpp:65` (flat area), `:142` (degenerate-axis offset),
  `:235` (flat box hittable); `tests/test_plane.cpp:30`, `:36`; `tests/test_mat4.cpp:86`, `:146`;
  `tests/test_mesh.cpp:114`.

### Tradeoffs

| Policy for degenerate triangles | Consequence |
|---|---|
| Carry them, never hit (chosen) | No special case anywhere; primitive count is honest; a few zero-area leaves |
| Filter at load | Cleaner tree, but the primitive count no longer matches the file, which corrupts benchmark comparisons |
| Assert / reject the mesh | Rejects real-world assets; most exported meshes contain some |
| Snap or weld vertices | Changes the geometry; a repair step, not a geometry-core decision |

Similarly for empty boxes: returning 0 from `surfaceArea` keeps SAH cost sums finite; returning NaN
or a negative area would poison every cost comparison up the tree.

### Follow-up Questions

- *"How do you pick the degeneracy tolerance?"* It is an absolute area tolerance (`1e-6`), so it is
  scale-dependent — a correct triangle in a millimetre-scale model could be flagged. A scale-relative
  tolerance (against the mesh bounds diagonal) would be more robust. Not implemented.
- *"What if a whole mesh is planar?"* The root AABB is flat on one axis: surface area is non-zero,
  volume is zero, it is still hittable, and `offset` returns 0 on the flat axis so binning still
  works. Every piece of that path is tested.
- *"Would a degenerate triangle break ray/triangle intersection?"* Möller–Trumbore's determinant is
  zero for a degenerate triangle, and the standard epsilon test rejects it — so it never reports a
  hit, consistent with the `isDegenerate` comment. Not implemented in Phase 1.
- *"What's the worst degenerate case you didn't handle?"* `AABB::centroid()` on an empty box returns
  NaN. Known, documented, untested, unfixed.

---

## Q6 — Why AABBs, and not OBBs, spheres, or k-DOPs?

### Short Answer

The bounding volume is tested far more often than it is built, so what matters is the per-test cost,
not the tightness. An AABB/ray test is a handful of multiplies and comparisons with no matrix and no
trigonometry, and AABBs are closed under union via component-wise min/max — which is what makes
bottom-up bounds propagation in a BVH essentially free. The cost is a looser fit for diagonal
geometry, which shows up as extra false-positive node visits.

### Deep Answer

Frame it as an amortisation question. A bounding volume for a BVH node is built **once** and tested
**millions of times**, so the objective function is

```
total cost  ≈  N_tests × cost_per_test  +  (false positives) × cost_of_narrow_phase_test
```

Tighter volumes reduce the second term and increase the first. For ray queries against a BVH the
first term dominates overwhelmingly, because the whole point of the hierarchy is that most tests
are rejections at internal nodes.

**Against OBBs.** An oriented box is tighter — sometimes dramatically, for a long diagonal object —
but each test needs the ray transformed into the box's local frame first, which is a matrix
multiply before you can even start the slab test. It also stores a rotation per node, inflating node
size and therefore hurting the cache-density argument from Q1. And critically for a BVH: **the union
of two OBBs is not an OBB.** Computing a tight parent OBB from two children is an optimisation
problem, not two `min`/`max` calls, so bottom-up bounds propagation stops being cheap. OBBs earn
their keep in collision detection with a small number of rigid bodies, not in a per-node
acceleration hierarchy.

**Against spheres.** The cheapest test of all, and the union of two spheres is easy — but the fit is
terrible for anything elongated, and most triangles are elongated relative to a sphere. A sphere
bounding a thin diagonal triangle wastes enormous volume, and wasted volume is false-positive
descents.

**Against k-DOPs.** A discrete oriented polytope with `k` fixed plane normals (typically 14, 18 or
26) is the principled middle ground: tighter than an AABB, still closed under union component-wise
because the normals are shared and fixed. But cost and storage scale with `k`: a 26-DOP is 13 slab
tests instead of 3, and 26 scalars per node instead of 6. You are trading exactly the thing Q1 says
is scarce — bytes per node — for tightness.

**What AABBs give you structurally**, beyond the cheap test:

1. *Closed under union, component-wise.* `extend` is two `min`/`max` calls and no branch. That is
   what makes bottom-up bounds propagation in a BVH cheap.
2. *Surface area is a closed form*, which is what makes the SAH computable at all (see Q7).
3. *`offset` maps a point to a bin index in three divides*, which is what makes binned SAH cheap.

**The honest cost** is stated in the source: looser fit for diagonal geometry, appearing as more
false-positive node visits. Quantifying that against an OBB tree for this project would need
measurement — `UNVERIFIED`, no benchmark exists.

### Code Connection

- The full argument, in the header where the decision lives:
  `include/geometry/aabb.hpp:11-20`.
- Branch-free component-wise union: `include/geometry/aabb.hpp:42-52`, relying on `minComponents`/
  `maxComponents` at `include/geometry/vec3.hpp:96-101`.
- The test that pins why union-with-empty must be the identity — named for its BVH consumer:
  `tests/test_aabb.cpp:48-56` ("Relied on by bottom-up bounds propagation in BVH construction").
- Closed-form surface area: `include/geometry/aabb.hpp:69-73`.
- `offset` for bin indexing: `include/geometry/aabb.hpp:110-116`.
- The cheap test itself: `include/geometry/aabb.hpp:171-209`, "no divides, because the caller
  supplies the reciprocal direction" at `:146-147`.
- 24 bytes per box: `include/geometry/aabb.hpp:28-29`.

### Tradeoffs

| Volume | Test cost | Storage (float) | Union | Fit |
|---|---|---:|---|---|
| **AABB (chosen)** | ~6 mul, ~6 sub, compares | 24 B | component-wise min/max, branch-free | loose for diagonal geometry |
| Sphere | cheapest | 16 B | easy, but loose | very poor for elongated shapes |
| OBB | + a transform into local frame | 24 B + rotation | **not closed** — optimisation problem | tightest of these |
| k-DOP (k=14…26) | k/2 slab tests | 4k B | component-wise (normals fixed) | between AABB and OBB |

### Follow-up Questions

- *"When would you switch to OBBs?"* For collision between a small number of rigid bodies where the
  geometry is strongly oriented and the tree is shallow — the build-time cost is amortised over
  fewer, more expensive tests. Not for a per-node ray-query hierarchy.
- *"What about compressed / quantised node bounds?"* That is the real production answer to the
  tightness-versus-bytes tension: store child bounds as 8-bit offsets from the parent. It needs
  directed rounding (min down, max up) to stay conservative, and it is a Phase 2+ optimisation.
- *"Does AABB tightness depend on the mesh's orientation?"* Yes — that is exactly the weakness.
  A 45°-rotated thin plate has an AABB many times its actual volume. An interesting experiment for
  the explorer: rotate the scene and watch node-visit counts change with no change to the geometry.
- *"How would you measure the false-positive cost?"* Instrument traversal to count nodes visited and
  triangles tested per ray. Phase 5 of the plan calls for exactly that, against a brute-force
  baseline.

---

## Q7 — Why is surface area the cost term in the SAH?

### Short Answer

For a convex volume, the probability that a uniformly distributed random ray which hits the parent
also hits a child is the ratio of their **surface areas**, not their volumes. So the expected number
of primitive tests for a split is the area-weighted sum of child primitive counts, and minimising
that expectation is exactly what the SAH does.

### Deep Answer

This is a geometric-probability result. For a convex body in 3D, the measure of the set of lines
intersecting it — under the unique rigid-motion-invariant measure on lines — is proportional to its
surface area. (It is a corollary of Cauchy's formula: the mean projected area of a convex body over
all directions is one quarter of its surface area.) For a convex child `C` fully contained in a
convex parent `P`, a uniformly distributed ray conditioned on hitting `P` hits `C` with probability

```
P(hit C | hit P) = SA(C) / SA(P)
```

That single fact is the whole justification. Write the expected cost of a candidate split into
children `L` and `R`:

```
Cost(split) = C_trav + P(hit L)·N_L·C_isect + P(hit R)·N_R·C_isect
            = C_trav + [ SA(L)·N_L + SA(R)·N_R ] · C_isect / SA(P)
```

`SA(P)` and `C_isect` are constant across candidate splits of the same node, so choosing the best
split means minimising `SA(L)·N_L + SA(R)·N_R` — an **area-weighted primitive count**. Everything
else about SAH — the sweep, the binning, the bin count parameter — is machinery for evaluating that
objective quickly.

Now the "why not volume" part, which is where the question is usually really aimed. Volume is the
right measure for *point* containment queries: the probability a uniformly distributed point lands
in a child is the volume ratio. But rays are lines, not points, and the measure on lines is the
area one. Using volume gives you a heuristic with no probabilistic justification for ray queries,
and it fails badly on the exact case that matters: a flat box — a planar sheet of geometry — has
**zero volume but substantial surface area**. Under volume weighting its cost is zero, so the
builder would happily create such splits despite rays hitting that sheet constantly.

This project pins that case explicitly: a 2×3×0 box has surface area 12 and volume 0, and both are
asserted in a test named for the distinction.

Two implementation details in `surfaceArea` follow from its role as a cost term:

```cpp
constexpr Scalar surfaceArea() const {
    if (isEmpty()) return Scalar(0);
    const Vec3 d = max - min;
    return Scalar(2) * (d.x*d.y + d.y*d.z + d.z*d.x);
}
```

The empty guard exists so SAH cost sums stay finite — without it, an empty child's diagonal is
`-inf` and the sum becomes NaN, which would poison every comparison up the tree. And the function
is `constexpr` and branch-light because it is evaluated once per bin per axis per node during a
build.

**What does not exist yet:** the SAH itself. Only the cost term and the binning helper are here.
`UNKNOWN — not yet implemented`.

### Code Connection

- `surfaceArea` with the ray-hit-probability justification stated inline:
  `include/geometry/aabb.hpp:62-73` — "for a convex volume, the probability that a uniformly
  distributed random ray hitting the parent also hits the child is the ratio of their surface areas.
  That relationship is why SAH minimises area-weighted primitive counts rather than, say,
  volume-weighted ones."
- The empty guard and its reason ("so SAH cost sums stay finite"): `:68`, `:70`.
- `volume()` provided for reporting, not for cost: `include/geometry/aabb.hpp:75-79`.
- The flat-box case that distinguishes the two measures: `tests/test_aabb.cpp:65-72`
  (`SurfaceAreaOfFlatBoxIsTwiceTheFace`), area 12, volume 0.
- Known-answer area test (1×2×3 → 22): `tests/test_aabb.cpp:58-63`.
- `longestAxis` as the cheap proxy split axis when SAH is not used, and its rationale — "a cheap
  proxy for the axis whose split will most reduce child surface area":
  `include/geometry/aabb.hpp:81-83`.
- The binning helper SAH will need: `AABB::offset` — `include/geometry/aabb.hpp:107-116`.

### Tradeoffs

| Cost measure | Justification | Failure mode |
|---|---|---|
| **Surface area (chosen)** | geometric probability for lines | assumes uniformly distributed rays — real ray distributions are not uniform |
| Volume | correct for *point* queries | a flat box costs 0; planar geometry is split disastrously |
| Primitive count only (median split) | trivially cheap to build | ignores that a large box is hit more often; builds unbalanced-in-cost trees |
| Measured ray distribution | matches the real workload | needs a representative ray set at build time; not general-purpose |

The SAH's own assumptions are worth naming, because a good interviewer will: rays are uniformly
distributed and infinite; they do not terminate early on a hit; and the children's costs are
independent. All three are false in practice — primary rays come from a point, traversal shrinks
`tMax` on every hit — and SAH still wins comfortably, which is the interesting part.

### Follow-up Questions

- *"Where does the surface-area/probability result come from?"* Integral geometry — Cauchy's formula
  gives mean projected area = SA/4 for a convex body, and the measure of lines meeting a convex body
  is proportional to that.
- *"Why is the SAH assumption that rays don't terminate early wrong, and does it matter?"* Traversal
  shrinks `tMax` as hits are found, so a near child's real cost is lower than SAH assumes. It
  matters little in practice; front-to-back ordered traversal recovers most of it.
- *"What is `C_trav / C_isect` in your cost model?"* `UNKNOWN — not yet implemented`. It is the ratio
  a real builder must pick, and the leaf-size decision falls out of it.
- *"Why bin instead of doing a full sweep?"* A full sweep is O(n log n) per node from the sort; binning
  is O(n) per node with a small constant, at the cost of only evaluating `k` candidate planes. Bin
  count is a Phase 3 parameter (4/8/16/32 in the plan). Not implemented.
- *"Your `offset` returns 0 on a degenerate axis — what does that do to binning?"* Every centroid
  maps to bin 0, so that axis produces no valid split and the builder must fall back to another axis
  or make a leaf. Guarding the divide is what keeps it a *decision* rather than a NaN.

---

## Q8 — Why is an empty AABB represented as `min = +inf, max = -inf`?

### Short Answer

Because it makes `extend` branch-free. Merging the first point into an empty box gives exactly that
point's degenerate box, since `min(+inf, p) == p` and `max(-inf, p) == p`, and merging an empty box
into a real one is automatically a no-op. A zero-initialised box would instead wrongly contain the
origin.

### Deep Answer

`extend` is the innermost operation of bounds computation — it runs once per vertex when bounding a
mesh, and once per child at every node when propagating bounds up a BVH. Any branch in it is a
branch in the hottest loop of the build.

The inverted sentinel removes the branch by making the identity element of the union operation
*representable*. In algebraic terms: boxes under union form a monoid, and `[+inf, -inf]` is its
identity. Component-wise `min`/`max` then handle the empty case for free, with no test:

```cpp
void extend(const Vec3& p) { min = minComponents(min, p); max = maxComponents(max, p); }
void extend(const AABB& b) { min = minComponents(min, b.min); max = maxComponents(max, b.max); }
```

Both directions work. First point into an empty box: `min(+inf, p) = p`, `max(-inf, p) = p` — a
degenerate single-point box, correct. Empty box into a real box: `min(m, +inf) = m`,
`max(M, -inf) = M` — unchanged, correct. Neither case is special-cased anywhere in the file.

**Why the obvious alternatives are worse.** A zero-initialised box is not merely inelegant, it is
*silently wrong*: `AABB{{0,0,0},{0,0,0}}` already contains the origin, so bounding a mesh that sits
entirely at `x > 100` produces a root box stretching back to zero. Every node in the tree inherits
the inflation, every ray tests boxes that contain nothing, and there is no crash to tell you.

An explicit `bool empty` flag is correct but costs a branch in `extend` and widens the struct from
24 bytes to 28 (or 32 with alignment) — directly against the node-density argument from Q1.

**`isEmpty()` follows from the representation:** `min > max` on at least one axis. And the
representation forces a distinction the header is careful to draw: *empty* (not a region at all)
versus *degenerate* (a point or flat plane — a real, zero-volume region that legitimately
participates in intersection tests). The two behave oppositely in the ray test: an empty box is
never hit; a flat box is always hittable. Both are tested under those names.

**Where the sentinel bites.** It is not free — it interacts with arithmetic that assumes finite
corners:

1. The ray test must be written so the inverted interval *stays* inverted. Swapping on
   `tNear > tFar` repairs `[+inf, -inf]` into `[-inf, +inf]` and makes every empty box hit every
   ray. That is Q10, and it is the best story in this milestone.
2. `surfaceArea` and `volume` need an explicit empty guard, or `max - min` is `-inf` and the
   products are NaN.
3. **`centroid()` does not have that guard** and returns NaN for an empty box:
   `+inf + (-inf)*0.5 = NaN`. Known gap, untested, worth fixing before Phase 2 uses centroid bounds.

So the honest framing is: the sentinel buys a branch-free hot path and pays for it with three places
that must handle infinity deliberately. Two of them do; one does not.

### Code Connection

- The sentinel and the full rationale: `include/geometry/aabb.hpp:22-29` — "A zero-initialised box
  would instead wrongly contain the origin."
- Branch-free `extend`, both overloads, with the no-op-on-empty comment:
  `include/geometry/aabb.hpp:42-52`.
- `isEmpty` and the empty-vs-degenerate distinction: `include/geometry/aabb.hpp:37-40`.
- Empty guards that exist: `surfaceArea` `:70`, `volume` `:76`; the reason ("so SAH cost sums stay
  finite") at `:68`.
- The guard that is missing: `centroid` — `include/geometry/aabb.hpp:56-60`.
- Empty-set semantics elsewhere: `contains(AABB)` returns true for empty (`:93`), `intersects`
  returns false (`:101`).
- Tests: `tests/test_aabb.cpp:9-18` (sentinel values and zero measures), `:20-30` (first point gives
  a degenerate box), `:48-56` (union with empty is the identity — named for its BVH consumer),
  `:244-248` (empty box is never hit).

### Tradeoffs

| Representation | `extend` cost | Size | Correctness risk |
|---|---|---:|---|
| **Inverted sentinel (chosen)** | branch-free, 2 min + 2 max | 24 B | infinities must be handled in every arithmetic path |
| `bool empty` flag | a branch per extend | 28–32 B | none, but the branch is in the hottest loop |
| Zero-initialised | branch-free | 24 B | **silently wrong** — contains the origin |
| `std::optional<AABB>` | branch + unwrapping everywhere | 28–32 B | none; heavy at every call site |
| Separate `EmptyAABB` type | none at runtime | 24 B | type proliferation; conversions everywhere |

### Follow-up Questions

- *"How does an empty box behave in the ray test?"* It must never hit. That is not automatic — see
  Q10.
- *"What is the difference between empty and degenerate?"* Empty is "not a region"; degenerate is a
  real region of zero volume. A flat box has surface area 12 and is hittable; an empty box has area
  0 and is not.
- *"Does `merge(empty, empty)` work?"* Yes, it stays empty — tested at `tests/test_aabb.cpp:55`.
- *"Why does `contains(empty)` return true but `intersects(empty)` return false?"* Empty-set
  semantics: the empty set is a subset of everything and intersects nothing. Both are deliberate and
  tested.
- *"Any bugs from the sentinel?"* Yes, two: the swap bug (Q10), and `centroid()` returning NaN,
  which is still open.

---

## Q9 — Walk me through the slab method.

### Short Answer

An AABB is the intersection of three axis-aligned slabs. For each axis I compute the two `t` values
where the ray crosses that slab's planes, intersect the running `[tEnter, tExit]` interval with
them, and bail out as soon as the interval inverts. Six multiplies, six subtracts, a few compares,
and no divides — the caller supplies the reciprocal direction.

### Deep Answer

**The geometry.** The box `[min, max]` is `{p : min ≤ p ≤ max}` componentwise, which is exactly the
intersection of three slabs, each the region between two parallel planes. A ray hits the box iff the
parameter intervals during which it is inside each slab have a common point. So: intersect three
intervals and see whether anything survives.

**The loop.**

```cpp
Scalar t0 = tMin;                     // running entry
Scalar t1 = tMax;                     // running exit
for (int axis = 0; axis < 3; ++axis) {
    Scalar tNear = (box.min[axis] - origin[axis]) * invDir[axis];
    Scalar tFar  = (box.max[axis] - origin[axis]) * invDir[axis];
    if (invDir[axis] < Scalar(0)) { swap(tNear, tFar); }
    tFar *= Scalar(1) + Scalar(2) * gamma(3);
    t0 = tNear > t0 ? tNear : t0;     // intersect: entry = max
    t1 = tFar  < t1 ? tFar  : t1;     // intersect: exit  = min
    if (t1 < t0) return false;
}
```

Note `t0` starts at the ray's own `tMin` and `t1` at its `tMax`, so the ray's valid range is just a
fourth interval in the same intersection — no separate range check is needed. And on success `t0` is
returned as the entry distance, which for a ray originating *inside* the box is `tMin`, not a
negative backward distance.

**Cost.** Two subtracts and two multiplies per axis for the `t` values, plus the widening multiply,
plus three compares. Six subtracts and six multiplies over three axes — no divides, because
`invDir` was computed once per ray. That is the point of precomputing the reciprocal: it turns three
divisions per box into three multiplications, and a box test happens once per node visited.

The early-out at `t1 < t0` matters more than it looks: most tests in a BVH are rejections, and a
ray that misses on the first axis pays for one axis, not three.

**Why the swap is on `sign(invDir)` and not `tNear > tFar`** — that is Q10, and it is the most
interesting line in the function.

**The two numerical hazards** — `0 * inf = NaN` and the `gamma(3)` widening — are Q4.

**Why the final comparison is strict.** `t1 < t0`, not `t1 <= t0`, so a flat box where `t1 == t0`
still reports a hit. Planar geometry stays visible.

### Code Connection

- The routine: `include/geometry/aabb.hpp:171-209`; the full derivation and hazard analysis in the
  comment block at `:141-170`.
- Cost claim ("6 multiplies, 6 adds... no divides, because the caller supplies the reciprocal
  direction"): `:146-147`. Reading the loop confirms the arithmetic; the widening multiply at `:196`
  adds one more per axis that the comment does not count.
- Interval initialised from the ray's own range: `:173-174`; `tEnter` semantics at `:166-167`.
- Early-out: `:204`. Strict `<` for flat boxes: `:203-204`.
- Precomputed reciprocal, once per ray: `include/geometry/ray.hpp:38-51`.
- Convenience overload that recomputes `invDirection` per call — fine for tests, **not** for
  traversal: `include/geometry/aabb.hpp:211-213`.
- `Ray::tMax` is carried in the ray so traversal can shrink it as closer hits are found:
  `include/geometry/ray.hpp:21-24`.
- Twelve ray-intersection tests: `tests/test_aabb.cpp:152-248` — hit in front, miss to the side,
  miss behind, origin inside, `tMin`/`tMax` clipping, axis-parallel outside, origin exactly on a
  slab plane, negative direction, diagonal corner hit, flat box, empty box.

### Tradeoffs

| Alternative | Consequence |
|---|---|
| Slab method (chosen) | Branch-light, no divides, uniform across axes, vectorises well |
| Divide inside the test | Three divides per box instead of three multiplies per ray |
| Separate-axis / plane-by-plane tests | More branches, worse for SIMD |
| Precomputed sign-based min/max corner lookup | Avoids the swap entirely by indexing `bounds[sign[axis]]`; needs a 2-element bounds array and a per-ray sign vector — a real alternative, arguably cleaner, and it sidesteps the Q10 bug class by construction |
| Branchless `std::max`/`std::min` | **Rejected here** — NaN behaviour is not guaranteed to match the required semantics |

The sign-lookup variant is the one to mention if asked "how else could you write it": storing the
box as `Vec3 bounds[2]` and reading `bounds[sign[axis]][axis]` replaces the swap with an index. It
is a genuine improvement in that it makes the swap bug impossible, at the cost of a per-ray sign
vector and a less readable box type.

### Follow-up Questions

- *"Why is `tEnter` returned at all?"* Front-to-back ordered traversal: visit the nearer child
  first, and skip the farther subtree entirely once a hit closer than its entry distance is found.
  That is the single most effective pruning mechanism in a BVH, and it is why `tMax` lives in the
  `Ray`.
- *"How would you SIMD this?"* Two ways: 4 or 8 rays against one box (packet traversal), or one ray
  against 4/8 children of a wide BVH node. The second is the modern choice and it wants a
  structure-of-arrays node layout. Not implemented.
- *"Does it work for a ray starting inside the box?"* Yes — `t0` never drops below `tMin`, so the
  reported entry is `tMin` rather than a negative value. Tested.
- *"What if `tMin > tMax` on the ray itself?"* The first axis comparison rejects it immediately,
  since the interval starts already inverted.
- *"Where does this get called from?"* Nowhere yet — BVH traversal is Phase 2.
  `UNKNOWN — not yet implemented`.

---

## Q10 — Tell me about a bug you found and fixed.

> Two, and they are the strongest material in this milestone. Both were found while writing tests
> for Phase 1, both are recorded in the `841777e` commit message and in the source comments, and —
> because Phase 1 landed as one commit — neither appears as a diff. The buggy versions never got
> committed.

### Short Answer

The slab test originally swapped `tNear`/`tFar` on the condition `tNear > tFar`, which is the form
you see in most published implementations. It is wrong in combination with an inverted-interval
empty box: the swap "repairs" `[+inf, -inf]` into `[-inf, +inf]`, so **every empty box reports a hit
against every ray**. Swapping on the sign of the reciprocal direction instead leaves the inverted
interval inverted, so the `tExit < tEnter` check rejects it. Same instruction count, one fewer way to
be wrong.

### Deep Answer

**The setup.** An empty `AABB` is `min = +inf, max = -inf` — the inverted sentinel from Q8. Take a
default-constructed box and a ray with positive `x` direction, so `invDir.x > 0`:

```
tNear = (+inf - origin.x) * invDir.x = +inf
tFar  = (-inf - origin.x) * invDir.x = -inf
```

**With the comparison-based swap:** `tNear > tFar` is `+inf > -inf`, which is true. Swap. Now
`tNear = -inf`, `tFar = +inf`. The interval `[-inf, +inf]` constrains nothing:
`t0 = max(tMin, -inf) = tMin`, `t1 = min(tMax, +inf) = tMax`. Nothing changed. All three axes behave
identically, `t1 < t0` never fires, and the function **returns true**.

The `if` intended to repair a negative-direction ordering has instead repaired the sentinel that
encodes "this box is not a region at all".

**With the sign-based swap:** `invDir.x` is positive, so no swap. `t0 = max(tMin, +inf) = +inf`,
`t1 = min(tMax, -inf) = -inf`, and `t1 < t0` fires on the very first axis. Correctly rejected.

Check the other cases, because "it happens to work for one sign" is not a fix. Negative direction:
`tNear = -inf`, `tFar = +inf`, the sign test swaps them to `+inf`/`-inf`, rejection still fires.
Axis-parallel (`invDir = +inf`): the same `+inf`/`-inf` pair, same rejection. The sign form is
correct in every case, and it is correct for the ordinary non-empty box too, because *the sign of
the direction is what determines the crossing order in the first place* — `tNear > tFar` is a
*symptom* of a negative direction, and the sign is the *cause*. Testing the cause cannot be confused
by a degenerate input.

**Why this matters in a BVH, not just in a unit test.** Empty bounds are routine there: a node with
no primitives after a partition, a partially constructed tree, an unfilled child slot, a
conservatively initialised bin during SAH binning. If every empty node reports a hit, traversal
descends into it, tests whatever it finds, and comes back. **Nothing crashes. No pixel is wrong.**
The acceleration structure just quietly accelerates less, and the resulting performance cliff is
very hard to attribute — you would be profiling the traversal loop looking for a cache problem.

**Why it is a good interview story.** The two forms look equivalent. The buggy one is the one almost
everyone has seen and copied. The failure is silent and performance-only, which is the hardest class
of bug to find. It only manifests through an interaction between two *separately correct* design
decisions — the inverted-interval empty box and the standard slab test. And the fix costs exactly
nothing: one comparison either way, on a value already in a register. The comment ends with the
right summary — *"Same cost, one fewer way to be wrong."*

**The second bug**, briefly, since it pairs well: `invDirection` originally computed `1/d`
directly. For an axis-parallel ray that is a literal division by zero. IEEE-754 defines the result
as `±inf` and that is exactly the value the slab test wants — but `[expr.mul]/4` makes it undefined
behaviour regardless of operand type, and UBSan flagged it. The fix was not to suppress the check
but to construct the same value with `copysign`. Full treatment in Q3; the general principle —
"the hardware defines this" and "the abstract machine defines this" are different claims — is the
transferable part.

### Code Connection

- The fix and the full explanation of what was rejected:
  `include/geometry/aabb.hpp:189-193`, reasoning at `:180-188` — "The comparison form looks
  equivalent and is widely published, but it silently 'repairs' an inverted (empty) box into an
  infinite one."
- The sentinel that makes it possible: `include/geometry/aabb.hpp:28-29`.
- The check that does the rejecting: `include/geometry/aabb.hpp:204`.
- The guarantee it protects, stated in the header: `include/geometry/aabb.hpp:169-170`.
- Regression test: `tests/test_aabb.cpp:244-248` (`EmptyBoxIsNeverHit`). Negative-direction
  coverage — the case the swap exists for — at `:218-225`.
- Second bug: `safeReciprocal` — `include/geometry/scalar.hpp:47-62`; call site
  `include/geometry/ray.hpp:48-51`; tests `tests/test_ray.cpp:42-59`.
- Both recorded in the `841777e` commit message.
- The sanitizer that found the second one: `CMakeLists.txt:86-91`.

### Tradeoffs

Genuinely none on cost — both forms are one comparison and a conditional swap on values already in
registers. The alternatives are about *eliminating the bug class* rather than fixing this instance:

| Approach | Effect |
|---|---|
| Swap on `tNear > tFar` | The bug |
| Swap on `sign(invDir)` (chosen) | Correct; same cost; the sign is the cause, not a symptom |
| Guard with `if (box.isEmpty()) return false;` | Correct, but adds a branch to the hottest loop to work around a representation the rest of the file handles for free |
| Sign-indexed bounds array (`bounds[sign[axis]][axis]`) | Makes the bug unrepresentable; needs a different box layout and a per-ray sign vector |
| Don't use an inverted sentinel | Trades a branch-free `extend` for a branch-free ray test — the wrong trade, since `extend` runs per vertex and per node during builds |

### Follow-up Questions

- *"How did you find it?"* By writing `EmptyBoxIsNeverHit` as a degeneracy test, before there was
  any BVH to notice the symptom. That is the argument for testing degenerate inputs at the primitive
  level rather than waiting for the system to behave oddly.
- *"Would a fuzzer have found it?"* A random-box fuzzer, probably not — empty boxes have measure
  zero in any natural random distribution. You find this by enumerating *representation* edge cases,
  not value ranges.
- *"Is the published form wrong in general?"* No — it is correct for any box with `min ≤ max`. It is
  wrong *in combination with* an inverted-interval empty box. Two locally correct decisions, one
  bug at the interface. That is the most common shape for real bugs.
- *"What else in the codebase has this flavour?"* `AABB::centroid()` returns NaN for an empty box —
  the same interaction, same sentinel, still unfixed. I would rather name it than let you find it.
- *"Why did neither appear in the git history?"* Phase 1 landed as one commit, so the fixes are
  documented in the commit message and the code comments rather than as a diff. If I were doing it
  again I would commit the test-then-fix separately — the diff is more useful to a reviewer than the
  prose.

---

## Q11 — Why is `Mat4` column-major?

### Short Answer

To match GLSL and SPIR-V, so a `Mat4` can be memcpy'd straight into a Vulkan uniform buffer and used
by a shader with no transpose. The cost is that memory order no longer matches how a matrix is
written on paper, which is why every access site says `m[c][r]` explicitly rather than using a flat
16-element array.

### Deep Answer

GLSL's `mat4` is column-major, and the std140/std430 layout rules store each column consecutively.
If the CPU side is row-major, then every upload needs a transpose — a cost per matrix per frame, and
more importantly a *convention* that has to be remembered at every upload site. Forget it once and
you get geometry that is wrong in a way that looks almost right, which is among the more annoying
graphics bugs to chase. Matching the GPU's layout makes the upload a `memcpy` and removes the
question.

The storage is `Scalar m[4][4]` with the documented convention `m[column][row]`, so `m[0]` is the
first column and translation lives in the **fourth column** rather than the fourth row. The
associated convention is column-vector: a transform applies as `M * v`, and "first A, then B"
composes as `B * A`. Both are pinned by tests, which matters because a convention that is only in a
comment will drift.

The honest cost is readability: `m[1][2]` is column 1, row 2 — the transpose of how you would read
it off a blackboard. The code's mitigation is to never use a flat `Scalar m[16]` array, where
`m[6]` would be genuinely ambiguous, and to write `m[c][r]` everywhere so the two indices are always
distinguishable by name.

There is one place the code deliberately works against its own layout: `invert` copies into
row-indexed locals, because Gauss-Jordan elimination is naturally row-oriented, and transposes back
at the end. The comment says exactly that — "We transpose in and out rather than contort the
algorithm" — which is the right call: two transposes of a 4×4 in a function that runs once per
frame, versus an elimination loop written backwards forever.

**Caveat worth stating:** the GPU half of this is currently a *design intent*, not a demonstrated
fact. There is no Vulkan code in the tree, and the Vulkan SDK is not yet installable on this machine
because the Command Line Tools are too old. The decision is defensible on its merits; I have not yet
memcpy'd a matrix into a uniform buffer to prove it.

### Code Connection

- Storage, convention, and rationale: `include/geometry/mat4.hpp:11-23` — "so a Mat4 can be memcpy'd
  into a uniform buffer and consumed by a Vulkan shader with no transpose."
- Column-vector convention (`M * v`, `B * A` for "first A then B"):
  `include/geometry/mat4.hpp:19-20`.
- Translation in the fourth *column*: `src/geometry/mat4.cpp:11-14`.
- `fromColumns` builds in storage order: `include/geometry/mat4.hpp:36-46`.
- Multiplication in terms of the convention: `(a*b)[c][r] = sum_k a[k][r] * b[c][k]` —
  `src/geometry/mat4.cpp:57-68`.
- Working against the layout where it pays: `src/geometry/mat4.cpp:108-116`, `:159-160`.
- Tests that pin the conventions: `tests/test_mat4.cpp:28` (`ColumnMajorStorageLayout`), `:43`
  (column/row accessors), `:99` (`MultiplicationAppliesRightmostFirst`), `:200` (matrix-vector
  product matches manual expansion).
- Vulkan status: `UNKNOWN — not yet implemented`; toolchain blocker documented in `CLAUDE.md`.

### Tradeoffs

| Choice | Gain | Cost |
|---|---|---|
| **Column-major (chosen)** | memcpy to GLSL/SPIR-V; no transpose; matches GLM's default too | memory order ≠ written order; `m[c][r]` discipline required |
| Row-major | reads like the blackboard; matches DirectX/HLSL default and C 2D-array intuition | transpose on every GPU upload, or `transpose=true` at every upload site |
| Flat `Scalar m[16]` | one index, trivially memcpy'd | `m[6]` is ambiguous; the bug class this project avoids by construction |
| Row-major CPU + transpose on upload | natural CPU code | one more place to forget |

### Follow-up Questions

- *"Does column-major change the math?"* No — it is purely storage. What changes is which index you
  write first and whether you compose left or right.
- *"Why column-vector rather than row-vector?"* It pairs with column-major to make `M * v` a
  column-dot, and it matches GLSL and the standard mathematical convention. Row-vector plus
  row-major (the DirectX tradition) is equally self-consistent; mixing the two is what breaks.
- *"What about `transpose=true` in `glUniformMatrix4fv`?"* It exists precisely for this mismatch, but
  Vulkan has no equivalent — you write bytes into a buffer. So matching layout matters more with
  Vulkan than with OpenGL.
- *"How do you know the convention is right?"* `tests/test_mat4.cpp:28` asserts the storage layout
  directly, and `:99` asserts composition order. A convention that is only documented drifts.

---

## Q12 — Why Gauss-Jordan with partial pivoting instead of cofactor expansion?

### Short Answer

Stability, and because it is not a hot path. Cofactor expansion is faster for a fixed 4×4, but this
inverse is used to bring rays into object space, where numerical error becomes a *wrong
intersection* rather than a slightly wrong pixel. Transforms are built once per frame at most, not
once per ray, so I can afford the more stable algorithm.

### Deep Answer

The two candidates are genuinely different in character. Cofactor (adjugate) expansion is a closed
form: compute sixteen 3×3 minors, divide by the determinant. It is branch-free, unrolls beautifully,
and is the standard choice for a fixed 4×4 in a graphics library. But it computes the determinant as
a sum of products of matrix entries, and for a poorly conditioned matrix those products can
catastrophically cancel — you subtract two nearly equal large numbers and keep the noise.

Gauss-Jordan with partial pivoting has a different error profile. At each column it picks the row
with the largest magnitude in that column as the pivot, which bounds the multipliers used in
elimination by 1 and therefore bounds the error growth. Without pivoting, dividing by a near-zero
pivot amplifies whatever rounding error is already present; that is the failure the pivot search
exists to prevent.

The decision rule is the interesting part, and it is the same one used everywhere else in this
codebase: **how often does it run?** The reciprocal direction is precomputed per ray because it is
consumed per node. The AABB test is optimised because it runs millions of times. The matrix inverse
is *allowed to be slower* because it runs once per frame. Being able to say "I applied the same
criterion and it pointed the other way here" is a much stronger answer than "I optimise everything".

And the consequence of getting it wrong is asymmetric. If this inverse were only used for a camera
matrix, a little error is a sub-pixel shift nobody sees. But it is used to transform rays into object
space, and there error moves the ray, which moves the intersection point, which can move it onto the
wrong side of a surface.

**Implementation detail worth mentioning:** the algorithm operates on `[A | I]` in row-indexed local
copies — the transpose of the class's column-major storage — because elimination is naturally
row-oriented, and transposes back at the end. And it fails *loudly*: a best pivot of magnitude
`≤ 1e-20` returns `false` with the output untouched, rather than producing an inf/NaN matrix that
propagates silently.

**A limitation I would raise before being asked:** that threshold is *absolute*, not relative to the
matrix norm. A perfectly well-conditioned matrix that has simply been scaled down by `1e-10` would
be rejected. A scale-invariant test against the matrix norm would be better.

The claim that cofactor expansion is faster is `UNVERIFIED` — no measurement exists in this
repository. It is the conventional result and it follows from operation counts, but I have not timed
it here.

### Code Connection

- The decision and its reasoning: `include/geometry/mat4.hpp:88-98` — "slower but more stable, and
  it is not a hot path... error becomes a wrong intersection rather than a slightly wrong pixel."
- The algorithm: `src/geometry/mat4.cpp:105-162`. Pivot search and its justification at `:118-130`
  ("dividing by a near-zero pivot amplifies existing rounding error without it"). Singularity
  rejection at `:132-133`. Row swap `:135-140`, normalisation `:142-146`, elimination `:148-156`.
- Transposing in and out rather than contorting the algorithm: `:108-116`, `:159-160`.
- Failure contract — returns `false`, leaves `out` untouched:
  `include/geometry/mat4.hpp:96-97`.
- The unsafe convenience wrapper, explicitly marked as such:
  `include/geometry/mat4.hpp:100-102`, `src/geometry/mat4.cpp:164-170`.
- A cofactor-style expansion *is* present for `determinant`, which is used only for
  reporting/validation and says so: `src/geometry/mat4.cpp:85-87`.
- Tests: `tests/test_mat4.cpp:129` (`M·M⁻¹ = I`), `:139` (round-trips points), `:146`
  (rejects singular). The ray round-trip through an inverse composite transform:
  `tests/test_ray.cpp:93-106`.

### Tradeoffs

| Method | Speed | Stability | Notes |
|---|---|---|---|
| Cofactor / adjugate | fastest for fixed 4×4, branch-free, SIMD-friendly | worst — cancellation in the determinant | the usual graphics-library choice |
| **Gauss-Jordan + partial pivoting (chosen)** | slower; pivot search adds branches | good — multipliers bounded by 1 | ~4 passes × 4 rows × 4 cols |
| LU with partial pivoting | similar to G-J; better if you reuse the factorisation for many solves | same as G-J | overkill when you want the explicit inverse once |
| Full pivoting | slowest | best | unjustifiable for 4×4 transforms |
| Affine special case (invert 3×3 + negate translation) | much faster | fine | only valid for affine matrices; would need a separate path and a check |

That last row is the most interesting alternative: almost every matrix in this project *is* affine,
and an affine inverse is far cheaper. It is not implemented, because `transformPoint` already
supports projection matrices and a special-cased path would need a validity check and a second
codepath to test — for a function that runs once per frame.

### Follow-up Questions

- *"What is the condition number and why does it matter?"* `κ(A) = ‖A‖·‖A⁻¹‖`, the factor by which
  relative input error can be amplified in the output. A near-singular matrix has huge κ, which is
  when the choice of algorithm actually shows.
- *"Why not just invert the affine part directly?"* Cheaper and I would consider it if this ever
  showed up in a profile, but it needs an affine-validity check and it gives up support for
  projection matrices in the same function.
- *"Is `1e-20` a good threshold?"* It is absolute, so no — it is not scale-invariant. A relative test
  against the matrix norm would be better. Known limitation.
- *"Why does `inverse()` return identity on failure?"* It is the convenience wrapper: it asserts in
  debug and degrades to identity in release. The header explicitly says to prefer `invert()` wherever
  failure is possible, which is the honest API design — a function that cannot fail should not be
  the only one offered.

---

## Q13 — Why do normals need the inverse transpose when directions don't?

### Short Answer

A normal is a covector, not a vector. Under non-uniform scale it transforms by the inverse transpose
of the upper-left 3×3, not by the matrix itself — scaling `x` by 2 *halves* the `x` component of a
surface normal rather than doubling it. Using the direction transform is the classic bug that leaves
normals no longer perpendicular to their surfaces.

### Deep Answer

The clean derivation is one line. A normal `n` is defined by being perpendicular to every tangent
`t` of the surface: `nᵀt = 0`. Under a transform `M`, tangents transform like directions:
`t' = Mt`. We need `n'` such that `n'ᵀt' = 0` for all such `t'`:

```
n'ᵀ(Mt) = 0   for all t with nᵀt = 0
```

Try `n' = (M⁻¹)ᵀn`:

```
n'ᵀ(Mt) = ((M⁻¹)ᵀn)ᵀ(Mt) = nᵀM⁻¹Mt = nᵀt = 0   ✓
```

So the inverse transpose is exactly what preserves perpendicularity. The intuition is worth having
too: stretch a surface along `x`, and it becomes *flatter* relative to `x`, so its normal tilts
*away* from `x`. The normal moves opposite to the way a tangent does — which is what "covector"
means.

**When does it matter?** Only for non-uniform transforms. For a pure rotation `M⁻¹ = Mᵀ`, so
`(M⁻¹)ᵀ = M` and the two agree. For a uniform scale they differ only by a scalar factor, which
vanishes on renormalisation. That is exactly why the bug survives: it is invisible until someone
non-uniformly scales an object, and then the lighting is subtly wrong in a way that is easy to blame
on the shader.

**Why directions do not need it.** A direction — a ray direction, a tangent, a velocity — is a
genuine vector: it transforms as `Mv` with `w = 0` so translation does not apply. The type system
carries the distinction explicitly here: `Vec4` has a `w` component, and points get `w = 1` while
directions get `w = 0`. The header makes the design intent clear — keeping that distinction in the
type is what makes `transformPoint` and `transformVector` separate, explicit operations rather than
one function with a silent convention.

**API detail worth defending.** `transformNormalWithInverse` takes the **already-inverted** matrix
rather than inverting internally, so a caller transforming ten thousand normals pays for one inverse
rather than ten thousand. The name says so, which prevents the obvious misuse. And the transpose is
never materialised — the multiply is written transposed inline.

I verified that expansion against the storage convention rather than trusting the comment: with
`i.m[c][r]` meaning column `c`, row `r`, the element of `iᵀ` at row `r`, column `c` is `i.m[r][c]`,
so `(iᵀn)_r = Σ_c i.m[r][c]·n_c` — which is exactly what the three expressions compute.

**The test is the good part.** It does not just check the correct answer; it constructs a
tangent/normal pair, applies a non-uniform scale, then asserts that the inverse-transpose result
stays perpendicular to the transformed tangent **and** that the naive `transformVector` result does
not. So the test fails if someone "simplifies" the function — which is the only kind of test that
actually protects a subtle correctness property.

### Code Connection

- Declaration with the full covector explanation and the "classic bug" warning:
  `include/geometry/mat4.hpp:116-125`.
- Implementation, transpose never materialised: `src/geometry/mat4.cpp:188-195`.
- The contrast: `transformPoint` (`w = 1`, with perspective divide) at `src/geometry/mat4.cpp:172-181`;
  `transformVector` (`w = 0`) at `:183-186`; and the explicit note that length is *not* preserved
  under scaling and that this is deliberate — `include/geometry/mat4.hpp:111-113`.
- Why the distinction lives in the type: `include/geometry/vec4.hpp:10-14`.
- The test that pins both halves: `tests/test_mat4.cpp:158-182`.
- Perspective-divide handling, including the affine fast path:
  `src/geometry/mat4.cpp:174-180`, tested at `tests/test_mat4.cpp:183-198`.

### Tradeoffs

| Approach | Consequence |
|---|---|
| **Inverse transpose (chosen)** | Correct for all invertible transforms; needs the inverse |
| Transform as a direction | Correct only for rotations and uniform scales; silently wrong otherwise |
| Rotation-only fast path | Cheap and correct when `M` is orthogonal, but needs a check or a promise |
| Recompute the cofactor matrix instead of the inverse | Avoids the divide by the determinant; the scale factor washes out on renormalisation — a legitimate optimisation, not implemented here |
| Invert inside the function | Convenient; catastrophic for a loop over many normals |

The cofactor variant is worth naming if asked for an optimisation: since normals are renormalised
anyway, any positive scalar multiple is acceptable, so the adjugate works and you skip the division
by the determinant entirely.

### Follow-up Questions

- *"When can you skip it?"* Pure rotations, and rigid transforms generally: `(M⁻¹)ᵀ = M` for
  orthogonal `M`. Uniform scale differs only by a scalar that renormalisation removes.
- *"Does it preserve unit length?"* No — the result must be renormalised. Neither does
  `transformVector`, and for ray directions that is deliberate (Q14).
- *"What about the translation column?"* Irrelevant — normals use only the upper-left 3×3, which is
  why the implementation reads nine elements and ignores the rest.
- *"Where would you use this in this project?"* Shading normals in the Vulkan renderer (Phase 7),
  and any object-space query that needs a surface normal back in world space. Neither exists yet:
  `UNKNOWN — not yet implemented`.

---

## Q14 — Why aren't ray directions normalized?

### Short Answer

Two reasons. Normalising costs a `sqrt` per ray and the slab test works with any direction scale, so
it buys nothing. More importantly, when a ray is transformed into an object's local space the
transform may scale it — and renormalising there would silently change the meaning of `t`, so hit
distances found in local space would no longer match world space.

### Deep Answer

The second reason is the real one, and it is the kind of thing that only bites once you have an
object hierarchy.

The standard pattern for instanced geometry is: keep the BVH in the object's local space, and for
each query transform the *ray* into that space with the object's inverse transform, rather than
transforming the geometry. Suppose the object has a 2× scale. World-space direction `d` becomes
local-space `2d`. A hit at world parameter `t` is at world point `o + t·d`; the same physical point
in local space is at `o' + t·(2d)` — **the same `t`**, because the direction scaled by exactly the
factor the space did.

Now renormalise the local direction. It becomes `d̂`, and the same physical point is now at
parameter `2t`. You have two spaces reporting different `t` for the same intersection. Every
comparison against the ray's current closest hit is now wrong, `tMax` no longer means what traversal
thinks it means, and the bug shows up as instanced objects occluding each other incorrectly — miles
from the `normalize` call that caused it.

Leaving the direction alone makes `t` invariant under the transform. The price is stated honestly in
the header: **`t` is measured in units of `|direction|`, not in world distance.** A caller that needs
a true metric distance must normalise up front and accept the `sqrt` — a deliberate decision pushed
to the boundary rather than silently made in the middle.

The first reason is the cheaper one: the slab test is scale-invariant, since scaling `d` by `k`
scales `invDir` by `1/k` and every `t` by `1/k` uniformly, so all the comparisons come out the same.
Normalising would be a `sqrt` per ray for no benefit to the test that dominates traversal.

The related decision in the same type: `[tMin, tMax]` is carried *inside* the `Ray` rather than
passed alongside it, so traversal can shrink `tMax` as closer hits are found. That is the single
most effective pruning mechanism in a BVH — once a hit at `t` is known, every subtree whose entry
distance exceeds `t` can be skipped outright. It only works if `t` means the same thing everywhere,
which is exactly what not renormalising guarantees.

There is a test that asserts the invariant directly:
`transformPoint(m, r.at(t)) == transformRay(m, r).at(t)` — transforming the hit point and evaluating
the transformed ray give the same answer. That equation is false the moment you renormalise.

### Code Connection

- The decision and both reasons: `include/geometry/ray.hpp:9-19` — "Renormalising there would
  silently change the meaning of t, so hit distances found in local space would no longer match
  world space", and the consequence for callers at `:17-19`.
- `tMin`/`tMax` in the ray, and the pruning argument: `include/geometry/ray.hpp:21-24`.
- `transformRay` — origin as a point, direction as a vector, range copied unchanged, no
  renormalisation: `include/geometry/ray.hpp:59-61`, with the reasoning restated at `:56-58`.
- `transformVector` explicitly documents that length is not preserved and that this is deliberate:
  `include/geometry/mat4.hpp:111-113`.
- The invariant test: `tests/test_ray.cpp:79-91`
  (`TransformKeepsHitParameterConsistentUnderScaling`).
- Range preservation: `tests/test_ray.cpp:71-77`. Round-trip through an inverse composite transform:
  `:93-106`. Non-unit direction accepted: `:35-40`.
- Scale-invariance of the slab test follows from `include/geometry/aabb.hpp:177-178` — `tNear` and
  `tFar` both scale by `1/k`.

### Tradeoffs

| Choice | Gain | Cost |
|---|---|---|
| **Non-normalised (chosen)** | no `sqrt`; `t` invariant under transform; local and world `t` comparable | `t` is not a metric distance; callers who need one must normalise |
| Always normalised | `t` is world distance everywhere | `sqrt` per ray; **breaks** `t` comparability across scaled spaces |
| Store both `d` and `d̂` | both available | 12 more bytes per ray and a consistency invariant to maintain |
| Normalise only at the API boundary | callers see distances | still breaks object-space traversal if the transform scales |

### Follow-up Questions

- *"What if I want the actual distance to the hit?"* Multiply `t` by `|direction|`, or normalise the
  ray before you issue it. The header says which.
- *"Does this affect shading?"* Yes — a shader wanting a unit direction must normalise it, and the
  surface normal must be transformed with the inverse transpose (Q13). Neither is in Phase 1.
- *"Why keep `tMin`/`tMax` in the ray rather than as parameters?"* So traversal can shrink `tMax` in
  place as closer hits are found; that is the main pruning mechanism.
- *"Does the slab test really not care about scale?"* No — scaling `d` by `k` scales every `t` by
  `1/k`, and the comparisons are all between `t` values, so the outcome is identical. Only the
  reported `t` changes, consistently.
- *"Where is this actually used?"* Nowhere yet — object-space BVH traversal is Phase 2.
  `UNKNOWN — not yet implemented`. The test at `tests/test_ray.cpp:93-106` simulates the
  world→object→world path it is designed for.

---

## Q15 — Why is the mesh indexed rather than an array of triangles?

### Short Answer

A closed mesh shares each vertex among about six triangles, so indexed storage is roughly half the
bytes of an unpacked `vector<Triangle>` — and during a BVH build, where you stream over every vertex
repeatedly, bytes are the constraint. It is also the layout `vkCmdDrawIndexed` wants, so the render
path uploads both buffers with no repacking. The cost is one level of indirection per vertex fetch.

### Deep Answer

**The arithmetic, which is worth doing on a whiteboard.** Let `m` be triangles and `n` vertices. For
a closed manifold, Euler's relation gives roughly `n ≈ m/2` — each vertex is shared by about six
triangles, and each triangle has three vertices, so vertices-per-triangle is `3/6 = 0.5`.

- Unpacked `vector<Triangle>`: 3 × `Vec3` = **36 bytes per triangle**.
- Indexed: positions `12n ≈ 6m` bytes, plus indices `3 × 4 = 12m` bytes → **≈ 18 bytes per
  triangle**.

So about a **2× saving**. (The source comment says "roughly 12 bytes + 12 bytes of indices per
triangle versus 36" — 24 vs 36, a 1.5× saving. Its own "~6 triangles per vertex" premise actually
gives 6 + 12 = 18. The conclusion holds; the arithmetic in the comment is conservative. I checked
rather than quoting it.)

**Why the bytes matter — the cache argument.** BVH construction is not a single pass. A top-down
build touches the primitives at the root, then again at each child, then again at each grandchild —
`O(log n)` passes over progressively smaller subsets, but the upper levels stream the whole vertex
set. If that data fits in L2/L3, the build is bound by arithmetic; if it does not, every pass goes
to DRAM. Halving the footprint moves that threshold by a factor of two in mesh size. The source
comment puts it as "on a million-triangle mesh that difference decides whether the vertex data fits
in cache during a build" — which is the right *shape* of argument, though whether a particular mesh
crosses a particular machine's threshold is exactly the kind of claim that needs measurement.
`UNVERIFIED`: no benchmark exists.

**The GPU reason.** Vulkan's `vkCmdDrawIndexed` consumes precisely this pair of buffers. Storing
unpacked triangles would mean either repacking before upload or wasting 2× the VRAM and vertex-fetch
bandwidth. Design intent only for now — there is no Vulkan code in the tree.

**The cost, stated honestly in the source.** One level of indirection per vertex fetch:
`positions_[indices_[base + k]]` is a dependent load — you cannot start the second until the first
returns. That is fine during a build, which is bandwidth-bound and streams predictably, but it is
bad during *traversal*, which is latency-bound and random-access. The header says so and names the
fix: "Phase 2 can revisit by storing unpacked triangles in leaf order."

That is the genuinely interesting part of this answer. **The right layout for the build and the
right layout for the query are different**, and the usual production resolution is to keep both: an
indexed mesh as the authoritative representation, and a flattened, leaf-ordered, unpacked triangle
array built alongside the BVH for traversal. You pay 36 bytes per triangle again, but in a layout
where a leaf's triangles are contiguous, which turns three dependent loads into one sequential
stream. The tradeoff is already flagged in the source rather than discovered later.

**Two related decisions in the same class.** Indices are `uint32_t`, not `size_t`: half the width,
and 4 billion vertices is beyond anything this project will load. And `Mesh::triangle` returns by
value, with the reasoning given — 36 bytes is cheaper to copy than to alias, and returning a value
keeps the `Mesh` immutable to callers.

**What makes the unchecked fetch safe.** The constructor validates the whole index buffer once —
multiple of 3, every index in range — and throws `std::invalid_argument` otherwise. That `O(n)` pass
at the boundary buys an unchecked inner loop for the lifetime of the mesh, and exceptions rather than
assertions are used because this is where untrusted file data enters the system, and assertions
vanish in release builds — exactly when malformed data shows up.

### Code Connection

- Storage and the full rationale, including the Vulkan reason and the admitted indirection cost:
  `include/geometry/mesh.hpp:15-26`.
- The invariant that licenses unchecked indexing: `include/geometry/mesh.hpp:28-30`; enforcement at
  `src/geometry/mesh.cpp:10-23`; "O(n) here buys us an unchecked inner loop for the lifetime of the
  mesh" at `:14-15`; exceptions-not-assertions rationale at `include/geometry/mesh.hpp:38-40`.
- Materialising a triangle — three indirect loads: `include/geometry/mesh.hpp:61-66`; the
  return-by-value justification at `:59-60`.
- Buffers moved, not copied: `include/geometry/mesh.hpp:35-36`, `src/geometry/mesh.cpp:9`.
- The unpacked counterpart and why both exist: `include/geometry/triangle.hpp:9-16` — "indexed
  storage is the compact form for holding a mesh, while the unpacked form is what an intersection
  routine wants, since chasing indices during traversal costs an extra dependent memory access per
  test."
- Cached bounds, and the honest note that they cover the *vertex array* rather than the referenced
  triangles: `include/geometry/mesh.hpp:68-78`; orphan-vertex test at `tests/test_mesh.cpp:101`.
- Validation tests: `tests/test_mesh.cpp:84`, `:88`, `:93`, `:97`.

### Tradeoffs

| Layout | Bytes/triangle | Build | Traversal |
|---|---:|---|---|
| **Indexed (chosen)** | ≈18 | compact; good streaming | one dependent load per vertex |
| `vector<Triangle>` | 36 | 2× the bandwidth | no indirection; random access is one load |
| Indexed + leaf-ordered unpacked copy | ≈54 total | compact during build | best traversal; the production answer |
| SoA (separate x/y/z arrays) | ≈18 | SIMD-friendly | 3 streams instead of 1 for scalar code |
| Quantised positions (16-bit) | ≈12 | smallest | dequantise per fetch; precision loss |

### Follow-up Questions

- *"Why `uint32_t` and not `size_t`?"* Half the width, and the vertex counts this project targets are
  nowhere near 2³². It halves the index buffer, which is two thirds of the indexed footprint.
- *"Won't the indirection hurt traversal?"* Yes, and the source says so. The fix is a leaf-ordered
  unpacked triangle array built alongside the BVH — build-optimal and query-optimal layouts are
  different, and you are allowed to have both.
- *"How would you measure whether it matters?"* Build both layouts, hold the mesh and ray set fixed,
  and compare build time, query time, and cache-miss counters. Phase 9 of the plan defines the
  methodology. Nothing is measured yet.
- *"Why validate eagerly instead of using `.at()`?"* `.at()` is a bounds check per access forever;
  validation is one pass ever. The invariant then holds for the object's lifetime because nothing
  public can mutate the index buffer.
- *"What about vertex normals and UVs?"* Not present — Phase 1 stores positions only. Adding them
  raises an AoS-vs-SoA question, since the BVH build wants positions alone and the renderer wants
  all attributes interleaved. `UNKNOWN — not yet implemented`.

---

## Q16 — Is this really C++20?

### Short Answer

Not fully, and the build says so out loud. The installed Apple clang 11.0.3 has no C++20 standard
library, and CMake silently falls back to the draft `-std=c++2a` while still reporting "20" — so I
added a configure-time probe that compiles `<concepts>` and `<span>` and warns when they are
missing. The geometry core does not use any C++20 library feature, so it builds and passes, but the
project cannot honestly claim C++20 yet.

### Deep Answer

`CMAKE_CXX_STANDARD 20` with `CMAKE_CXX_STANDARD_REQUIRED ON` reads like a guarantee and is not one.
For a compiler that only knows the pre-release draft flag, CMake emits `-std=c++2a` and reports
"20" anyway. You get a build that looks conformant and is missing most of the C++20 standard
library — `<concepts>`, `<span>`, `<ranges>`, `<numbers>` are all absent on this toolchain.

Rather than let that sit silently, the build probes for it: `check_cxx_source_compiles` on a
translation unit that includes `<concepts>` and `<span>` and instantiates a `std::floating_point`
constrained template. On failure it emits a `message(WARNING)` naming the compiler and pointing at
the fix, and the configuration summary prints
`C++ standard ....... 20 (DRAFT -- no C++20 library)` instead of a bare "20".

**Why it does not block Phase 1.** Nothing in `include/geometry/` or `src/geometry/` uses a C++20
library facility. The features actually relied on — `constexpr` member functions, default member
initialisers, `inline constexpr` namespace-scope variables, hidden friend operators — are C++17 or
earlier. So the accurate statement is: *this is C++17-compatible code compiled under a draft C++20
flag.*

**Why it will block later phases.** Homebrew refuses to compile anything on this configuration
("Your Command Line Tools are too outdated"), which already blocked `brew install llvm` and will
block GLFW, the Vulkan SDK and ImGui the same way. The fix needs a GUI installer and `sudo`, so it
has to be done by hand.

**The transferable point** is not the version number. It is that a build system reporting "C++20"
was *wrong*, and the response was to add a check that makes the discrepancy visible rather than
inheriting it. That is the same instinct as the two bugs in Q10 — assume the thing that looks fine
might be silently lying, and add the check.

### Code Connection

- Standard requested: `CMakeLists.txt:12-14`, with `CMAKE_CXX_EXTENSIONS OFF` so it is `-std=c++20`
  rather than `-std=gnu++20`.
- The reason a probe is needed, in the build file: `CMakeLists.txt:27-31`.
- The probe: `CMakeLists.txt:32-40`. The warning: `:42-48`. The honest summary line: `:132-136`.
- Platform and toolchain analysis: `CLAUDE.md`, "Toolchain status — action needed".
- Also configured deliberately: Release default because "an unset `CMAKE_BUILD_TYPE` silently gives
  no optimisation at all" (`CMakeLists.txt:19-25`); the strict warning set with per-flag
  justifications (`:61-73`); the `-ffast-math` ban (`:81-84`); sanitizers behind an option
  (`:86-91`); and the layering rule stated in the build (`:96-98`).

### Tradeoffs

| Response to the toolchain gap | Consequence |
|---|---|
| **Probe and warn (chosen)** | Honest; build proceeds; the gap is visible at every configure |
| Say nothing | The project claims C++20 falsely; the first `#include <span>` fails mysteriously |
| Hard-fail the configure | Would block Phase 1 for no technical reason — the core does not need C++20 |
| Downgrade to `CMAKE_CXX_STANDARD 17` | Honest, but discards the intent and would need reverting |
| Install a newer toolchain first | The right fix; needs sudo and a GUI installer, so it cannot be scripted from here |

### Follow-up Questions

- *"What would you use C++20 for here?"* `std::span` for non-owning views over index and position
  buffers; `concepts` to constrain a `Scalar` template if precision ever became configurable;
  `<numbers>` for `pi` instead of the literal at `include/geometry/scalar.hpp:30`. None is load-bearing.
- *"Why not just use C++17?"* The plan specifies C++20 and later phases may want `std::span` at
  buffer boundaries. Recording the gap costs nothing and keeps the intent.
- *"Does the draft flag change code generation?"* Not for anything this code uses — the missing
  pieces are library headers, not language semantics the core depends on.
- *"How would an interviewer read this?"* Favourably, if you frame it as "the build was reporting
  something untrue and I made it stop" rather than "my compiler is old".

---

## Q17 — How do you know any of this is correct?

### Short Answer

105 unit tests across seven files, and the interesting ones are not known-answer checks — they are
property tests and degeneracy tests named after the failure they prevent. The suite is clean under
`-Werror` with a strict warning set and clean under UBSan, which is how the divide-by-zero bug was
found.

### Deep Answer

The suite has four distinguishable layers, and being able to name them is more useful in an
interview than the count.

**1. Known-answer tests.** A 1×2×3 box has surface area 22 and volume 6; a ray from `z = -5` hits
the unit box at `t = 4`. Fast to write, catch gross errors, prove nothing subtle.

**2. Property tests** — assertions about mathematical relationships rather than constants. The cross
product is orthogonal to both inputs; rotation preserves length; rotation about an axis leaves that
axis fixed; `M·M⁻¹ = I`; a world→object→world round-trip recovers the original ray; a matrix-vector
product matches its manual expansion. These survive refactoring in a way that magic constants do
not.

**3. Degeneracy and hazard tests** — the distinctive part. Nearly every degenerate input in the
system has a named test, and several are named after the bug they prevent:
`RayLyingExactlyOnSlabBoundaryIsHandled`, `EmptyBoxIsNeverHit`, `FlatBoxIsStillHittable`,
`OffsetOnDegenerateAxisDoesNotDivideByZero`, `CollinearPointsGiveDegeneratePlaneNotNaN`,
`RotationAboutDegenerateAxisIsIdentity`. This layer is what found the slab-swap bug, before any BVH
existed to exhibit the symptom — which is the argument for testing representation edge cases at the
primitive level rather than waiting for the system to behave oddly.

**4. Convention-pinning tests** — `ColumnMajorStorageLayout`, `MultiplicationAppliesRightmostFirst`,
`CrossProductIsRightHanded`, `NormalFollowsCounterClockwiseWinding`,
`DirectionIsNotRequiredToBeNormalized`. A convention that exists only in a comment will drift; one
with a test will not.

**Two tests worth reading aloud.** `NormalTransformUnderNonUniformScale` asserts both that the
inverse-transpose result stays perpendicular to the transformed tangent *and* that the naive
`transformVector` answer does not — so the test fails if someone "simplifies" the function. And
`TransformKeepsHitParameterConsistentUnderScaling` asserts
`transformPoint(m, r.at(t)) == transformRay(m, r).at(t)`, which is exactly the property that would
break if `transformRay` renormalised the direction. Both encode *why* the code is the way it is.

**Comparison discipline.** Exact `EXPECT_EQ` on `Vec3` only where values are exactly representable —
sentinels, integral coordinates, unchanged components — and `nearlyEqual`/`EXPECT_NEAR` wherever
arithmetic has happened. That mirrors the policy documented on `Vec3::operator==` itself, which says
in so many words never to use it on results of floating-point arithmetic.

**Beyond tests.** A strict warning set with per-flag justifications in the build file
(`-Wconversion` because silent narrowing changes results; `-Wdouble-promotion` because an accidental
promotion in a hot loop undoes the float decision; `-Wshadow` because a shadowed variable in
geometry code is nearly always a bug), `-Werror` available behind an option, and ASan+UBSan behind
another. The commit reports the tree clean under both.

**Gaps I would name before being asked.** No property-based or fuzz testing — a randomised
ray-vs-brute-force cross-check only becomes possible once traversal exists. No test for the
`gamma(3)` widening: deleting that line still passes all 105 tests. No test for `AABB::centroid()`
on an empty box, which returns NaN. And **no benchmarks at all** — `benchmarks/results/` holds only
a format and methodology specification, so every performance statement in this project is an
argument from layout or complexity, not a measurement.

**One disclosure.** I did not re-run the suite while writing this document: there is no `build/`
directory in the tree and a first configure needs network access to fetch GoogleTest. The count of
105 is verified statically from the source; the pass/fail status is quoted from the `841777e` commit
message.

### Code Connection

- Test wiring, pinned GoogleTest v1.15.2 and why it is pinned: `tests/CMakeLists.txt:1-22`;
  per-`TEST()` registration for named failures and parallel `ctest`: `:46-51`.
- Counts by file: `test_aabb.cpp` 28, `test_mat4.cpp` 18, `test_vec3.cpp` 17, `test_mesh.cpp` 15,
  `test_triangle.cpp` 12, `test_ray.cpp` 10, `test_plane.cpp` 5 — 105 total.
- Property tests: `tests/test_vec3.cpp:84`; `tests/test_mat4.cpp:80`, `:92`, `:129`, `:200`;
  `tests/test_ray.cpp:93`.
- Degeneracy tests: `tests/test_aabb.cpp:142`, `:207`, `:235`, `:244`; `tests/test_plane.cpp:30`,
  `:36`; `tests/test_mat4.cpp:86`, `:146`; `tests/test_triangle.cpp:82`, `:91`, `:100`;
  `tests/test_mesh.cpp:84`, `:88`, `:93`, `:114`.
- Convention tests: `tests/test_mat4.cpp:28`, `:99`; `tests/test_vec3.cpp:67`;
  `tests/test_triangle.cpp:20`; `tests/test_ray.cpp:35`.
- The two exemplary tests: `tests/test_mat4.cpp:158-182`; `tests/test_ray.cpp:79-91`.
- Comparison policy: `include/geometry/vec3.hpp:46-48`; `nearlyEqual` at
  `include/geometry/scalar.hpp:67-74`.
- Warning set with justifications: `CMakeLists.txt:61-73`. Sanitizers: `:86-91`.
- Benchmark rules: `benchmarks/results/README.md:3-5` and the methodology at `:48-62`.

### Tradeoffs

| Approach | What it catches | What it misses |
|---|---|---|
| Known-answer tests | gross errors | anything subtle; brittle under refactoring |
| Property tests | relationship violations | wrong constants that still satisfy the relationship |
| Degeneracy tests | representation edge cases — found the slab bug here | value-range issues |
| Fuzz / property-based | large input spaces | needs an oracle; empty boxes have measure zero in random distributions |
| Sanitizers | UB — found the `1/0` bug here | only on paths actually executed |
| Differential testing vs. a reference | almost everything | needs a reference; the project rules forbid depending on one for core algorithms |

### Follow-up Questions

- *"What would you add next?"* A randomised ray-vs-brute-force differential test as soon as traversal
  exists — the strongest possible check on a BVH, since brute force is the oracle. Plus a test for
  the `gamma(3)` widening and one for `centroid()` on an empty box.
- *"How do you test performance?"* Not yet at all. The methodology is written down —
  Release-only, median of ≥5 runs, discard warmup, fixed seeded ray sets, one variable at a time,
  record the commit — but no run exists.
- *"Is 105 tests a lot?"* The count is not the point; coverage of *representation* edge cases is.
  Two real bugs were found by tests in layer 3, neither of which a coverage metric would have
  demanded.
- *"What is your coverage percentage?"* Not measured. I would rather name the specific untested
  properties than quote a line-coverage number that would be near 100% and still miss both gaps.

---

## Not yet answerable

Listed so this file cannot be mistaken for something it is not. Each becomes answerable when the
corresponding phase lands.

| Question | Status |
|---|---|
| Why a BVH rather than a KD-tree, octree, or grid? | `UNKNOWN — not yet implemented` (Phase 2). Arguable from theory, but nothing in this repository demonstrates it. |
| How is the BVH node represented, and why? | `UNKNOWN — not yet implemented` (Phase 2) |
| What is the complexity of BVH construction? | `UNKNOWN — not yet implemented` (Phase 2) |
| What is the complexity of traversal? | `UNKNOWN — not yet implemented` (Phase 2) |
| Why does traversal reduce the number of triangle tests, and by how much? | `UNKNOWN — not yet implemented` (Phases 2, 5) |
| Median vs centroid-median vs SAH splitting | `UNKNOWN — not yet implemented` (Phase 3). Only the SAH *cost term* (`AABB::surfaceArea`) and the *binning helper* (`AABB::offset`) exist — Q7 covers those. |
| How is SAH binning implemented, and how many bins? | `UNKNOWN — not yet implemented` (Phase 3) |
| Build time vs query time tradeoff | `UNKNOWN — not yet implemented` (Phases 3, 9) |
| Why might a smaller leaf size be worse? | `UNKNOWN — not yet implemented` (Phase 3) |
| Ray/triangle intersection — Möller–Trumbore or otherwise | **Deliberately not in Phase 1.** No such code exists in this tree. |
| Watertight ray-triangle intersection at shared edges | `UNKNOWN — not yet implemented` |
| Why Vulkan compute? What stays on the CPU? | `UNKNOWN — not yet implemented` (Phase 8) |
| What causes CPU/GPU synchronization stalls? | `UNKNOWN — not yet implemented` (Phase 8) |
| Vulkan instance/device/queues/descriptors/pipelines | `UNKNOWN — not yet implemented` (Phase 7) |
| How would you parallelize BVH construction? | `UNKNOWN — not yet implemented` (Phase 8/9) |
| How would you optimize the memory layout? | Partially answerable now — Q1 (node width), Q15 (mesh layout, and the build-vs-query layout split). The BVH node layout itself is `UNKNOWN — not yet implemented`. |
| How would this scale to millions of triangles? | Arguable from complexity and layout; **no measurement exists**. Any specific number would be fabricated. |
| Any benchmark result whatsoever | `UNVERIFIED` — `benchmarks/results/` contains only `README.md`, a format and methodology specification. |
| CPU vs GPU speedup on this machine | `UNKNOWN — not yet implemented`. Note the hardware context in `CLAUDE.md`: Intel Core i5-8257U with integrated Iris Plus, Vulkan only via MoltenVK, so a modest or even negative GPU result is the expected honest outcome. |
| OBJ loading | `UNKNOWN — not yet implemented` |
| The interactive explorer, parameter controls, visualization | `UNKNOWN — not yet implemented` (Phases 4–5, 10) |
