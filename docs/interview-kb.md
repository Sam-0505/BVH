# Interview Knowledge Base

Questions this project can currently answer *from its own source*, with the answer you would give,
the detail behind it, and the exact code that proves it.

**Scope.** Phase 1 only, at `935c354` "Phase 1 review fixes: UB, preconditions, scale-invariance".
Questions about BVH construction, traversal, SAH implementation, Vulkan, GPU compute and measured
performance are listed at the end as [not yet answerable](#not-yet-answerable), so you do not walk
into an interview thinking you can defend them.

**Rule this file follows.** There are no benchmarks — `benchmarks/results/` contains only a format
specification. No answer below quotes a number as measured. Where a code comment makes a performance
argument, the answer presents it as an argument and says what would have to be measured to close it.
"I argued it from the memory layout but haven't measured it" is a strong answer; inventing a speedup
is a fatal one.

**Verified status at `935c354`**, so you can state it accurately: Release **122/122** pass, Debug
**128/128** pass (the difference is the death tests, which only exist when asserts are live),
`-Werror` clean under the full strict warning set, UBSan clean in both configurations. **ASan cannot
run on this machine at all** — a hello-world built with `-fsanitize=address` exits 139, verified in
isolation — so there is no memory-safety sanitizer coverage. Say that plainly if asked; see [Q19](#q19--how-do-you-know-any-of-this-is-correct).

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
13. [Why is a matrix-norm-relative singularity tolerance wrong for a homogeneous transform?](#q13--why-is-a-matrix-norm-relative-singularity-tolerance-wrong-for-a-homogeneous-transform)
14. [Why do normals need the inverse transpose when directions don't?](#q14--why-do-normals-need-the-inverse-transpose-when-directions-dont)
15. [Why aren't ray directions normalized?](#q15--why-arent-ray-directions-normalized)

**Data layout and API design**

16. [Why is the mesh indexed rather than an array of triangles?](#q16--why-is-the-mesh-indexed-rather-than-an-array-of-triangles)
17. [When do you throw and when do you assert?](#q17--when-do-you-throw-and-when-do-you-assert)

**Process and engineering**

18. [Is this really C++20?](#q18--is-this-really-c20)
19. [How do you know any of this is correct?](#q19--how-do-you-know-any-of-this-is-correct)

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
entirely on whether the workload is limited by arithmetic throughput or memory traffic, and BVH
traversal is firmly the second: a pointer-chasing walk over a node array where almost every byte
fetched is bounds data, and the arithmetic per node is a handful of multiplies and compares that a
modern core issues faster than the cache can feed it.

An `AABB` is exactly 2 × `Vec3` = 24 bytes at float, 48 at double. On a 64-byte cache line that is
two-and-a-bit nodes versus one-and-a-bit. The choice is really about node density, and node density
is the lever on traversal time.

The honest half of the answer is the second one: 7 digits is not much, and the code has to earn it
back. That is why the slab test widens the exit distance by a *relative* error bound instead of
trusting `t` to be exact, and why every comparison in it is NaN-safe. Those mechanisms are the price
of the float decision, and I can point at both.

I kept the door open cheaply: `Scalar` is a single typedef, so a double-precision build is a one-line
change, and nothing is templated on precision — which would have put template noise at every call
site to buy a configurability nobody has asked for.

### Code Connection

- `using Scalar = float;` — `include/geometry/scalar.hpp:20`, memory-bound rationale at `:12-16`,
  "typedef not template" at `:8-10`.
- The 24-byte `AABB`: `include/geometry/aabb.hpp:44-45` (two `Vec3`, each three floats at
  `include/geometry/vec3.hpp:24-26`).
- The robustness that pays for it: `kRayBoxWidening` at `include/geometry/aabb.hpp:22`, applied at
  `:237`; NaN-tolerant narrowing at `:241-242`.
- `-Wdouble-promotion` is enabled specifically so an accidental `float`→`double` promotion in a hot
  loop cannot silently undo the decision — `CMakeLists.txt:71`.

### Tradeoffs

| Choice | Gain | Cost |
|---|---|---|
| `float` (chosen) | 24-byte AABB, double node density per cache line | ~7 digits; intersection code must be written defensively |
| `double` | ~16 digits; naive intersection code is usually fine | 48-byte AABB; half the nodes per cache line |
| Templated on precision | Both available | Template noise at every call site; two codepaths to test |
| Mixed (float storage, double arithmetic) | Compact storage, accurate math | Conversion per access; `-Wdouble-promotion` exists to catch this happening by accident |

**Be explicit about what is unmeasured.** The claim that memory traffic dominates ALU work *for this
code on this machine* is `UNVERIFIED`: no float-vs-double build has been benchmarked, and the
traversal loop the argument is about does not exist yet. The reasoning is standard and matches
production renderers, but I would want a measurement before putting a number on it.

### Follow-up Questions

- *"How would you verify the memory-bound claim?"* Build the same scene at both precisions, hold the
  ray set and seed fixed, compare traversal time and L1/L2 miss rate. A hardware counter profile is
  what distinguishes "memory-bound" from "I assumed it was".
- *"Where would float precision actually bite?"* Large scenes far from the origin: absolute spacing
  between representable floats grows with magnitude, so a scene at coordinate 10⁵ has roughly 10⁻²
  resolution. That is where you move to doubles, or re-origin the scene.
- *"Would you use half precision for node bounds?"* Only with directed rounding — min down, max up —
  so bounds stay conservative. Without that, quantisation produces false misses.
- *"What's the AVX angle?"* Float gives 8 lanes per 256-bit register versus 4 for double, so packet
  or 8-wide-BVH traversal doubles its throughput too. Not implemented.

---

## Q2 — Why does `Vec3::operator[]` use a conditional chain instead of `(&x)[i]`?

### Short Answer

`(&x)[i]` is undefined behaviour — pointer arithmetic is only defined *within* a single object, and
three separate members are not an array however they are laid out. The conditional chain is
well-defined, still returns a real reference because a ternary over lvalues is an lvalue, and
compiles to the same thing.

### Deep Answer

The `(&x)[i]` trick is everywhere in graphics code and it works on every compiler anyone has tried.
That is what makes it worth talking about: a case where "it works" and "it is correct" come apart.

The standard's rule is that `p + n` is only defined when `p` points into an array object (or one past
its end), and a scalar counts as an array of length 1 for this purpose. `&x` points at a single
`Scalar`, so `(&x)[1]` is already past the end of the only object it may walk. The layout of `Vec3`
is almost certainly three contiguous floats — but layout is not the question; the abstract machine's
object model is. A compiler entitled to assume the access stays in bounds may reorder it against a
store to `y`, or apply an alias-analysis result that assumes `&x + 1` cannot be `&y`.

The alternative costs nothing:

```cpp
constexpr Scalar& operator[](int i) {
    assert(i >= 0 && i < 3);
    return i == 0 ? x : (i == 1 ? y : z);
}
```

Two subtleties make it work. A conditional expression whose second and third operands are lvalues of
the same type is itself an lvalue, so the function genuinely returns a reference and `v[i] = 3.0f`
works. And when `i` is a compile-time constant the chain folds away; for a runtime index the compiler
emits a cmov or a small branch.

Why `Vec3` needs indexed access at all is worth saying, or the question looks academic: the slab test
loops over axes and reads `box.min[axis]` with a runtime `axis`, and BVH split-axis selection will do
the same. Named members are what you want for readability (`v.x` beats `v[0]`), indexed access is
what algorithms need, and this is how you get both.

### Code Connection

- Implementation and both overloads: `include/geometry/vec3.hpp:32-39`; reasoning, including the
  lvalue-ternary point and the codegen expectation, at `:11-22`.
- Same pattern for `Vec4`: `include/geometry/vec4.hpp:25-32`.
- Why indexed access is required: `intersectRay` reads `box.min[axis]` / `box.max[axis]` on a runtime
  axis — `include/geometry/aabb.hpp:217-218`.
- `maxAxis` returns exactly such a runtime index — `include/geometry/vec3.hpp:141-144`.
- Tested against named members at `tests/test_vec3.cpp:20`; out-of-range index has a death test at
  `tests/test_preconditions.cpp:65-68`.

### Tradeoffs

| Approach | Defined? | Notes |
|---|---|---|
| `(&x)[i]` | **No** | Works in practice; UB in principle; UBSan and strict aliasing can both object |
| Conditional chain (chosen) | Yes | Same codegen expected; slightly more source |
| `Scalar v[3]` storage + named accessors | Yes | Loses `v.x` as a plain member |
| Anonymous union of struct + array | **No** (in C++) | Type-punning through a union is UB in C++, unlike C — the most popular "fix" is also not a fix |
| `std::array<Scalar,3>` + `x()`/`y()`/`z()` | Yes | Clean, but every call site gains parentheses |

The codegen-equivalence claim is `UNVERIFIED` — no disassembly is recorded. If pressed: "I expect
identical codegen and I would check with `-S` before claiming it."

### Follow-up Questions

- *"Does the anonymous-union trick fix it?"* Not in C++ — reading a union member other than the one
  last written is UB. It is legal in C, which is why so much graphics code inherited it.
- *"What about `std::launder` or `reinterpret_cast`?"* Neither creates an array object that wasn't
  there. `std::launder` reinterprets a pointer to an object that exists.
- *"Have you seen it break?"* Not in this project. I treat it like the `1/0` case — the standard's
  guarantee is what survives an optimiser, not the observed behaviour.
- *"Is the `assert` free in release?"* Yes, `NDEBUG` removes it. It has a death test in the Debug
  build, which is why both configurations are required.

---

## Q3 — You divide by zero deliberately. Isn't that undefined behaviour?

### Short Answer

It is, which is why I don't. The slab test genuinely wants `±inf` for an axis-parallel ray, but
`x / 0` is UB per `[expr.mul]/4` regardless of operand type — the IEEE-754 guarantee is a property of
the hardware, not of the C++ abstract machine. `safeReciprocal` constructs the identical value with
`copysign` instead, and `Vec3::operator/` does the same component-wise.

### Deep Answer

Start with why the infinity is wanted. When a ray has a zero direction component, its reciprocal is
infinite and the slab products become `±inf`. Those compare correctly: the slab is either fully
entered or fully missed, and the interval arithmetic gets the right answer with no branch. Branching
to special-case axis-parallel rays inside traversal would cost more than it saves — a per-node cost
to avoid a per-ray one.

So the goal is not to avoid infinity. The goal is to obtain it legally.

`[expr.mul]/4` says: *"If the second operand of `/` or `%` is zero, the behavior is undefined."* No
exception for floating point. IEEE-754 does define `1.0/0.0` as `+inf` and your CPU implements
that — but UB in C++ is not about what the hardware does, it is about what the optimiser is entitled
to assume. A compiler that sees `1.0f / d` may assume `d != 0` from that point forward and delete a
later check, or constant-fold under that assumption. UBSan's `float-divide-by-zero` check flags it,
and that is how this was found here.

```cpp
inline Scalar safeReciprocal(Scalar v) {
    if (v != Scalar(0)) return Scalar(1) / v;
    return std::copysign(kInfinity, v);
}
```

`copysign` rather than a bare `kInfinity` because `-0.0` must give `-inf`, matching `1 / -0.0`. That
matters: the sign determines which slab plane is the entry plane, so getting it wrong on a
negative-zero direction flips entry and exit for that axis.

**The fix later had to be extended, which is the more interesting half of the story.** `Vec3` has a
scalar division operator, reachable from `normalize`. In a Debug build `normalize` asserts on a
zero-length vector — but `NDEBUG` strips that assert, and then the division executes with a zero
divisor. So the precondition being violated in Release was still UB. The operator now guards it:

```cpp
inline Vec3 operator/(const Vec3& v, Scalar s) {
    if (s == Scalar(0)) {
        const Scalar inf = safeReciprocal(s);
        return {v.x * inf, v.y * inf, v.z * inf};
    }
    return {v.x / s, v.y / s, v.z / s};
}
```

`x * (±inf)` is `±inf` with the correct sign, and `0 * inf` is NaN — exactly what `x/0` and `0/0`
produce. That operator also deliberately keeps a *true divide* on the non-zero path rather than
multiplying by a reciprocal, because multiply-by-reciprocal adds a second rounding step and overflows
to infinity for a denormal divisor where the true quotient is finite. Not a hot path, so correctness
wins.

The general principle is the thing to carry into an interview: **"the hardware defines this" and "the
abstract machine defines this" are different claims, and only the second survives an optimiser.** The
same principle explains two other decisions here — the `(&x)[i]` avoidance (Q2) and the absolute ban
on `-ffast-math`, which would let the compiler assume the NaNs and infinities the slab test depends on
never occur. Three instances of one idea is a theme; one is trivia.

### Code Connection

- `safeReciprocal`: `include/geometry/scalar.hpp:58-62`; `[expr.mul]/4` reasoning at `:50-54`;
  `copysign`-for-negative-zero note at `:60`; cost analysis at `:56-57`.
- Called once per ray: `invDirection` — `include/geometry/ray.hpp:48-51`, with the "branching in
  traversal would cost more than it saves" argument at `:43-47`.
- `Vec3::operator/` guard and the true-divide justification: `include/geometry/vec3.hpp:60-76`;
  `operator/=` moved out of line to reach it (`:44`, `:78-83`).
- Why `normalize`'s release path still had to be defined: `include/geometry/vec3.hpp:101-112`.
- The slab test consuming the infinities: `include/geometry/aabb.hpp:180-184`.
- `-ffast-math` ban: `CMakeLists.txt:83-86`, also a project rule in `CLAUDE.md`.
- The IEEE requirement is asserted, not assumed: `include/geometry/scalar.hpp:26-27`.
- Tests: infinities with the right sign `tests/test_ray.cpp:42-51`; finite components unaffected
  `:53-59`; `Vec3 / 0` matches IEEE including `0/0 → NaN` `tests/test_preconditions.cpp:92-100`;
  Release-only check that `normalize(Vec3(0))` is NaN rather than UB `:78-90`.

### Tradeoffs

| Approach | Verdict |
|---|---|
| `1.0f / d` | Simplest, correct on hardware, UB in C++, flagged by UBSan |
| Suppress the sanitizer | Hides the report without fixing the UB — and loses the check for genuine bugs elsewhere |
| Clamp `d` away from zero | Removes the UB but introduces a *fictitious direction*: `t` becomes subtly wrong instead of correctly infinite |
| Branch inside the traversal loop | Moves a 3-per-ray cost to a 3-per-node cost |
| `safeReciprocal` + guarded `operator/` (chosen) | Same values, defined semantics, one predictable branch off the hot path |

### Follow-up Questions

- *"Is integer division by zero different?"* Also UB, but it traps on x86 (`#DE`) rather than
  producing a value, so it fails loudly. Float UB is worse precisely because it looks like it works.
- *"Is `0.0/0.0` also UB?"* Yes, same clause, and the "correct" IEEE answer would be NaN.
- *"What else does `-ffast-math` break here?"* It permits assuming no NaN and no inf, which turns the
  NaN-tolerant ternaries at `include/geometry/aabb.hpp:241-242` into code the compiler may simplify
  away — and it can reassociate the subtract-then-multiply, invalidating the `gamma(3)` bound. This
  is the rare flag that turns a correct program into an incorrect one rather than a faster one.
- *"How did you find it?"* UBSan, via `BVH_ENABLE_SANITIZERS` (`CMakeLists.txt:97-102`), while
  writing the axis-parallel ray tests. UBSan is clean in both configurations now.
- *"Why did the `Vec3` operator need fixing separately?"* Because the assert that was supposed to
  prevent the zero divisor disappears in Release — the exact configuration that ships. A precondition
  is a contract, not a guarantee of memory safety.

---

## Q4 — How do floating-point errors affect intersection tests?

### Short Answer

Two ways, both handled explicitly. Axis-parallel rays can produce `0 * inf = NaN`, so every
comparison in the slab loop is NaN-tolerant — a NaN leaves the interval unconstrained rather than
rejecting the box. And each `t` carries relative error up to `gamma(3)`, so the exit distance is
widened by that bound to keep the test conservative. The widening is a build option, because I can
argue for it but cannot yet measure that it matters.

### Deep Answer

**Hazard 1: NaN from `0 * inf`.** A zero direction component gives an infinite reciprocal. Normally
fine — the products are `±inf` and compare correctly. But if the ray origin lies *exactly* on a slab
plane, the numerator `box.min[axis] - origin[axis]` is exactly zero too, and `0 * inf` is NaN.

NaN makes every comparison false, which is either a disaster or a tool depending on how you write the
code. Here it is a tool:

```cpp
t0 = tNear > t0 ? tNear : t0;
t1 = tFar  < t1 ? tFar  : t1;
```

If `tNear` is NaN, `tNear > t0` is false and `t0` keeps its previous value. The degenerate axis
contributes nothing — the test simply does not constrain the interval there. Conservative: it may
report a hit exact arithmetic would not, costing one wasted narrow-phase test, but it can never
produce a false miss.

`std::max` and `std::min` are explicitly banned at that line. Their NaN behaviour is not guaranteed
to match: `std::max(a, b)` returns `a < b ? b : a`, whose result with a NaN operand depends on which
operand is NaN. The ternary makes the intent — "on NaN, keep what I had" — explicit.

**Hazard 2: rounding at grazing angles.** `t = (plane - origin) * invDir` is two operations, so by
Higham's bound the relative error is at most `gamma(3) = (3e)/(1 - 3e)` with `e = epsilon/2` — about
`1.8e-7` for float. Tiny, but the failure mode is not. Two sibling BVH nodes share a face; a ray
passes exactly through it; rounding pushes the left child's computed exit fractionally short and the
right child's computed entry fractionally long, so the ray is rejected by *both* and the geometry
behind the face is never tested. A visible crack — the classic watertightness bug. Widening the exit
distance makes the test err toward reporting a hit. Wasted work is recoverable; a hole is not.

**Here is where I would be honest, and it is the part that plays best.** The widening is a *named,
switchable constant*, not a hard-coded expression:

```cpp
#if BVH_CONSERVATIVE_RAY_BOX
inline constexpr Scalar kRayBoxWidening = Scalar(1) + Scalar(2) * gamma(3);
#else
inline constexpr Scalar kRayBoxWidening = Scalar(1);
#endif
```

Three reasons that shape is better than the expression inlined at the use site. First, flipping it
changes exactly one value and nothing else. Second, the disabled value is exactly 1, so the multiply
folds away rather than becoming a branch. Third — and this is the subtle one — the macro is defined
on the shared compile-options target rather than per file, because `intersectRay` is a header-inline
function and compiling it with different values in different translation units would be an ODR
violation.

And the reason it is switchable at all: I follow PBRT's error analysis, which is sound, but **no test
in this repository demonstrates a failure the widening prevents.** Its real justification arrives in
Phase 2, where watertightness matters between a node's bound and the triangle test inside it — and
there is no triangle test yet. So it is kept as the safe default and made measurable so Phase 9 can
price it. Structurally it is 3 of the 9 multiplies in the innermost loop of the whole system; what
that costs in time is `UNVERIFIED`.

What *is* tested is the **direction** of the effect: a randomised sweep compares the widened test
against a deliberately separate unwidened reference and asserts the widening never removes a hit,
logging rather than asserting how often it changes the answer at all — because "if a future change
makes the widening start mattering, that is information rather than a failure."

**The general policy** is that no geometric predicate uses an absolute epsilon. `kEpsilon` (`1e-6`)
exists for "are these two quantities the same" comparisons and is explicitly kept out of intersection
code, because an absolute epsilon is meaningless across magnitudes — too loose near the origin, too
strict at coordinate 10⁵. `gamma(3)` is *relative*, so it scales with the quantity it corrects. Two
other thresholds in the core follow the same rule: triangle degeneracy (Q5) and matrix singularity
(Q13).

### Code Connection

- Hazard analysis written out in full: `include/geometry/aabb.hpp:178-204`.
- NaN-tolerant comparisons and the `std::max`/`std::min` ban: `:239-242`.
- `gamma(n)` with the Higham bound stated: `include/geometry/scalar.hpp:37-45`.
- `kRayBoxWidening` and why it is a constant, not an `#if` at the use site:
  `include/geometry/aabb.hpp:11-25`; applied at `:237`; "folds away entirely" at `:235-236`.
- The ODR reasoning for the macro's placement: `CMakeLists.txt:88-95`; the option itself at `:52-53`;
  reported in the configuration summary at `:149`.
- The "HONEST STATUS" block, including "Do not restate the PBRT rationale as a measured result until
  there is a benchmark behind it": `include/geometry/aabb.hpp:194-204`.
- "No single global epsilon" policy: `include/geometry/scalar.hpp:32-34`.
- Test for the exact `0 * inf` case — origin on the `y = 1` face, no `y` direction:
  `tests/test_aabb.cpp:207-216`.
- Randomised test that the widening never removes a hit, with a separate unwidened reference:
  `tests/test_aabb_property.cpp:24-44`, `:113-143`.
- Overflow avoidance elsewhere: `AABB::centroid` is `min + 0.5*(max-min)` rather than `0.5*(min+max)`
  — `include/geometry/aabb.hpp:81-82`.

### Tradeoffs

| Approach | Consequence |
|---|---|
| Ignore both hazards | Occasional false misses; cracks between siblings; NaN-dependent behaviour |
| Widen the exit by `gamma(3)` (chosen, default on) | Slightly conservative; a few wasted narrow-phase tests |
| Widen both ends | More conservative still; more false positives for no extra safety on the crack case |
| Global absolute epsilon | Wrong at both ends of the magnitude range |
| Exact / adaptive-precision predicates | Exact answers, much slower; right for mesh booleans, overkill for an acceleration structure |
| Double precision | Shrinks the error but does not eliminate it, and costs the node density from Q1 |

### Follow-up Questions

- *"Why widen only `tFar`?"* It closes the sibling-crack case, which arises on the exit side, and it
  is the published form. Widening `tNear` too is more conservative at the cost of more false
  positives — I'd want a measurement before changing it.
- *"You said no test demonstrates a failure it prevents. Why keep it?"* Because a conservative bound
  is the safe default when the failure mode is a visible hole, and because Phase 2 introduces the
  node-bound-to-triangle-test boundary where it actually bites. Making it switchable means I am not
  defending it forever on faith — Phase 9 prices it.
- *"How would you construct a test that fails without it?"* Two adjacent boxes sharing a face, and a
  ray whose entry lands exactly on that plane at a magnitude where float spacing exceeds the error;
  assert at least one box reports a hit. I haven't managed to build one that discriminates yet, which
  is exactly why the status note says so.
- *"What's the ODR issue you mentioned?"* `intersectRay` is `inline` in a header. If TU A compiles it
  with widening on and TU B with it off, the two definitions differ and the program is ill-formed,
  no diagnostic required — the linker just picks one. Putting the macro on the shared INTERFACE
  target makes it impossible to get wrong.

---

## Q5 — What happens with degenerate geometry?

### Short Answer

Every degenerate case has a defined, finite result and a named test. Zero-area triangles return a
zero normal rather than NaN and still have valid bounds and centroids, so a BVH can carry them
without special-casing; flat AABBs are real zero-volume regions and stay hittable; `AABB::offset`
returns zero on a degenerate axis; and the degeneracy predicate itself is a scale-invariant aspect
ratio rather than an absolute area, so a small-but-healthy triangle is not falsely flagged.

### Deep Answer

Degenerate input is not hypothetical — exported meshes routinely contain zero-area triangles from
welded or duplicated vertices. The design rule is: *never return NaN, and never silently drop data.*

**The degeneracy predicate is the interesting part, because the obvious version is wrong.** The
natural test is `area <= tol`. That is scale-dependent, and the consequences are concrete: with an
absolute tolerance of `1e-6`, a perfectly healthy triangle with `1e-3` edges has area `5e-7` and gets
flagged, and a unit-scale mesh of a million triangles has a mean triangle area near `6e-6` — right at
the threshold. A degeneracy predicate that fires on a tenth of a valid mesh is worse than none.

The implementation compares a **shape ratio**:

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

Derive it on a whiteboard, because it is short and it makes the point: `normalUnnormalized()` is
`cross(v1-v0, v2-v0)`, whose magnitude is twice the area. Let `L` be the longest edge and `h` the
height above it; then `|cross| = 2·area = L·h`. The predicate is `|cross|² ≤ (tol·L²)²`, i.e.
`L·h ≤ tol·L²`, i.e.

```
h / L  ≤  tol
```

A pure aspect ratio — dimensionless, invariant under uniform scaling. Squaring both sides avoids the
`sqrt`. The all-vertices-coincident case has no edge at all and short-circuits, which also keeps
`limit` from being zero. So `tol` is **not** an area; it is a shape parameter.

**Zero-area triangles are carried, not filtered.** `normal()` uses `normalizeSafe`, which checks
squared length against a squared tolerance and returns a zero fallback rather than dividing by zero.
The crucial design point is what the comment says next: such triangles "cannot be hit by a ray in any
meaningful sense, but they still have valid bounds and a valid centroid, so a BVH can carry them
without special-casing — they simply never report a hit." That is a much better answer than "I filter
them out": filtering changes the primitive count, which silently changes every benchmark you then
compare against. `Mesh::countDegenerateTriangles` exists so input quality is *visible* rather than
quietly corrected.

**Flat AABBs.** A zero-thickness box is a real region, unlike an empty box, and the code keeps that
distinction sharp. A 2×3×0 box has surface area 12 and volume 0, and the slab test's final comparison
is strict (`if (t1 < t0) return false`) specifically so a flat box with `t1 == t0` still hits. With
`<=`, every axis-aligned planar primitive would become invisible.

**Degenerate axes in binning.** `AABB::offset` guards each divide (`if (max.x > min.x) o.x /= …`).
SAH binning hits this whenever all centroids share a coordinate — a planar mesh, or the last few
primitives in a subdivision.

**Elsewhere:** a zero-length rotation axis yields identity rather than NaN; collinear or coincident
points give a degenerate plane with a zero normal (both `Plane` factories use `normalizeSafe`, not
just one); and an all-zero matrix row makes `invert` return `false` before any division.

**And one case that is deliberately *not* tolerated.** `AABB::centroid()` and `offset()` on an *empty*
box would produce NaN (`+inf + (-inf)·0.5`). Rather than silently return it, both declare a non-empty
precondition and assert. The comment names the Phase 2 failure it prevents: SAH binning takes the
centroid of a primitive range's bounds, an empty range at a recursion boundary is exactly how you get
there, and a NaN centroid then propagates into a bin index and corrupts the tree instead of crashing.
That is the right call — degenerate *geometry* is data, but an empty *set* is a caller error.

### Code Connection

- `Triangle::isDegenerate`, the dimensionless derivation, and the absolute-area counter-argument:
  `include/geometry/triangle.hpp:46-72`; `normalUnnormalized` magnitude note at `:33-34`;
  `normal()` via `normalizeSafe` at `:39-42`.
- `normalizeSafe`: `include/geometry/vec3.hpp:114-121`; asserting `normalize` and its documented
  precondition at `:101-112`.
- Degenerate vs empty box distinction: `include/geometry/aabb.hpp:53-55`.
- Strict `<` so flat boxes stay hittable: `include/geometry/aabb.hpp:244-245`.
- `AABB::offset` divide guards: `:141-143`; SAH-binning motivation at `:135-137`.
- Empty-box preconditions and the Phase 2 justification: `:72-80`, `:138-139`.
- Degenerate rotation axis → identity: `src/geometry/mat4.cpp:31-32`.
- Both `Plane` factories degenerate-safe: `include/geometry/plane.hpp:22-37`.
- All-zero matrix row rejected before any division: `src/geometry/mat4.cpp:121-122`.
- Report-don't-drop: `include/geometry/mesh.hpp:92-95`, `src/geometry/mesh.cpp:36-43`.
- Tests: scale-invariance — a healthy area-`5e-7` triangle is not flagged, and the same shape agrees
  at unit and `1e5` scale (`tests/test_triangle.cpp:110-123`); a sliver is flagged at every scale
  (`:125-133`); the verdict is unchanged across eight orders of magnitude (`:135-145`); collinear and
  duplicate-vertex cases (`:82`, `:91`, `:100`); flat-box area and hittability
  (`tests/test_aabb.cpp:65`, `:235`); degenerate-axis offset (`:142`); empty-box preconditions
  (`tests/test_preconditions.cpp:50-59`); degenerate plane (`tests/test_plane.cpp:30`, `:36`);
  degenerate rotation axis (`tests/test_mat4.cpp:86`); mesh degenerate count
  (`tests/test_mesh.cpp:119`).

### Tradeoffs

| Policy for degenerate triangles | Consequence |
|---|---|
| Carry them, never hit (chosen) | No special case anywhere; primitive count is honest; a few zero-area leaves |
| Filter at load | Cleaner tree, but the primitive count no longer matches the file, which corrupts benchmark comparisons |
| Assert / reject the mesh | Rejects real-world assets; most exported meshes contain some |
| Snap or weld vertices | Changes the geometry; a repair step, not a geometry-core decision |

| Degeneracy threshold | Consequence |
|---|---|
| Absolute area (`area <= tol`) | Scale-dependent; flags healthy small triangles; mis-fires on dense unit-scale meshes |
| **Aspect ratio `h/L <= tol` (chosen)** | Dimensionless and scale-invariant; the verdict depends only on shape |
| Relative to mesh bounds diagonal | Also scale-invariant, but couples a per-triangle predicate to global state |

Similarly for empty boxes: returning 0 from `surfaceArea` keeps SAH cost sums finite; returning NaN or
a negative area would poison every cost comparison up the tree.

### Follow-up Questions

- *"Is `1e-6` the right aspect-ratio threshold?"* It means `h/L ≤ 1e-6`, which is very permissive —
  only extreme slivers are flagged. The *form* is right; the *value* is an open question a Phase 2
  measurement would settle. It also happens to share `kEpsilon`'s unrelated "same quantity" role,
  which I'd separate.
- *"What if a whole mesh is planar?"* The root AABB is flat on one axis: non-zero surface area, zero
  volume, still hittable, and `offset` returns 0 on the flat axis so binning still works. Every piece
  of that path is tested.
- *"Would a degenerate triangle break ray/triangle intersection?"* Möller–Trumbore's determinant is
  zero for a degenerate triangle and the standard epsilon test rejects it, so it never reports a hit
  — consistent with the `isDegenerate` comment. Not implemented in Phase 1.
- *"Why assert on an empty box instead of returning something?"* There is no correct value to return.
  Returning NaN propagates silently into a bin index; returning zero is a lie about where the box is.
  A precondition with a death test is the honest option.

---

## Q6 — Why AABBs, and not OBBs, spheres, or k-DOPs?

### Short Answer

The bounding volume is tested far more often than it is built, so what matters is the per-test cost,
not the tightness. An AABB/ray test is a handful of multiplies and comparisons with no matrix and no
trigonometry, and AABBs are closed under union via component-wise min/max — which is what makes
bottom-up bounds propagation in a BVH essentially free. The cost is a looser fit for diagonal
geometry, showing up as extra false-positive node visits.

### Deep Answer

Frame it as an amortisation question. A node's bounding volume is built **once** and tested
**millions of times**, so the objective is

```
total cost  ≈  N_tests × cost_per_test  +  (false positives) × cost_of_narrow_phase_test
```

Tighter volumes reduce the second term and increase the first. For ray queries against a BVH the
first dominates, because the whole point of the hierarchy is that most tests are rejections at
internal nodes.

**Against OBBs.** An oriented box is tighter — sometimes dramatically, for a long diagonal object —
but each test needs the ray transformed into the box's local frame first, a matrix multiply before
the slab test even starts. It also stores a rotation per node, inflating node size and hurting the
cache-density argument from Q1. And critically for a BVH: **the union of two OBBs is not an OBB.**
Computing a tight parent OBB from two children is an optimisation problem, not two `min`/`max` calls,
so bottom-up bounds propagation stops being cheap. OBBs earn their keep in collision detection with a
small number of rigid bodies, not in a per-node acceleration hierarchy.

**Against spheres.** Cheapest test of all, and the union is easy — but the fit is terrible for
anything elongated, and most triangles are elongated relative to a sphere. A sphere bounding a thin
diagonal triangle wastes enormous volume, and wasted volume is false-positive descents.

**Against k-DOPs.** A discrete oriented polytope with `k` fixed plane normals (typically 14, 18, 26)
is the principled middle ground: tighter than an AABB, still closed under union component-wise
because the normals are shared and fixed. But cost and storage scale with `k`: a 26-DOP is 13 slab
tests instead of 3, and 26 scalars per node instead of 6. You trade exactly the thing Q1 says is
scarce — bytes per node — for tightness.

**What AABBs give structurally**, beyond the cheap test:

1. *Closed under union, component-wise.* `extend` is two `min`/`max` calls and no branch — what makes
   bottom-up bounds propagation cheap.
2. *Surface area is a closed form*, which is what makes the SAH computable at all (Q7).
3. *`offset` maps a point to a bin index in three divides*, which makes binned SAH cheap.

**The honest cost** is in the source: looser fit for diagonal geometry, appearing as more
false-positive node visits. Quantifying that against an OBB tree would need measurement —
`UNVERIFIED`, no benchmark exists.

### Code Connection

- The full argument, in the header where the decision lives: `include/geometry/aabb.hpp:29-36`.
- Branch-free component-wise union: `:58-68`, relying on `minComponents`/`maxComponents` at
  `include/geometry/vec3.hpp:123-128`.
- The test that pins why union-with-empty must be the identity, named for its BVH consumer:
  `tests/test_aabb.cpp:48-56` ("Relied on by bottom-up bounds propagation in BVH construction").
- Closed-form surface area: `include/geometry/aabb.hpp:93-97`.
- `offset` for bin indexing: `:135-145`.
- The cheap test itself: `:211-250`, "no divides, because the caller supplies the reciprocal
  direction" at `:175-176`.
- 24 bytes per box: `:44-45`.

### Tradeoffs

| Volume | Test cost | Storage (float) | Union | Fit |
|---|---|---:|---|---|
| **AABB (chosen)** | ~6 mul, ~6 sub, compares | 24 B | component-wise min/max, branch-free | loose for diagonal geometry |
| Sphere | cheapest | 16 B | easy, but loose | very poor for elongated shapes |
| OBB | + a transform into local frame | 24 B + rotation | **not closed** — an optimisation problem | tightest of these |
| k-DOP (k=14…26) | k/2 slab tests | 4k B | component-wise (normals fixed) | between AABB and OBB |

### Follow-up Questions

- *"When would you switch to OBBs?"* Collision between a small number of rigid bodies where geometry
  is strongly oriented and the tree is shallow — build cost amortised over fewer, more expensive
  tests. Not for a per-node ray-query hierarchy.
- *"What about compressed / quantised node bounds?"* That is the production answer to the
  tightness-versus-bytes tension: store child bounds as 8-bit offsets from the parent. Needs directed
  rounding (min down, max up) to stay conservative. Phase 2+.
- *"Does AABB tightness depend on the mesh's orientation?"* Yes — exactly the weakness. A 45°-rotated
  thin plate has an AABB many times its actual volume. A good experiment for the explorer: rotate the
  scene and watch node-visit counts change with no change to the geometry.
- *"How would you measure the false-positive cost?"* Instrument traversal to count nodes visited and
  triangles tested per ray, against a brute-force baseline. Phase 5 of the plan calls for exactly
  that. Not implemented.

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
surface area. (A corollary of Cauchy's formula: the mean projected area of a convex body over all
directions is one quarter of its surface area.) For a convex child `C` contained in a convex parent
`P`, a uniformly distributed ray conditioned on hitting `P` hits `C` with probability

```
P(hit C | hit P) = SA(C) / SA(P)
```

That single fact is the whole justification. The expected cost of a candidate split into children
`L` and `R`:

```
Cost(split) = C_trav + P(hit L)·N_L·C_isect + P(hit R)·N_R·C_isect
            = C_trav + [ SA(L)·N_L + SA(R)·N_R ] · C_isect / SA(P)
```

`SA(P)` and `C_isect` are constant across candidate splits of the same node, so choosing the best
split means minimising `SA(L)·N_L + SA(R)·N_R` — an **area-weighted primitive count**. Everything
else about SAH — the sweep, the binning, the bin-count parameter — is machinery for evaluating that
objective quickly.

Now the "why not volume" part, where the question is usually really aimed. Volume is the right
measure for *point* containment: the probability a uniformly distributed point lands in a child is
the volume ratio. But rays are lines, not points, and the measure on lines is the area one. Volume
weighting gives a heuristic with no probabilistic justification for ray queries, and it fails on
exactly the case that matters: a flat box — a planar sheet of geometry — has **zero volume but
substantial surface area**. Under volume weighting its cost is zero, so the builder would happily
create such splits despite rays hitting that sheet constantly.

This project pins that case explicitly: a 2×3×0 box has surface area 12 and volume 0, both asserted
in a test named for the distinction.

Two implementation details follow from `surfaceArea`'s role as a cost term. The empty guard exists so
SAH cost sums stay finite — without it an empty child's diagonal is `-inf` and the sum becomes NaN,
poisoning every comparison up the tree. And the function is `constexpr` and branch-light because it is
evaluated once per bin per axis per node during a build.

**What does not exist yet:** the SAH itself. Only the cost term and the binning helper are here.
`UNKNOWN — not yet implemented`.

### Code Connection

- `surfaceArea` with the ray-hit-probability justification stated inline:
  `include/geometry/aabb.hpp:86-97` — "for a convex volume, the probability that a uniformly
  distributed random ray hitting the parent also hits the child is the ratio of their surface areas.
  That relationship is why SAH minimises area-weighted primitive counts rather than, say,
  volume-weighted ones."
- Empty guard and its reason ("so SAH cost sums stay finite"): `:92`, `:94`.
- `volume()` provided for reporting, not for cost: `:99-103`.
- The flat-box case that distinguishes the two measures: `tests/test_aabb.cpp:65-72`
  (`SurfaceAreaOfFlatBoxIsTwiceTheFace`), area 12, volume 0.
- Known-answer area test (1×2×3 → 22): `tests/test_aabb.cpp:58-63`.
- `longestAxis` as the cheap proxy split axis when SAH is not used — "a cheap proxy for the axis whose
  split will most reduce child surface area": `include/geometry/aabb.hpp:105-107`.
- The binning helper SAH will need: `AABB::offset` — `:135-145`.
- Why the mesh root bound is the *tight* one, stated in SAH terms: "The SAH normalises each child's
  surface area against its parent's, so a root inflated by unreferenced vertices would shift every
  split decision" — `include/geometry/mesh.hpp:80-85`.

### Tradeoffs

| Cost measure | Justification | Failure mode |
|---|---|---|
| **Surface area (chosen)** | geometric probability for lines | assumes uniformly distributed rays — real distributions are not uniform |
| Volume | correct for *point* queries | a flat box costs 0; planar geometry is split disastrously |
| Primitive count only (median split) | trivially cheap to build | ignores that a large box is hit more often |
| Measured ray distribution | matches the real workload | needs a representative ray set at build time; not general-purpose |

The SAH's own assumptions are worth naming, because a good interviewer will: rays are uniformly
distributed and infinite; they do not terminate early on a hit; children's costs are independent. All
three are false in practice — primary rays come from a point, traversal shrinks `tMax` on every hit —
and SAH still wins comfortably, which is the interesting part.

### Follow-up Questions

- *"Where does the surface-area/probability result come from?"* Integral geometry — Cauchy's formula
  gives mean projected area = SA/4 for a convex body, and the measure of lines meeting a convex body
  is proportional to that.
- *"Does it matter that SAH assumes rays don't terminate early?"* Traversal shrinks `tMax` as hits are
  found, so a near child's real cost is lower than SAH assumes. Little effect in practice;
  front-to-back ordered traversal recovers most of it.
- *"What is `C_trav / C_isect` in your cost model?"* `UNKNOWN — not yet implemented`. It is the ratio
  a real builder must pick, and the leaf-size decision falls out of it.
- *"Why bin instead of a full sweep?"* A full sweep is O(n log n) per node from the sort; binning is
  O(n) per node with a small constant, at the cost of only evaluating `k` candidate planes. Bin count
  is a Phase 3 parameter (4/8/16/32 in the plan). Not implemented.
- *"Your `offset` returns 0 on a degenerate axis — what does that do to binning?"* Every centroid maps
  to bin 0, so that axis produces no valid split and the builder must fall back to another axis or
  make a leaf. Guarding the divide keeps it a *decision* rather than a NaN.

---

## Q8 — Why is an empty AABB represented as `min = +inf, max = -inf`?

### Short Answer

Because it makes `extend` branch-free. Merging the first point into an empty box gives exactly that
point's degenerate box, since `min(+inf, p) == p` and `max(-inf, p) == p`, and merging an empty box
into a real one is automatically a no-op. A zero-initialised box would instead wrongly contain the
origin.

### Deep Answer

`extend` is the innermost operation of bounds computation — once per vertex when bounding a mesh, and
once per child at every node when propagating bounds up a BVH. Any branch in it is a branch in the
hottest loop of the build.

The inverted sentinel removes the branch by making the identity element of the union operation
*representable*. Algebraically: boxes under union form a monoid, and `[+inf, -inf]` is its identity.
Component-wise `min`/`max` then handle the empty case for free:

```cpp
void extend(const Vec3& p) { min = minComponents(min, p); max = maxComponents(max, p); }
void extend(const AABB& b) { min = minComponents(min, b.min); max = maxComponents(max, b.max); }
```

Both directions work. First point into an empty box: `min(+inf, p) = p`, `max(-inf, p) = p` — a
degenerate single-point box. Empty box into a real box: `min(m, +inf) = m`, `max(M, -inf) = M` —
unchanged. Neither case is special-cased anywhere in the file.

**Why the obvious alternatives are worse.** A zero-initialised box is not merely inelegant, it is
*silently wrong*: `AABB{{0,0,0},{0,0,0}}` already contains the origin, so bounding a mesh that sits
entirely at `x > 100` produces a root stretching back to zero. Every node inherits the inflation,
every ray tests boxes that contain nothing, and there is no crash to tell you.

An explicit `bool empty` flag is correct but costs a branch in `extend` and widens the struct from 24
bytes to 28 or 32 with alignment — directly against the node-density argument from Q1.

**`isEmpty()` follows from the representation:** `min > max` on at least one axis. And the
representation forces a distinction the header draws carefully: *empty* (not a region at all) versus
*degenerate* (a point or flat plane — a real, zero-volume region that legitimately participates in
intersection tests). The two behave oppositely in the ray test: an empty box is never hit; a flat box
is always hittable. Both tested under those names.

**Where the sentinel bites — and this is the part worth volunteering.** It is not free. It interacts
with every arithmetic path that assumes finite corners, and there are three:

1. The ray test must be written so the inverted interval *stays* inverted. Swapping on `tNear > tFar`
   repairs `[+inf, -inf]` into `[-inf, +inf]` and makes every empty box hit every ray. That is Q10,
   and it is the best story in this milestone.
2. `surfaceArea` and `volume` need an explicit empty guard, or `max - min` is `-inf` and the products
   are NaN. Both have one, returning 0 so SAH cost sums stay finite.
3. `centroid` and `offset` would produce NaN — `+inf + (-inf)·0.5`. These do **not** guard and return
   a value; they declare a non-empty **precondition** and assert it, with the comment naming the
   Phase 2 failure: SAH binning takes the centroid of a primitive range's bounds, an empty range at a
   recursion boundary is how you get there, and a NaN centroid propagates into a bin index and
   corrupts the tree instead of crashing.

So the honest framing is: the sentinel buys a branch-free hot path and pays for it with three places
that must handle infinity deliberately. All three now do, two by guarding and one by contract — and
the choice between guarding and asserting is itself a judgement, made on whether there is a
meaningful value to return.

### Code Connection

- The sentinel and full rationale: `include/geometry/aabb.hpp:38-45` — "A zero-initialised box would
  instead wrongly contain the origin."
- Branch-free `extend`, both overloads, with the no-op-on-empty comment: `:58-68`.
- `isEmpty` and the empty-vs-degenerate distinction: `:53-56`.
- Guards that return a value: `surfaceArea` `:94`, `volume` `:100`; reason at `:92`.
- Preconditions that assert instead: `centroid` `:72-80`, `offset` `:138-139`.
- Empty-set semantics, with the asymmetry explained: `contains(AABB)` true for empty (`:116-121`),
  `intersects` false (`:125-129`) — "(empty subset A) is true, while (empty intersect A) is empty."
- Tests: `tests/test_aabb.cpp:9-18` (sentinel values, zero measures), `:20-30` (first point gives a
  degenerate box), `:48-56` (union with empty is the identity — named for its BVH consumer), `:244-248`
  (empty box never hit); `tests/test_preconditions.cpp:50-59` (both preconditions).

### Tradeoffs

| Representation | `extend` cost | Size | Correctness risk |
|---|---|---:|---|
| **Inverted sentinel (chosen)** | branch-free, 2 min + 2 max | 24 B | infinities must be handled in every arithmetic path |
| `bool empty` flag | a branch per extend | 28–32 B | none, but the branch is in the hottest loop |
| Zero-initialised | branch-free | 24 B | **silently wrong** — contains the origin |
| `std::optional<AABB>` | branch + unwrapping everywhere | 28–32 B | none; heavy at every call site |
| Separate `EmptyAABB` type | none at runtime | 24 B | type proliferation; conversions everywhere |

### Follow-up Questions

- *"How does an empty box behave in the ray test?"* It must never hit. That is not automatic — Q10.
- *"Difference between empty and degenerate?"* Empty is "not a region"; degenerate is a real region of
  zero volume. A flat box has surface area 12 and is hittable; an empty box has area 0 and is not.
- *"Why does `surfaceArea` guard but `centroid` assert?"* Because zero is a meaningful, useful answer
  for the area of an empty box — it keeps cost sums finite. There is no meaningful centroid of the
  empty set, so returning anything would be a lie.
- *"Does `merge(empty, empty)` work?"* Yes, it stays empty — tested at `tests/test_aabb.cpp:55`.
- *"Why is `contains(empty)` true but `intersects(empty)` false?"* Empty-set semantics: the empty set
  is a subset of everything and intersects nothing. Both deliberate, both tested, and the header now
  says so explicitly so it doesn't read as an inconsistency.

---

## Q9 — Walk me through the slab method.

### Short Answer

An AABB is the intersection of three axis-aligned slabs. For each axis I compute the two `t` values
where the ray crosses that slab's planes, intersect the running `[tEnter, tExit]` interval with them,
and bail out as soon as the interval inverts. Six multiplies, six subtracts, a few compares, and no
divides — the caller supplies the reciprocal direction.

### Deep Answer

**The geometry.** The box `[min, max]` is `{p : min ≤ p ≤ max}` componentwise, which is exactly the
intersection of three *slabs*, each the region between two parallel planes. A ray hits the box iff the
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
    tFar *= kRayBoxWidening;
    t0 = tNear > t0 ? tNear : t0;     // intersect: entry = max
    t1 = tFar  < t1 ? tFar  : t1;     // intersect: exit  = min
    if (t1 < t0) return false;
}
```

`t0` starts at the ray's own `tMin` and `t1` at its `tMax`, so the ray's valid range is just a fourth
interval in the same intersection — no separate range check is needed. On success `t0` is returned as
the entry distance, which for a ray originating *inside* the box is `tMin`, not a negative backward
distance.

**Cost.** Two subtracts and two multiplies per axis for the `t` values, plus the widening multiply,
plus three compares: 6 subtracts and 6 multiplies over three axes for the core, and the source counts
the widening separately as "3 of the 9 multiplies in this function". No divides, because `invDir` was
computed once per ray. That is the point of precomputing the reciprocal: three divisions per box
become three multiplications, and a box test happens once per node visited.

The early-out at `t1 < t0` matters more than it looks: most tests in a BVH are rejections, and a ray
that misses on the first axis pays for one axis, not three.

**Why the swap is on `sign(invDir)` and not `tNear > tFar`** — Q10, the most interesting line in the
function.

**The two numerical hazards** — `0 * inf = NaN` and the `gamma(3)` widening — Q4.

**Why the final comparison is strict.** `t1 < t0`, not `t1 <= t0`, so a flat box where `t1 == t0`
still reports a hit. Planar geometry stays visible.

### Code Connection

- The routine: `include/geometry/aabb.hpp:211-250`; full derivation and hazard analysis in the comment
  block at `:170-210`.
- Cost claim ("6 multiplies, 6 adds... no divides, because the caller supplies the reciprocal
  direction"): `:175-176`; the widening counted separately at `:202`.
- Interval initialised from the ray's own range: `:213-214`; `tEnter` semantics at `:206-207`.
- Early-out: `:245`. Strict `<` for flat boxes: `:244-245`.
- Precomputed reciprocal, once per ray: `include/geometry/ray.hpp:38-51`.
- Convenience overload that recomputes `invDirection` per call — fine for tests, **not** for
  traversal: `include/geometry/aabb.hpp:252-254`.
- `Ray::tMax` carried in the ray so traversal can shrink it: `include/geometry/ray.hpp:21-24`.
- Twelve hand-written ray cases (`tests/test_aabb.cpp:152-248`) plus five randomised invariants
  (`tests/test_aabb_property.cpp`).

### Tradeoffs

| Alternative | Consequence |
|---|---|
| Slab method (chosen) | Branch-light, no divides, uniform across axes, vectorises well |
| Divide inside the test | Three divides per box instead of three multiplies per ray |
| Separate-axis / plane-by-plane tests | More branches, worse for SIMD |
| Precomputed sign-based corner lookup | Avoids the swap entirely by indexing `bounds[sign[axis]]`; needs a 2-element bounds array and a per-ray sign vector — a real alternative, arguably cleaner, and it makes the Q10 bug class impossible by construction |
| Branchless `std::max`/`std::min` | **Rejected here** — NaN behaviour is not guaranteed to match the required semantics |

The sign-lookup variant is the one to mention if asked "how else could you write it": storing the box
as `Vec3 bounds[2]` and reading `bounds[sign[axis]][axis]` replaces the swap with an index. A genuine
improvement in that it makes the swap bug unrepresentable, at the cost of a per-ray sign vector and a
less readable box type.

### Follow-up Questions

- *"Why is `tEnter` returned at all?"* Front-to-back ordered traversal: visit the nearer child first,
  and skip the farther subtree once a hit closer than its entry distance is found. The single most
  effective pruning mechanism in a BVH, and why `tMax` lives in the `Ray`.
- *"How would you SIMD this?"* Two ways: 4 or 8 rays against one box (packet traversal), or one ray
  against 4/8 children of a wide BVH node. The second is the modern choice and wants a
  structure-of-arrays node layout. Not implemented.
- *"Does it work for a ray starting inside the box?"* Yes — `t0` never drops below `tMin`, so the
  reported entry is `tMin`. Tested both by hand and over 5,000 random rays.
- *"What if `tMin > tMax` on the ray itself?"* The first axis comparison rejects it immediately, since
  the interval starts already inverted.
- *"Where does this get called from?"* Nowhere yet — BVH traversal is Phase 2.
  `UNKNOWN — not yet implemented`.

---

## Q10 — Tell me about a bug you found and fixed.

> Two, and they are the strongest material in this milestone. Both were found while writing tests,
> both are recorded in the `841777e` commit message and in the source comments, and — because Phase 1
> landed as one commit — neither appears as a diff. The buggy versions never got committed.

### Short Answer

The slab test originally swapped `tNear`/`tFar` on the condition `tNear > tFar`, the form you see in
most published implementations. It is wrong in combination with an inverted-interval empty box: the
swap "repairs" `[+inf, -inf]` into `[-inf, +inf]`, so **every empty box reports a hit against every
ray**. Swapping on the sign of the reciprocal direction instead leaves the inverted interval inverted,
so the `tExit < tEnter` check rejects it. Same instruction count, one fewer way to be wrong.

### Deep Answer

**The setup.** An empty `AABB` is `min = +inf, max = -inf` — the inverted sentinel from Q8. Take a
default-constructed box and a ray with positive `x` direction, so `invDir.x > 0`:

```
tNear = (+inf - origin.x) * invDir.x = +inf
tFar  = (-inf - origin.x) * invDir.x = -inf
```

**With the comparison-based swap:** `tNear > tFar` is `+inf > -inf`, true. Swap. Now `tNear = -inf`,
`tFar = +inf`. The interval `[-inf, +inf]` constrains nothing: `t0 = max(tMin, -inf) = tMin`,
`t1 = min(tMax, +inf) = tMax`. Nothing changed. All three axes behave identically, `t1 < t0` never
fires, and the function **returns true**.

The `if` intended to repair a negative-direction ordering has instead repaired the sentinel that
encodes "this box is not a region at all".

**With the sign-based swap:** `invDir.x` is positive, so no swap. `t0 = max(tMin, +inf) = +inf`,
`t1 = min(tMax, -inf) = -inf`, and `t1 < t0` fires on the very first axis. Correctly rejected.

Check the other cases, because "it happens to work for one sign" is not a fix. Negative direction:
`tNear = -inf`, `tFar = +inf`, the sign test swaps them to `+inf`/`-inf`, rejection still fires.
Axis-parallel (`invDir = +inf`): the same `+inf`/`-inf` pair, same rejection. The sign form is correct
in every case, and correct for the ordinary non-empty box too, because **the sign of the direction is
what determines the crossing order in the first place** — `tNear > tFar` is a *symptom* of a negative
direction, and the sign is the *cause*. Testing the cause cannot be confused by a degenerate input.

**Why this matters in a BVH, not just in a unit test.** Empty bounds are routine there: a node with no
primitives after a partition, a partially constructed tree, an unfilled child slot, a conservatively
initialised bin during SAH binning. If every empty node reports a hit, traversal descends into it,
tests whatever it finds, and comes back. **Nothing crashes. No pixel is wrong.** The acceleration
structure just quietly accelerates less, and the resulting performance cliff is very hard to
attribute — you would be profiling the traversal loop looking for a cache problem.

**Why it is a good interview story.** The two forms look equivalent. The buggy one is the one almost
everyone has seen and copied. The failure is silent and performance-only, the hardest class of bug to
find. It only manifests through an interaction between two *separately correct* design decisions — the
inverted-interval empty box and the standard slab test. And the fix costs exactly nothing: one
comparison either way, on a value already in a register. The comment's own summary is right: *"Same
cost, one fewer way to be wrong."*

**The second bug** pairs well and generalises further: `invDirection` originally computed `1/d`
directly. For an axis-parallel ray that is a literal division by zero. IEEE-754 defines the result as
`±inf` and that is exactly the value the slab test wants — but `[expr.mul]/4` makes it undefined
behaviour regardless of operand type, and UBSan flagged it. The fix was not to suppress the check but
to construct the same value with `copysign`. And it did not end there: the same latent UB existed in
`Vec3::operator/`, reachable through `normalize` in a Release build where the assert is gone, so the
operator now guards the zero divisor too. Full treatment in Q3; the transferable part is that "the
hardware defines this" and "the abstract machine defines this" are different claims.

### Code Connection

- The fix and the full explanation of what was rejected: `include/geometry/aabb.hpp:229-233`, reasoning
  at `:220-228` — "The comparison form looks equivalent and is widely published, but it silently
  'repairs' an inverted (empty) box into an infinite one."
- The sentinel that makes it possible: `:44-45`.
- The check that does the rejecting: `:245`. The guarantee it protects: `:209-210`.
- Regression test: `tests/test_aabb.cpp:244-248` (`EmptyBoxIsNeverHit`). Negative-direction coverage
  at `:218-225`. Randomised sweep over 5,000 directions including axis-aligned and negative ones,
  with a comment naming this bug as its reason: `tests/test_aabb_property.cpp:159-169`.
- Second bug: `safeReciprocal` — `include/geometry/scalar.hpp:47-62`; call site
  `include/geometry/ray.hpp:48-51`; the later extension to `Vec3::operator/`
  `include/geometry/vec3.hpp:60-76`; tests `tests/test_ray.cpp:42-59`,
  `tests/test_preconditions.cpp:78-100`.
- Both recorded in the `841777e` commit message. The sanitizer that found the second:
  `CMakeLists.txt:97-102`.

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

- *"How did you find it?"* By writing `EmptyBoxIsNeverHit` as a degeneracy test, before there was any
  BVH to notice the symptom. That is the argument for testing degenerate inputs at the primitive level
  rather than waiting for the system to behave oddly.
- *"Would a fuzzer have found it?"* A random-box fuzzer, probably not — empty boxes have measure zero
  in any natural random distribution. You find this by enumerating *representation* edge cases, not
  value ranges. Which is why the randomised suite added afterwards tests the empty box explicitly as a
  fixed input and randomises only the ray.
- *"Is the published form wrong in general?"* No — it is correct for any box with `min ≤ max`. It is
  wrong *in combination with* an inverted-interval empty box. Two locally correct decisions, one bug
  at the interface. The most common shape for real bugs.
- *"Did the same sentinel cause anything else?"* Yes — `AABB::centroid()` would have returned NaN for
  an empty box. That one is now a documented precondition with an assert and a death test, rather than
  a silent NaN. Same sentinel, same class of interaction, caught by review rather than by symptom.
- *"Why did neither appear in the git history?"* Phase 1 landed as one commit, so the fixes are
  documented in the commit message and code comments rather than as a diff. If I were doing it again
  I would commit test-then-fix separately — the diff is more useful to a reviewer than the prose.

---

## Q11 — Why is `Mat4` column-major?

### Short Answer

To match GLSL and SPIR-V, so a `Mat4` can be memcpy'd straight into a Vulkan uniform buffer and used
by a shader with no transpose. The cost is that memory order no longer matches how a matrix is written
on paper, which is why every access site says `m[c][r]` explicitly rather than using a flat 16-element
array.

### Deep Answer

GLSL's `mat4` is column-major, and the std140/std430 layout rules store each column consecutively. If
the CPU side is row-major, every upload needs a transpose — a cost per matrix per frame, and more
importantly a *convention* that has to be remembered at every upload site. Forget it once and you get
geometry that is wrong in a way that looks almost right, among the more annoying graphics bugs to
chase. Matching the GPU's layout makes the upload a `memcpy` and removes the question.

Storage is `Scalar m[4][4]` with the documented convention `m[column][row]`, so `m[0]` is the first
column and translation lives in the **fourth column** rather than the fourth row. The associated
convention is column-vector: a transform applies as `M * v`, and "first A, then B" composes as
`B * A`. Both are pinned by tests, which matters because a convention that lives only in a comment
will drift.

The honest cost is readability: `m[1][2]` is column 1, row 2 — the transpose of how you read it off a
blackboard. The mitigation is to never use a flat `Scalar m[16]`, where `m[6]` would be genuinely
ambiguous, and to write `m[c][r]` everywhere so the two indices are always distinguishable by name.

There is one place the code deliberately works against its own layout: `invert` copies into
row-indexed locals, because Gauss-Jordan elimination is naturally row-oriented, and transposes back at
the end. The comment says exactly that — "We transpose in and out rather than contort the algorithm" —
which is the right call: two transposes of a 4×4 in a function that runs once per frame, versus an
elimination loop written backwards forever.

**Caveat worth stating unprompted:** the GPU half of this is currently a *design intent*, not a
demonstrated fact. There is no Vulkan code in the tree, and the SDK is not yet installable on this
machine because the Command Line Tools are too old. The decision is defensible on its merits; I have
not yet memcpy'd a matrix into a uniform buffer to prove it.

### Code Connection

- Storage, convention, and rationale: `include/geometry/mat4.hpp:11-23` — "so a Mat4 can be memcpy'd
  into a uniform buffer and consumed by a Vulkan shader with no transpose."
- Column-vector convention (`M * v`, `B * A` for "first A then B"): `:19-20`.
- Translation in the fourth *column*: `src/geometry/mat4.cpp:10-16`.
- `fromColumns` builds in storage order: `include/geometry/mat4.hpp:36-46`.
- Multiplication in terms of the convention: `(a*b)[c][r] = sum_k a[k][r] * b[c][k]` —
  `src/geometry/mat4.cpp:58-69`.
- Working against the layout where it pays: `src/geometry/mat4.cpp:109-125`, `:184-185`.
- Tests that pin the conventions: `tests/test_mat4.cpp:28` (`ColumnMajorStorageLayout`), `:211`
  (`FromColumnsMatchesStorageOrder`, which also checks that rows come back transposed and that raw
  storage is `m[column][row]`), `:43` (column/row accessors), `:99`
  (`MultiplicationAppliesRightmostFirst`), `:200` (matrix-vector product matches manual expansion).
- Vulkan status: `UNKNOWN — not yet implemented`; toolchain blocker in `CLAUDE.md`.

### Tradeoffs

| Choice | Gain | Cost |
|---|---|---|
| **Column-major (chosen)** | memcpy to GLSL/SPIR-V; no transpose; matches GLM's default too | memory order ≠ written order; `m[c][r]` discipline required |
| Row-major | reads like the blackboard; matches DirectX/HLSL default and C 2D-array intuition | transpose on every GPU upload, or `transpose=true` at every upload site |
| Flat `Scalar m[16]` | one index, trivially memcpy'd | `m[6]` is ambiguous — the bug class this project avoids by construction |
| Row-major CPU + transpose on upload | natural CPU code | one more place to forget |

### Follow-up Questions

- *"Does column-major change the math?"* No — purely storage. What changes is which index you write
  first and whether you compose left or right.
- *"Why column-vector rather than row-vector?"* It pairs with column-major to make `M * v` a
  column-dot, and matches GLSL and the standard mathematical convention. Row-vector plus row-major
  (the DirectX tradition) is equally self-consistent; mixing the two is what breaks.
- *"What about `transpose=true` in `glUniformMatrix4fv`?"* It exists precisely for this mismatch, but
  Vulkan has no equivalent — you write bytes into a buffer. So matching layout matters more with
  Vulkan than with OpenGL.
- *"How do you know the convention is right?"* Two tests assert the storage layout directly and one
  asserts composition order. `fromColumns` in particular had no coverage until recently, and it is the
  constructor that most directly encodes the convention — getting it wrong would silently transpose
  every matrix built through it.

---

## Q12 — Why Gauss-Jordan with partial pivoting instead of cofactor expansion?

### Short Answer

Stability, and because it is not a hot path. Cofactor expansion is faster for a fixed 4×4, but this
inverse brings rays into object space, where numerical error becomes a *wrong intersection* rather
than a slightly wrong pixel. Transforms are built once per frame at most, not once per ray, so I can
afford the more stable algorithm.

### Deep Answer

The two candidates differ in character. Cofactor (adjugate) expansion is a closed form: sixteen 3×3
minors, divided by the determinant. Branch-free, unrolls beautifully, the standard choice for a fixed
4×4 in a graphics library. But it computes the determinant as a sum of products of matrix entries, and
for a poorly conditioned matrix those products can catastrophically cancel — you subtract two nearly
equal large numbers and keep the noise.

Gauss-Jordan with partial pivoting has a different error profile. At each column it picks the row with
the largest entry in that column as the pivot, which bounds the multipliers used in elimination and
therefore bounds error growth. Without pivoting, dividing by a near-zero pivot amplifies whatever
rounding error is already present; that is the failure the pivot search exists to prevent.

The decision rule is the interesting part, and it is the same one used everywhere else in this
codebase: **how often does it run?** The reciprocal direction is precomputed per ray because it is
consumed per node. The AABB test is optimised because it runs millions of times. The matrix inverse is
*allowed to be slower* because it runs once per frame. Being able to say "I applied the same criterion
and it pointed the other way here" is stronger than "I optimise everything". There is a second
instance in the same file: `Vec3::operator/` keeps a true divide rather than multiplying by a
reciprocal, for the same reason.

And the consequence of getting it wrong is asymmetric. If this inverse were only a camera matrix, a
little error is a sub-pixel shift nobody sees. But it is used to transform rays into object space, and
there error moves the ray, which moves the intersection point, which can move it onto the wrong side
of a surface.

**Implementation details worth mentioning.** The algorithm operates on `[A | I]` in row-indexed local
copies — the transpose of the class's column-major storage — because elimination is naturally
row-oriented, then transposes back. And it fails *loudly*: a pivot below tolerance returns `false`
with the output untouched, rather than producing an inf/NaN matrix that propagates silently.

**The pivoting is actually *scaled* partial pivoting, not plain partial pivoting**, and the reason is
worth its own question — see [Q13](#q13--why-is-a-matrix-norm-relative-singularity-tolerance-wrong-for-a-homogeneous-transform).

The claim that cofactor expansion is faster is `UNVERIFIED` — no measurement exists here. It is the
conventional result and follows from operation counts, but I have not timed it.

### Code Connection

- The decision and its reasoning: `include/geometry/mat4.hpp:88-94` — "slower but more stable, and it
  is not a hot path... error becomes a wrong intersection rather than a slightly wrong pixel."
- The algorithm: `src/geometry/mat4.cpp:106-188`. Pivot search `:144-153`; rejection `:156`; row swap
  `:158-165`; normalisation `:167-171`; elimination `:173-181`.
- Transposing in and out rather than contorting the algorithm: `:107-111`, `:184-185`.
- Failure contract — returns `false`, leaves `out` untouched: `include/geometry/mat4.hpp:96-97`.
- The unsafe convenience wrapper, explicitly marked as such: `include/geometry/mat4.hpp:104-105`,
  `src/geometry/mat4.cpp:189-195`.
- A cofactor-style expansion *is* present for `determinant`, used only for reporting/validation and
  says so: `src/geometry/mat4.cpp:86-88`.
- The same "how often does it run?" criterion pointing the other way: `include/geometry/vec3.hpp:66-69`.
- Tests: `tests/test_mat4.cpp:129` (`M·M⁻¹ = I`), `:139` (round-trips points), `:146` (rejects
  singular, leaves `out` untouched). Ray round-trip through an inverse composite transform:
  `tests/test_ray.cpp:93-106`.

### Tradeoffs

| Method | Speed | Stability | Notes |
|---|---|---|---|
| Cofactor / adjugate | fastest for fixed 4×4, branch-free, SIMD-friendly | worst — cancellation in the determinant | the usual graphics-library choice |
| **Gauss-Jordan + scaled partial pivoting (chosen)** | slower; pivot search adds a divide and branches | good — multipliers bounded | ~4 passes × 4 rows × 4 cols |
| LU with partial pivoting | similar; better if you reuse the factorisation for many solves | same | overkill when you want the explicit inverse once |
| Full pivoting | slowest | best | unjustifiable for 4×4 transforms |
| Affine special case (invert 3×3 + negate translation) | much faster | fine | only valid for affine matrices; needs a separate path and a check |

That last row is the most interesting alternative: almost every matrix here *is* affine, and an affine
inverse is far cheaper. Not implemented, because `transformPoint` already supports projection matrices
and a special-cased path would need a validity check and a second codepath to test — for a function
that runs once per frame.

### Follow-up Questions

- *"What is the condition number and why does it matter?"* `κ(A) = ‖A‖·‖A⁻¹‖`, the factor by which
  relative input error can be amplified in the output. A near-singular matrix has huge κ, which is
  when the choice of algorithm shows.
- *"Why not just invert the affine part directly?"* Cheaper, and I'd consider it if it showed up in a
  profile, but it needs an affine-validity check and gives up projection-matrix support in the same
  function.
- *"Why does `inverse()` return identity on failure?"* It is the convenience wrapper: asserts in debug,
  degrades to identity in release. The header explicitly says to prefer `invert()` wherever failure is
  possible, which is honest API design — a function that cannot fail should not be the only one
  offered.
- *"How do you choose the pivot tolerance?"* That is the interesting part — Q13.

---

## Q13 — Why is a matrix-norm-relative singularity tolerance wrong for a homogeneous transform?

### Short Answer

Because a homogeneous transform always carries `m[3][3] == 1`, which dominates the matrix norm. Scale
by the largest entry and `scaling(1e-21)` looks singular — its global maximum is 1 while every pivot
that matters is `1e-21` — even though it is perfectly invertible. The fix is to judge each pivot
against the largest entry in **its own row**, which gives dimensionless ratios in `[0,1]`.

### Deep Answer

This one is a good question precisely because there are *three* wrong answers before the right one,
and each looks like a fix for the previous.

**Attempt 0 — no pivoting.** Divide by `lhs[col][col]` whatever it is. A near-zero pivot amplifies
existing rounding error without bound. Everyone agrees this is wrong.

**Attempt 1 — plain partial pivoting with an absolute cutoff**, e.g. `if (best <= 1e-20) return false`.
This is where the code started. Two problems. First, comparing raw magnitudes makes the pivot choice
depend on how each row happens to be scaled: multiply one equation through by `1e6` and it wins every
pivot contest without being any better conditioned. Second, the absolute cutoff is not scale-invariant
— a perfectly well-conditioned matrix that has simply been scaled down is rejected.

**Attempt 2 — scale the tolerance by the matrix norm.** The obvious fix, and the one a reviewer
suggested here. `tolerance = maxEntry(A) * epsilon * k`. It *is* scale-invariant under uniform scaling
of the whole matrix, so it looks correct.

It is wrong for exactly the matrices this project uses. A homogeneous 4×4 transform is not uniformly
scaled: the bottom-right entry is `1` by construction, always. So for `scaling(1e-21)`:

```
| 1e-21    0      0     0 |
|   0    1e-21    0     0 |
|   0      0    1e-21   0 |
|   0      0      0     1 |     <- m[3][3] == 1 dominates the norm
```

`maxEntry(A) == 1`, so the tolerance is roughly `epsilon`, and every pivot that matters is `1e-21`.
The matrix is rejected. It is perfectly invertible — its inverse is `scaling(1e21)`.

The mirror-image failure is just as bad: a matrix with entries near `1e20` could be wildly
near-singular and still clear a norm-scaled threshold, because the threshold has been inflated by the
large entries.

**Attempt 3 — scaled ("implicit") partial pivoting.** Record the largest absolute entry of *each row*
once, then judge each candidate pivot as a ratio against its own row's scale:

```cpp
const Scalar ratio = std::fabs(lhs[r][col]) / rowScale[r];
```

Those ratios are dimensionless and lie in `[0, 1]`, so the tolerance becomes a pure precision figure —
`epsilon * 8` — with no units to get wrong. Uniformly scaling a row changes nothing, because the
numerator and the row scale move together. And when rows are swapped the scales travel with them,
because a scale is a property of the row, not of the position.

An all-zero row is handled up front and rejected outright: it is rank-deficient at any scale, and it
also would make the ratio a division by zero.

**Why this is a good interview answer.** It is not "I used the textbook algorithm." It is: the obvious
fix was applied, a test rejected it, and the reason it failed is a *domain* fact — the structure of a
homogeneous transform — rather than a numerical-analysis fact. That is the kind of thing that only
shows up when you actually run the code against the matrices your application uses. The regression
test exists specifically to keep the wrong fix out: it asserts `scaling(1e-21)` is accepted and
round-trips to identity, and that genuinely rank-deficient matrices are still rejected at both
`1e-21` and `1e21` scale.

It also connects to a theme: **every geometric tolerance in this core is scale-invariant**, and in two
of the three cases the naive relative form was still wrong. The triangle degeneracy test (Q5) went
from absolute area to an aspect ratio; this one went from absolute to matrix-norm to per-row. Being
scale-invariant is necessary but not sufficient — it has to be invariant with respect to the *right*
thing.

### Code Connection

- The whole argument, written where the decision lives: `src/geometry/mat4.cpp:127-142` — including
  "a homogeneous transform always carries `m[3][3] == 1`, so `scaling(1e-21)` has a global maximum of
  1 while every pivot that matters is 1e-21. A matrix-norm threshold rejects it, even though it is
  perfectly invertible."
- The row-scale pass and the all-zero-row rejection: `:115-125`.
- The dimensionless tolerance: `:142` — `epsilon * 8`, with "Row-relative ratios are dimensionless and
  in [0, 1], so the tolerance is a pure precision figure with no units to get wrong" at `:140-141`.
- The ratio comparison and rejection: `:147-156`.
- Scales travelling with swapped rows: `:163-164`.
- Header summary of the contract: `include/geometry/mat4.hpp:96-101`.
- The regression test that rejected attempt 2: `tests/test_mat4.cpp:241-254`
  (`InvertAcceptsUniformlyTinyButInvertibleMatrices`) — accepts `scaling(1e-21)` and verifies
  `tiny * inv ≈ I`, then rejects `scaling(1e-21, 1e-21, 0)` and `scaling(1e21, 1e21, 0)`.
- The sibling scale-invariance decision: `include/geometry/triangle.hpp:52-63`.

### Tradeoffs

| Tolerance | Scale-invariant? | Fails on |
|---|---|---|
| Absolute (`1e-20`) | No | any uniformly small matrix |
| Matrix-norm-relative | Under uniform scaling of the whole matrix | **homogeneous transforms — `m[3][3] == 1` dominates** |
| **Per-row ratio (chosen)** | Yes, per row | nothing found so far; costs one divide per candidate pivot |
| Condition-number estimate | Yes | much more expensive; the right tool if you need to *report* conditioning rather than just reject |

The cost of the chosen form is a division per candidate pivot in the search loop, plus a 16-element
pass to compute the row scales. Acceptable precisely because `invert` is not a hot path (Q12) — the
same criterion again.

### Follow-up Questions

- *"Why `epsilon * 8` and not `epsilon`?"* A few ulps of headroom for the error already accumulated by
  the time a pivot is evaluated. The factor is a judgement, not a derivation, and I'd say so.
- *"Does scaled pivoting change which row gets chosen, or only whether you reject?"* Both — the ratio
  drives the `argmax`, so it selects a different pivot row than plain partial pivoting would whenever
  rows have different scales. That is the stability benefit; the rejection threshold is a side effect
  of having a dimensionless quantity to threshold.
- *"Is scaled partial pivoting always better?"* It is not free — a divide per candidate — and for
  matrices whose rows are already comparably scaled it picks the same pivots as the plain version.
  It earns its cost here because homogeneous transforms are systematically *not* comparably scaled.
- *"What would a condition-number check add?"* An actual measure of how much error to expect, rather
  than a binary accept/reject. Worth it if the answer fed a user-facing warning; overkill for
  rejecting singular transforms.
- *"How did you catch the matrix-norm version?"* A test written for the specific case:
  `scaling(1e-21)`. The reviewer proposed norm-relative, the test failed, and the failure named the
  reason.

---

## Q14 — Why do normals need the inverse transpose when directions don't?

### Short Answer

A normal is a covector, not a vector. Under non-uniform scale it transforms by the inverse transpose
of the upper-left 3×3, not by the matrix itself — scaling `x` by 2 *halves* the `x` component of a
surface normal rather than doubling it. Using the direction transform is the classic bug that leaves
normals no longer perpendicular to their surfaces.

### Deep Answer

The clean derivation is one line. A normal `n` is defined by being perpendicular to every tangent `t`
of the surface: `nᵀt = 0`. Under a transform `M`, tangents transform like directions: `t' = Mt`. We
need `n'` such that `n'ᵀt' = 0`:

```
Try n' = (M⁻¹)ᵀn:
n'ᵀ(Mt) = ((M⁻¹)ᵀn)ᵀ(Mt) = nᵀM⁻¹Mt = nᵀt = 0   ✓
```

So the inverse transpose is exactly what preserves perpendicularity. The intuition is worth having
too: stretch a surface along `x` and it becomes *flatter* relative to `x`, so its normal tilts *away*
from `x`. The normal moves opposite to the way a tangent does — which is what "covector" means.

**When does it matter?** Only for non-uniform transforms. For a pure rotation `M⁻¹ = Mᵀ`, so
`(M⁻¹)ᵀ = M` and the two agree. For a uniform scale they differ only by a scalar factor, which
vanishes on renormalisation. That is exactly why the bug survives: invisible until someone
non-uniformly scales an object, and then the lighting is subtly wrong in a way that is easy to blame
on the shader.

**Why directions do not need it.** A direction — a ray direction, a tangent, a velocity — is a genuine
vector: it transforms as `Mv` with `w = 0` so translation does not apply. The type system carries the
distinction: `Vec4` has a `w` component, points get `w = 1`, directions get `w = 0`, and keeping that
in the type is what makes `transformPoint` and `transformVector` separate, explicit operations rather
than one function with a silent convention.

**Two API details worth defending.** `transformNormalWithInverse` takes the **already-inverted** matrix
rather than inverting internally, so a caller transforming ten thousand normals pays for one inverse
rather than ten thousand — the name says so, which prevents the obvious misuse. And the transpose is
never materialised; the multiply is written transposed inline. I verified that expansion against the
storage convention rather than trusting the comment: with `i.m[c][r]` meaning column `c`, row `r`, the
element of `iᵀ` at row `r`, column `c` is `i.m[r][c]`, so `(iᵀn)_r = Σ_c i.m[r][c]·n_c` — exactly what
the three expressions compute.

**One thing to volunteer before being asked: the result is not unit length.** The inverse transpose
preserves perpendicularity, not magnitude. A unit normal through the inverse of a uniform 4× scale
comes back with length 1/4. The header says so and a test pins the number. Shading needs to
renormalise; a sidedness test does not.

**The test is the good part.** It does not just check the correct answer; it constructs a
tangent/normal pair, applies a non-uniform scale, then asserts that the inverse-transpose result stays
perpendicular to the transformed tangent **and** that the naive `transformVector` answer does not. So
the test fails if someone "simplifies" the function — the only kind of test that actually protects a
subtle correctness property.

### Code Connection

- Declaration with the full covector explanation and the "classic bug" warning:
  `include/geometry/mat4.hpp:126-140`; the not-unit-length warning at `:136-139`.
- Implementation, transpose never materialised: `src/geometry/mat4.cpp:213-220`.
- The contrast: `transformPoint` (`w = 1`, with perspective divide) at `src/geometry/mat4.cpp:197-206`;
  `transformVector` (`w = 0`) at `:208-211`; and the explicit note that length is *not* preserved under
  scaling and that this is deliberate — `include/geometry/mat4.hpp:121-123`.
- Why the distinction lives in the type: `include/geometry/vec4.hpp:10-14`.
- Tests: the perpendicularity-vs-naive comparison at `tests/test_mat4.cpp:158-182`; the length
  behaviour pinned at `:256-266` (`NormalTransformDoesNotPreserveLength`, asserting length 0.25 through
  a uniform 4× scale and that the direction is unchanged).
- Perspective-divide handling, including the affine fast path and the `w == 0` case:
  `include/geometry/mat4.hpp:110-118`, `src/geometry/mat4.cpp:201-204`, tested at
  `tests/test_mat4.cpp:183-198` and via `Mesh` at `tests/test_mesh.cpp:151-168`.

### Tradeoffs

| Approach | Consequence |
|---|---|
| **Inverse transpose (chosen)** | Correct for all invertible transforms; needs the inverse |
| Transform as a direction | Correct only for rotations and uniform scales; silently wrong otherwise |
| Rotation-only fast path | Cheap and correct when `M` is orthogonal, but needs a check or a promise |
| Cofactor (adjugate) instead of the inverse | Avoids the divide by the determinant; the scale factor washes out on renormalisation — a legitimate optimisation, not implemented |
| Invert inside the function | Convenient; catastrophic for a loop over many normals |

### Follow-up Questions

- *"When can you skip it?"* Pure rotations and rigid transforms: `(M⁻¹)ᵀ = M` for orthogonal `M`.
  Uniform scale differs only by a scalar that renormalisation removes.
- *"Does it preserve unit length?"* No, and that is documented and tested rather than left to be
  discovered. Neither does `transformVector` — and for ray directions that is deliberate (Q15).
- *"What about the translation column?"* Irrelevant — normals use only the upper-left 3×3, which is
  why the implementation reads nine elements and ignores the rest.
- *"Where would you use this here?"* Shading normals in the Vulkan renderer (Phase 7), and any
  object-space query needing a surface normal back in world space. Neither exists:
  `UNKNOWN — not yet implemented`.

---

## Q15 — Why aren't ray directions normalized?

### Short Answer

Two reasons. Normalising costs a `sqrt` per ray and the slab test works with any direction scale, so
it buys nothing. More importantly, when a ray is transformed into an object's local space the
transform may scale it — and renormalising there would silently change the meaning of `t`, so hit
distances found in local space would no longer match world space.

### Deep Answer

The second reason is the real one, and it only bites once you have an object hierarchy.

The standard pattern for instanced geometry is: keep the BVH in the object's local space, and for each
query transform the *ray* into that space with the object's inverse transform, rather than
transforming the geometry. Suppose the object has a 2× scale. World-space direction `d` becomes
local-space `2d`. A hit at world parameter `t` is at world point `o + t·d`; the same physical point in
local space is at `o' + t·(2d)` — **the same `t`**, because the direction scaled by exactly the factor
the space did.

Now renormalise the local direction. It becomes `d̂`, and the same physical point is at parameter `2t`.
Two spaces now report different `t` for the same intersection. Every comparison against the ray's
current closest hit is wrong, `tMax` no longer means what traversal thinks it means, and the bug shows
up as instanced objects occluding each other incorrectly — miles from the `normalize` call that caused
it.

Leaving the direction alone makes `t` invariant under the transform. The price is stated honestly in
the header: **`t` is measured in units of `|direction|`, not in world distance.** A caller that needs a
true metric distance must normalise up front and accept the `sqrt` — a deliberate decision pushed to
the boundary rather than silently made in the middle.

The first reason is the cheaper one: the slab test is scale-invariant, since scaling `d` by `k` scales
`invDir` by `1/k` and every `t` by `1/k` uniformly, so all the comparisons come out the same.
Normalising would be a `sqrt` per ray for no benefit to the test that dominates traversal.

The related decision in the same type: `[tMin, tMax]` is carried *inside* the `Ray` rather than passed
alongside it, so traversal can shrink `tMax` as closer hits are found. That is the single most
effective pruning mechanism in a BVH — once a hit at `t` is known, every subtree whose entry distance
exceeds `t` can be skipped outright. It only works if `t` means the same thing everywhere, which is
exactly what not renormalising guarantees.

There is a test that asserts the invariant directly:
`transformPoint(m, r.at(t)) == transformRay(m, r).at(t)` — transforming the hit point and evaluating
the transformed ray give the same answer. That equation is false the moment you renormalise.

### Code Connection

- The decision and both reasons: `include/geometry/ray.hpp:9-19` — "Renormalising there would silently
  change the meaning of t, so hit distances found in local space would no longer match world space",
  with the consequence for callers at `:17-19`.
- `tMin`/`tMax` in the ray, and the pruning argument: `:21-24`.
- `transformRay` — origin as a point, direction as a vector, range copied unchanged, no
  renormalisation: `:59-61`, reasoning restated at `:56-58`.
- `transformVector` explicitly documents that length is not preserved and that this is deliberate:
  `include/geometry/mat4.hpp:121-123`.
- The invariant test: `tests/test_ray.cpp:79-91`
  (`TransformKeepsHitParameterConsistentUnderScaling`). Range preservation `:71-77`; round-trip through
  an inverse composite transform `:93-106`; non-unit direction accepted `:35-40`.
- Scale-invariance of the slab test follows from `include/geometry/aabb.hpp:217-218` — `tNear` and
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
- *"Does this affect shading?"* Yes — a shader wanting a unit direction must normalise, and the surface
  normal must be transformed with the inverse transpose and renormalised (Q14). Neither is in Phase 1.
- *"Why keep `tMin`/`tMax` in the ray rather than as parameters?"* So traversal can shrink `tMax` in
  place as closer hits are found; the main pruning mechanism.
- *"Does the slab test really not care about scale?"* No — scaling `d` by `k` scales every `t` by
  `1/k`, and the comparisons are all between `t` values, so the outcome is identical. Only the reported
  `t` changes, consistently.
- *"Where is this actually used?"* Nowhere yet — object-space BVH traversal is Phase 2.
  `UNKNOWN — not yet implemented`. The test at `tests/test_ray.cpp:93-106` simulates the
  world→object→world path it is designed for.

---

## Q16 — Why is the mesh indexed rather than an array of triangles?

### Short Answer

A closed mesh shares each vertex among about six triangles, so indexed storage is roughly half the
bytes of an unpacked `vector<Triangle>` — and during a BVH build, where you stream over every vertex
repeatedly, bytes are the constraint. It is also the layout `vkCmdDrawIndexed` wants, so the render
path uploads both buffers with no repacking. The cost is one level of indirection per vertex fetch.

### Deep Answer

**The arithmetic, worth doing on a whiteboard.** Let `m` be triangles and `n` vertices. For a closed
manifold, Euler's relation gives roughly `n ≈ m/2` — each vertex is shared by about six triangles and
each triangle has three vertices, so vertices-per-triangle is `3/6 = 0.5`.

- Unpacked `vector<Triangle>`: 3 × `Vec3` = **36 bytes per triangle**.
- Indexed: positions `12n ≈ 6m` bytes, plus indices `3 × 4 = 12m` bytes → **≈ 18 bytes per triangle**.

About a **2× saving**. (The source comment says "roughly 12 bytes + 12 bytes of indices per triangle
versus 36" — 24 vs 36, a 1.5× saving. Its own "~6 triangles per vertex" premise actually gives
6 + 12 = 18. The conclusion holds; the comment's arithmetic is conservative. I checked rather than
quoting it.)

**Why the bytes matter — the cache argument.** BVH construction is not a single pass. A top-down build
touches the primitives at the root, then again at each child, then again at each grandchild —
`O(log n)` passes over progressively smaller subsets, but the upper levels stream the whole vertex set.
If that data fits in L2/L3, the build is bound by arithmetic; if not, every pass goes to DRAM. Halving
the footprint moves that threshold by a factor of two in mesh size. The source puts it as "on a
million-triangle mesh that difference decides whether the vertex data fits in cache during a build" —
the right *shape* of argument, though whether a particular mesh crosses a particular machine's
threshold needs measurement. `UNVERIFIED`: no benchmark exists.

**The GPU reason.** Vulkan's `vkCmdDrawIndexed` consumes precisely this pair of buffers. Storing
unpacked triangles would mean either repacking before upload or wasting 2× the VRAM and vertex-fetch
bandwidth. Design intent only for now — there is no Vulkan code in the tree.

**The cost, stated honestly in the source.** One level of indirection per vertex fetch:
`positions_[indices_[base + k]]` is a *dependent* load — you cannot start the second until the first
returns. Fine during a build, which is bandwidth-bound and streams predictably, but bad during
*traversal*, which is latency-bound and random-access. The header says so and names the fix: "Phase 2
can revisit by storing unpacked triangles in leaf order."

That is the genuinely interesting part of this answer. **The right layout for the build and the right
layout for the query are different**, and the usual production resolution is to keep both: an indexed
mesh as the authoritative representation, and a flattened, leaf-ordered, unpacked triangle array built
alongside the BVH for traversal. You pay 36 bytes per triangle again, but in a layout where a leaf's
triangles are contiguous, turning three dependent loads into one sequential stream. The tradeoff is
already flagged in the source rather than discovered later.

**A related design decision worth raising: the mesh caches two bounds, not one.** `bounds()` is tight —
over vertices actually *referenced* by the index buffer — and `vertexBounds()` is the conservative
extent of the whole vertex array. The reason the tight one is the default is stated in SAH terms: the
SAH normalises each child's surface area against its parent's, so a root inflated by unreferenced
vertices would shift every split decision and make benchmark numbers depend on how clean the input
file happens to be. `vertexBounds()` is still there because it is what you want for sizing a GPU vertex
buffer. Two different questions, two different answers, both named — rather than one bound that is
silently wrong for one of the uses.

**Two smaller decisions in the same class.** Indices are `uint32_t`, not `size_t`: half the width, and
4 billion vertices is beyond anything this project will load. And `Mesh::triangle` returns by value,
with the reasoning given — 36 bytes is cheaper to copy than to alias, and returning a value keeps the
`Mesh` immutable to callers.

**What makes the unchecked fetch safe.** The constructor validates the whole index buffer once —
multiple of 3, every index in range — and throws otherwise. That `O(n)` pass at the boundary buys an
unchecked inner loop for the lifetime of the mesh. One detail worth noticing: the check *widens the
index* rather than narrowing the vertex count, because narrowing would wrap for a vertex buffer larger
than 2³² and silently accept bad indices.

### Code Connection

- Storage and the full rationale, including the Vulkan reason and the admitted indirection cost:
  `include/geometry/mesh.hpp:19-27`.
- Tight `bounds()` with the SAH justification, and conservative `vertexBounds()`: `:77-90`; the two
  cached members at `:106-109`; both computed in `src/geometry/mesh.cpp:28-34`.
- The invariant that licenses unchecked indexing: `include/geometry/mesh.hpp:29-31`; enforcement at
  `src/geometry/mesh.cpp:10-24`; "O(n) here buys us an unchecked inner loop for the lifetime of the
  mesh" at `:14-15`; the widen-don't-narrow detail at `:17-19`.
- Materialising a triangle — three indirect loads: `include/geometry/mesh.hpp:69-75`; the
  return-by-value justification at `:67-68`.
- Buffers moved, not copied: `:36-37`, `src/geometry/mesh.cpp:9`.
- The unpacked counterpart and why both exist: `include/geometry/triangle.hpp:11-18` — "indexed storage
  is the compact form for holding a mesh, while the unpacked form is what an intersection routine
  wants, since chasing indices during traversal costs an extra dependent memory access per test."
- Tests: the three-way bounds assertion (tight excludes the orphan, conservative includes it,
  conservative contains tight) at `tests/test_mesh.cpp:101-116`; validation rejections at `:84`, `:88`,
  `:93`.

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
  nowhere near 2³². It halves the index buffer, which is two thirds of the indexed footprint. The
  validation loop widens rather than narrows so the choice stays safe on a 64-bit host.
- *"Won't the indirection hurt traversal?"* Yes, and the source says so. The fix is a leaf-ordered
  unpacked triangle array built alongside the BVH — build-optimal and query-optimal layouts are
  different, and you are allowed to have both.
- *"Why two bounds instead of one?"* Because "the extent of the geometry" and "the extent of the vertex
  buffer" are different questions, and the BVH needs the first while the renderer needs the second.
  Using the conservative one as the root would shift every SAH split decision based on file hygiene.
- *"How would you measure whether the layout matters?"* Build both, hold the mesh and ray set fixed,
  compare build time, query time and cache-miss counters. Phase 9 defines the methodology. Nothing is
  measured yet.
- *"Why validate eagerly instead of using `.at()`?"* `.at()` is a bounds check per access forever;
  validation is one pass ever. The invariant then holds for the object's lifetime because nothing
  public can mutate the index buffer.
- *"What about vertex normals and UVs?"* Not present — positions only. Adding them raises an AoS-vs-SoA
  question, since the BVH build wants positions alone and the renderer wants all attributes
  interleaved. `UNKNOWN — not yet implemented`.

---

## Q17 — When do you throw and when do you assert?

### Short Answer

Throw at the trust boundary; assert everywhere else. The `Mesh` constructor throws, because that is
where untrusted file data enters the system. Everything else — an out-of-range triangle index, the
centroid of an empty box, normalising a zero vector — is a programming error, so it asserts. And
because `-DNDEBUG` strips every assert, the project requires both a Debug and a Release test run
before a milestone is done.

### Deep Answer

The distinction is about *who made the mistake*.

A malformed index buffer is not a bug in my code — it is bad input, and it will happen, because meshes
come from files written by other tools. The caller needs to find out and decide what to do. So the
constructor throws `std::invalid_argument`, with a message naming the offending index and its slot so
the failure is diagnosable from a log. The header says it explicitly: errors are reported "explicitly
rather than by assertion because this is the boundary where untrusted file data enters the system."

An out-of-range *triangle* index is different. There is no file that can cause it; it means a loop
bound is wrong in code I wrote. Recovering is meaningless — the right behaviour is to stop as close to
the mistake as possible. So it asserts. The comment even names where the bug will come from: "Phase 2
will call this from a loop bounded by a node's primitive range, which is exactly where an off-by-one
would land."

Notice that these two live in the same class, one line apart in spirit: the constructor validates the
*contents* of the index buffer as data, and the accessor checks the caller's *index* as a contract.

**The part that makes the policy real rather than decorative** is that `NDEBUG` deletes every assert,
so a Release-only test run exercises none of them. This project therefore requires both configurations
to pass, and the CMake configuration summary prints whether asserts are live so you cannot be confused
about which build you are looking at. Verified status: Release 122/122, Debug 128/128 — the difference
is the death tests, which only exist when asserts are compiled in.

**And the death-test file is careful about its own validity**, which is the detail I would actually
point at. Under `NDEBUG` it compiles down to a single `GTEST_SKIP` with an explanatory message, because
if the tests were left unguarded they would "pass" in Release by doing nothing — worse than not having
them, since a green suite would be reporting coverage that does not exist.

**There is a third tier, and it is the one people forget.** A precondition is a contract, not a
memory-safety guarantee. When `NDEBUG` removes the assert, the operation still executes — so it must
still be *defined*, even though its result is meaningless. `normalize(Vec3(0))` is the case here: in
Debug the assert fires; in Release it divides by zero, which used to be undefined behaviour and is now
guaranteed to produce NaN because `Vec3::operator/` guards it. The test file has a section for exactly
this, with tests that run in Release and assert "this is a NaN, and the point is that it is a NaN and
not UB."

So the full policy is three layers: **throw** for bad data, **assert** for bad calls, and **define the
behaviour anyway** for when the assert is gone.

### Code Connection

- The policy as stated: `CLAUDE.md` (Build and test) and `tests/test_preconditions.cpp:1-11`.
- Throwing at the boundary: `include/geometry/mesh.hpp:39-41`; implementation
  `src/geometry/mesh.cpp:10-24`, with diagnosable messages at `:11-12` and `:20-22`.
- Asserting on a caller-supplied index, with the Phase 2 justification:
  `include/geometry/mesh.hpp:53-60`, `:70`.
- Other asserted preconditions: `AABB::centroid` `include/geometry/aabb.hpp:72-80`, `AABB::offset`
  `:138-139`, `normalize` `include/geometry/vec3.hpp:101-110`, `Vec3::operator[]` `:33`, `:37`.
- The third tier — defined behaviour when the assert is gone: `include/geometry/vec3.hpp:103-107`
  ("In a release build the assert is gone, so the division must still be well-defined"), guaranteed by
  `operator/` at `:70-76`.
- Both configurations required: `CLAUDE.md`, `README.md` (Building), and the assert status in the
  configuration summary `CMakeLists.txt:150-154`.
- The death-test file's self-guard and its reasoning: `tests/test_preconditions.cpp:8-11`, `:23-28`.
- The 7 death tests: `:32-68`. The 3 release-safety tests: `:78-105`.
- Throwing half tested at `tests/test_mesh.cpp:84`, `:88`, `:93`, `:97`.

### Tradeoffs

| Policy | Consequence |
|---|---|
| **Throw at the boundary, assert inside (chosen)** | Bad data is recoverable and diagnosable; bad calls fail fast; no per-call cost in Release |
| Assert everywhere | Assertions vanish in Release — exactly when malformed file data arrives |
| Throw everywhere | A `try`/`catch` cost and an exception-safety obligation on paths that can only fail through a bug |
| Error codes / `expected` | Explicit, but forces every accessor to return a wrapper; heavy for a hot primitive like `triangle(i)` |
| Bounds-checked accessors (`.at()`) | A check per access forever, versus one validation pass ever |
| Silent clamping | The worst option — turns a detectable bug into wrong geometry |

### Follow-up Questions

- *"Why not exceptions everywhere for consistency?"* Because the two cases have different audiences. A
  caller can meaningfully handle "this file is malformed"; nobody can meaningfully handle "your loop
  bound is wrong". And `triangle(i)` is on the path Phase 2 will call per primitive per node.
- *"Isn't a death test slow?"* Yes — each one forks. That is acceptable for 7 of them in a Debug-only
  run, and it is the only way to test that a process aborts.
- *"What if someone runs only Release?"* Then the assert half of the policy is untested, which is
  precisely why the project requires both and why the configuration summary prints the assert state.
  The skip-stub also prints a message telling you to run Debug.
- *"You said a precondition isn't a memory-safety guarantee — what is?"* Sanitizers, and here that is a
  gap: UBSan is clean in both configurations, but ASan cannot run on this machine at all (a
  hello-world built with `-fsanitize=address` exits 139, verified in isolation). So use-after-free and
  buffer-overflow classes are currently unchecked. The core's exposure is small — it heap-allocates
  only through `std::vector` and has no raw owning pointers — but it is an honest gap, and it will
  matter more once Vulkan resource handles arrive.

---

## Q18 — Is this really C++20?

### Short Answer

Not fully, and the build says so out loud. The installed Apple clang 11.0.3 has no C++20 standard
library, and CMake silently falls back to the draft `-std=c++2a` while still reporting "20" — so there
is a configure-time probe that compiles `<concepts>` and `<span>` and warns when they are missing. The
geometry core uses no C++20 library feature, so it builds and passes, but the project cannot honestly
claim C++20 yet.

### Deep Answer

`CMAKE_CXX_STANDARD 20` with `CMAKE_CXX_STANDARD_REQUIRED ON` reads like a guarantee and is not one.
For a compiler that only knows the pre-release draft flag, CMake emits `-std=c++2a` and reports "20"
anyway. You get a build that looks conformant and is missing most of the C++20 standard library —
`<concepts>`, `<span>`, `<ranges>`, `<numbers>` are all absent on this toolchain.

Rather than let that sit silently, the build probes: `check_cxx_source_compiles` on a translation unit
that includes `<concepts>` and `<span>` and instantiates a `std::floating_point` constrained template.
On failure it emits a `message(WARNING)` naming the compiler and pointing at the fix, and the
configuration summary prints `C++ standard ....... 20 (DRAFT -- no C++20 library)` instead of a bare
"20". The README carries the same caveat for anyone who never runs CMake.

**Why it does not block Phase 1.** Nothing in `include/geometry/` or `src/geometry/` uses a C++20
library facility. The features actually relied on — `constexpr` member functions, default member
initialisers, `inline constexpr` namespace-scope variables, hidden friend operators — are C++17 or
earlier. So the accurate statement is: *this is C++17-compatible code compiled under a draft C++20
flag.*

**Why it will block later phases.** Homebrew refuses to compile anything on this configuration ("Your
Command Line Tools are too outdated"), which already blocked `brew install llvm` and will block GLFW,
the Vulkan SDK and ImGui the same way. The fix needs a GUI installer and `sudo`, so it has to be done
by hand.

**The transferable point** is not the version number. It is that a build system reporting "C++20" was
*wrong*, and the response was to add a check that makes the discrepancy visible rather than inheriting
it. Same instinct as the two bugs in Q10: assume the thing that looks fine might be silently lying,
and add the check.

### Code Connection

- Standard requested: `CMakeLists.txt:12-14`, with `CMAKE_CXX_EXTENSIONS OFF` so it is `-std=c++20`
  rather than `-std=gnu++20`.
- Why a probe is needed, stated in the build file: `:27-31`.
- The probe: `:32-40`. The warning: `:42-48`. The honest summary line: `:145`.
- Platform and toolchain analysis: `CLAUDE.md`, "Toolchain status — action needed"; user-facing caveat
  in `README.md`.
- Other deliberate build decisions worth mentioning in the same breath: Release default because "an
  unset `CMAKE_BUILD_TYPE` silently gives no optimisation at all" (`CMakeLists.txt:19-25`); the strict
  warning set with per-flag justifications (`:63-75`); the `-ffast-math` ban (`:83-86`); the ODR-safe
  placement of `BVH_CONSERVATIVE_RAY_BOX` (`:88-95`); sanitizers behind an option (`:97-102`); the
  layering rule stated in the build (`:107-109`); and the assert-state line in the summary
  (`:150-154`).

### Tradeoffs

| Response to the toolchain gap | Consequence |
|---|---|
| **Probe and warn (chosen)** | Honest; build proceeds; the gap is visible at every configure |
| Say nothing | The project claims C++20 falsely; the first `#include <span>` fails mysteriously |
| Hard-fail the configure | Would block Phase 1 for no technical reason — the core does not need C++20 |
| Downgrade to `CMAKE_CXX_STANDARD 17` | Honest, but discards the intent and would need reverting |
| Install a newer toolchain first | The right fix; needs sudo and a GUI installer, so it cannot be scripted |

### Follow-up Questions

- *"What would you use C++20 for here?"* `std::span` for non-owning views over index and position
  buffers; `concepts` to constrain a `Scalar` template if precision became configurable; `<numbers>`
  for `pi` instead of the literal at `include/geometry/scalar.hpp:30`. None is load-bearing.
- *"Why not just use C++17?"* The plan specifies C++20 and later phases may want `std::span` at buffer
  boundaries. Recording the gap costs nothing and keeps the intent.
- *"Does the draft flag change code generation?"* Not for anything this code uses — the missing pieces
  are library headers, not language semantics the core depends on.
- *"How should an interviewer read this?"* Favourably, if framed as "the build was reporting something
  untrue and I made it stop" rather than "my compiler is old".

---

## Q19 — How do you know any of this is correct?

### Short Answer

122 tests pass in Release and 128 in Debug — the difference is death tests, which only exist when
asserts are compiled in. Clean under `-Werror` with a strict warning set, and clean under UBSan in
both configurations. The interesting tests are not known-answer checks: they are property tests,
randomised invariants over a seeded generator, and degeneracy tests named after the failure they
prevent. ASan cannot run on this machine, so there is no memory-safety sanitizer coverage — that is a
real gap and I would rather say so.

### Deep Answer

The suite has six distinguishable layers, and naming them is more useful in an interview than the
count.

**1. Known-answer tests.** A 1×2×3 box has surface area 22 and volume 6; a ray from `z = -5` hits the
unit box at `t = 4`. Fast to write, catch gross errors, prove nothing subtle.

**2. Property tests** — mathematical relationships rather than constants. Cross product orthogonal to
both inputs; rotation preserves length; rotation about an axis leaves that axis fixed; `M·M⁻¹ = I`; a
world→object→world round-trip recovers the original ray; a matrix-vector product matches its manual
expansion. These survive refactoring in a way magic constants do not.

**3. Randomised property tests over a seeded generator** — the strongest layer, and the one I would
lead with. The generator is seeded with a fixed constant, because "a randomised test that cannot be
replayed is a flaky test", and it deliberately makes directions *sometimes exactly axis-aligned* so the
infinite-reciprocal paths stay in the sample. Five invariants: any sampled point inside the box implies
a reported hit; the widening never removes a hit; `tEnter` stays within the ray range and finite; an
empty box is never hit from any direction; a ray originating inside always hits with `tEnter == tMin`.

Three design details make it a real suite rather than a token one. The soundness test is deliberately
**one-directional** — sampling along a ray can prove a hit but never prove a miss, so there are no
false alarms. It **guards against silently proving nothing** by requiring a minimum number of proven
hits, which catches the failure mode where a generator change makes every ray miss and the test passes
vacuously. And the widening comparison **logs rather than asserts** the disagreement count, because if
a future change makes the widening start mattering, that is information rather than a failure. It also
keeps a deliberately separate unwidened reference implementation so the two can be compared at all.

That file also names its future role: the oracle for Phase 2, since BVH traversal is correct only if
it reports the same hits as testing every primitive directly.

**4. Death tests for preconditions**, covering the asserting half of the error policy (Q17): 7 of them,
plus 3 release-safety tests that run the other way and assert that the operation is still *defined*
when the assert is gone. The file guards itself under `NDEBUG` so it cannot pass vacuously in Release.

**5. Degeneracy and hazard tests** named after the failure they prevent:
`RayLyingExactlyOnSlabBoundaryIsHandled`, `EmptyBoxIsNeverHit`, `FlatBoxIsStillHittable`,
`OffsetOnDegenerateAxisDoesNotDivideByZero`, `CollinearPointsGiveDegeneratePlaneNotNaN`,
`RotationAboutDegenerateAxisIsIdentity`, `DegeneracyMatchesAcrossUniformScaling`. This layer found the
slab-swap bug, before any BVH existed to exhibit the symptom — the argument for testing representation
edge cases at the primitive level rather than waiting for the system to behave oddly.

**6. Convention-pinning tests**: `ColumnMajorStorageLayout`, `FromColumnsMatchesStorageOrder`,
`MultiplicationAppliesRightmostFirst`, `CrossProductIsRightHanded`,
`NormalFollowsCounterClockwiseWinding`, `DirectionIsNotRequiredToBeNormalized`. A convention that lives
only in a comment will drift; one with a test will not.

**Four tests are worth reading aloud.** `NormalTransformUnderNonUniformScale` asserts both that the
inverse-transpose result stays perpendicular *and* that the naive answer does not — so it fails if
someone "simplifies" the function. `TransformKeepsHitParameterConsistentUnderScaling` asserts
`transformPoint(m, r.at(t)) == transformRay(m, r).at(t)`, exactly what breaks if `transformRay`
renormalised. `BoundsAreTightAndVertexBoundsAreConservative` makes three assertions that together pin
the whole two-bounds design. And `InvertAcceptsUniformlyTinyButInvertibleMatrices` exists specifically
to reject a plausible-but-wrong fix to the pivot tolerance (Q13) — a test whose job is to keep a bad
patch out, which is a category worth being able to name.

**Beyond tests.** A strict warning set with per-flag justifications in the build file (`-Wconversion`
because silent narrowing changes results; `-Wdouble-promotion` because an accidental promotion in a hot
loop undoes the float decision; `-Wshadow` because a shadowed variable in geometry code is nearly always
a bug), `-Werror` behind an option and currently clean, and sanitizers behind another.

**What I would volunteer without being asked.** ASan does not run here: a hello-world built with
`-fsanitize=address` exits 139, verified in isolation, so the failure is environmental rather than a
defect. UBSan runs clean in both configurations and found one of the two real bugs. But use-after-free,
buffer-overflow and leak classes are currently unchecked. The core's exposure is small — it
heap-allocates only through `std::vector` inside `Mesh` and has no raw owning pointers — but "no ASan
coverage" is an honest limitation, and it will matter much more once Vulkan resource handles arrive in
Phase 7.

**Other gaps.** No test for `transformPoint` on a point projecting to exactly `w == 0`. No test for the
ODR hazard the `BVH_CONSERVATIVE_RAY_BOX` macro placement guards against, which would need a two-TU
build with mismatched definitions. And **no benchmarks at all** — every performance statement in this
project is an argument from layout or complexity, not a measurement.

### Code Connection

- Status at `935c354`: Release 122/122, Debug 128/128, `-Werror` clean, UBSan clean in both, ASan
  unavailable. 129 `TEST()` macros exist; `tests/test_preconditions.cpp` is conditionally compiled —
  7 death tests only when `NDEBUG` is undefined (`:30-70`), a single skip-stub only when it is defined
  (`:23-28`), and 3 release-safety tests always (`:78-105`) — so Release registers 118 + 1 + 3 = 122
  and Debug registers 118 + 7 + 3 = 128.
- Test wiring, GoogleTest pinned to v1.15.2 and why: `tests/CMakeLists.txt:3-22`; per-`TEST()`
  registration at `:51`.
- Randomised suite: `tests/test_aabb_property.cpp` — reproducibility rationale `:9-10`; separate
  unwidened reference `:24-44`; axis-aligned direction generation `:64-73`; one-directional soundness
  `:81-111`; the vacuity guard `:109-110`; widening direction `:113-143`; log-don't-assert `:137-142`;
  `tEnter` range `:145-157`; empty box from any direction `:159-169`; origin inside `:171-186`;
  Phase 2 oracle role `:6-7`.
- Death tests and the self-guard: `tests/test_preconditions.cpp:8-11`, `:23-28`, `:32-68`, `:78-105`.
- Property tests: `tests/test_vec3.cpp:84`; `tests/test_mat4.cpp:80`, `:92`, `:129`, `:200`;
  `tests/test_ray.cpp:93`.
- Degeneracy tests: `tests/test_aabb.cpp:142`, `:207`, `:235`, `:244`; `tests/test_plane.cpp:30`, `:36`;
  `tests/test_mat4.cpp:86`, `:146`; `tests/test_triangle.cpp:82`, `:91`, `:100`, `:110`, `:125`, `:135`;
  `tests/test_mesh.cpp:84`, `:88`, `:93`, `:119`.
- Convention tests: `tests/test_mat4.cpp:28`, `:99`, `:211`; `tests/test_vec3.cpp:67`;
  `tests/test_triangle.cpp:20`; `tests/test_ray.cpp:35`.
- The four exemplary tests: `tests/test_mat4.cpp:158-182`, `tests/test_ray.cpp:79-91`,
  `tests/test_mesh.cpp:101-116`, `tests/test_mat4.cpp:241-254`.
- Comparison policy: `include/geometry/vec3.hpp:46-48`; `nearlyEqual` at
  `include/geometry/scalar.hpp:67-74`.
- Warning set with justifications: `CMakeLists.txt:63-75`. Sanitizers: `:97-102`.
- Benchmark rules: `benchmarks/results/README.md:3-5`, methodology at `:48-62`.

### Tradeoffs

| Approach | What it catches | What it misses |
|---|---|---|
| Known-answer tests | gross errors | anything subtle; brittle under refactoring |
| Property tests | relationship violations | wrong constants that still satisfy the relationship |
| Randomised property tests | sign, swap and ordering mistakes across a large input space | needs an oracle or a one-directional invariant; a bad generator can prove nothing |
| Death tests | violated preconditions | only in builds with asserts live — hence the two-configuration rule |
| Degeneracy tests | representation edge cases — found the slab bug here | value-range issues |
| UBSan | undefined behaviour — found the `1/0` bug here | only on paths actually executed |
| ASan | memory safety | **unavailable on this machine** |
| Differential testing vs. a reference | almost everything | needs a reference; project rules forbid depending on one for core algorithms |

### Follow-up Questions

- *"What would you add next?"* A randomised ray-vs-brute-force differential test as soon as traversal
  exists — the strongest possible check on a BVH, since brute force is the oracle. The property file is
  already written with that role in mind.
- *"How do you test performance?"* Not at all yet. The methodology is written down — Release-only,
  median of ≥5 runs, discard warmup, fixed seeded ray sets, one variable at a time, record the commit
  — but no run exists, so every performance claim in this project is an argument.
- *"Is 129 tests a lot?"* The count is not the point; coverage of *representation* edge cases is. Two
  real bugs were found by layer 5, neither of which a coverage metric would have demanded.
- *"What is your line coverage?"* Not measured. I would rather name the specific untested properties
  than quote a number that would be near 100% and still miss every gap above.
- *"Why can't you run ASan?"* Environmental — a hello-world with `-fsanitize=address` exits 139 on this
  machine, so it is not this code. Likely the same stale Command Line Tools that block C++20 and
  Homebrew. It is on the list with the toolchain fix.

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
| Does the `gamma(3)` widening actually prevent a crack in practice? | **Open.** Argued from PBRT, direction-of-effect tested, but no case is known where it changes the answer. Made switchable so Phase 9 can settle it — Q4. |
| Why Vulkan compute? What stays on the CPU? | `UNKNOWN — not yet implemented` (Phase 8) |
| What causes CPU/GPU synchronization stalls? | `UNKNOWN — not yet implemented` (Phase 8) |
| Vulkan instance/device/queues/descriptors/pipelines | `UNKNOWN — not yet implemented` (Phase 7) |
| How would you parallelize BVH construction? | `UNKNOWN — not yet implemented` (Phase 8/9) |
| How would you optimize the memory layout? | Partially answerable now — Q1 (node width), Q16 (mesh layout, and the build-vs-query layout split). The BVH node layout itself is `UNKNOWN — not yet implemented`. |
| How would this scale to millions of triangles? | Arguable from complexity and layout; **no measurement exists**. Any specific number would be fabricated. |
| Any benchmark result whatsoever | `UNVERIFIED` — `benchmarks/results/` contains only `README.md`, a format and methodology specification. |
| CPU vs GPU speedup on this machine | `UNKNOWN — not yet implemented`. Hardware context in `CLAUDE.md`: Intel Core i5-8257U with integrated Iris Plus, Vulkan only via MoltenVK, so a modest or even negative GPU result is the expected honest outcome. |
| OBJ loading | `UNKNOWN — not yet implemented` |
| The interactive explorer, parameter controls, visualization | `UNKNOWN — not yet implemented` (Phases 4–5, 10) |
