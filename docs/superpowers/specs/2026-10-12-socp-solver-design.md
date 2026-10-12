# SOCP solver (`optim`) - design

Status: approved in conversation 2026-10-12, sub-project 1 of 3 of the tower-catch landing guidance
(1 SOCP solver -> 2 launch tower and catch arms -> 3 6-DOF successive-convexification landing guidance).

## Purpose

A general, dependency-free second-order cone program (SOCP) solver for the C++ core. Its first user is the
6-DOF successive-convexification (SCvx) powered-descent guidance of sub-project 3, which re-solves a
trajectory problem in flight. It is deliberately not landing-specific: ship landings (Earth/Moon) and
rendezvous planning (M9) may use it later.

Why our own solver: ECOS is GPLv3; Clarabel needs a Rust toolchain for C++ use; the project must stay free
and self-contained. CVXPY/Clarabel are used only offline, as reference tools for test fixtures.

## Placement

- New core module `optim`: `core/include/starsim/optim/`, `core/src/optim/`.
- Depends only on the C++ standard library (no `math`, `gnc`, ... dependency). `gnc` may use it.
- Tests in `core/tests/test_optim_*.cpp`, fixtures in `tests/validation/socp/`.

## Problem form

Standard conic form (as in ECOS / Clarabel):

```
minimise    c^T x
subject to  A x = b
            G x + s = h,   s in K
K = R+^l  x  Q^{q_1}  x ... x  Q^{q_m}
Q^q = { s in R^q : ||(s_1, ..., s_{q-1})||_2 <= s_0 }      (Lorentz cone)
```

- `x in R^n`, `A in R^{p x n}` (may be empty), `G in R^{m x n}`; the rows of `G`/`h` are ordered as the cone
  list: first the `l` non-negative rows, then each second-order cone in turn.
- Linear objective only. Quadratic penalties (e.g. the SCvx trust region) are written as epigraph
  constraints with a second-order cone.
- Matrices are given as triplets (row, col, value; duplicates are summed) and stored internally as
  compressed sparse column (CSC).

### `SocpBuilder`

Helper that assembles a `SocpProblem` readably, so sub-project 3 never computes indices by hand:

- `add_variables(count, name)` -> a `VariableBlock` (offset, size, name for diagnostics).
- `add_equality(terms, rhs)`: sum of (coefficient, variable index) == rhs.
- `add_nonnegative(terms, constant)`: affine expression >= 0.
- `add_second_order_cone(rows)`: each row an affine expression; rows[0] >= ||rows[1..]||.
- `set_objective(terms)`.
- `build()` -> `SocpProblem` with the inequality rows reordered into the cone order above.

Affine expressions are a small `AffineExpr` (sparse list of (index, coefficient) plus a constant).

## Algorithm

Primal-dual interior point method on the homogeneous self-dual embedding (HSDE), as in ECOS
(Domahidi, Chu, Boyd 2013) and Vandenberghe's CVXOPT notes:

- **Embedding:** variables `(x, y, z, s, tau, kappa)`; infeasibility and unboundedness certificates come
  from `tau -> 0`.
- **Steps:** Mehrotra predictor-corrector: an affine (no centring) direction, step length to the cone
  boundary, centring parameter `sigma = (1 - alpha_aff)^3`, then the combined direction with the
  second-order correction term. Step length scaled by 0.99 of the distance to the boundary.
- **Scaling:** Nesterov-Todd scaling point per cone. Non-negative orthant: diagonal. Second-order cones:
  dense `W` block per cone (our cones have dimension 3-14); no expanded sparse representation is needed.
- **Equilibration:** Ruiz scaling of `[A; G]` (column and row norms, ~10 passes; rows of one cone share a
  factor so the cone is preserved) before solving; the solution is unscaled afterwards.
- **Initial point:** as in ECOS/CVXOPT (two least-squares solves, shifted into the cone interior).
- **Termination** (relative, on the unscaled problem): primal residual, dual residual and duality gap
  below `tolerance` (default 1e-8, settable); infeasibility certificate tolerance 1e-8;
  `max_iterations` default 50.
- **Result:** `SocpResult { status, x, y, z, s, objective, iterations, primal_residual, dual_residual, gap }`
  with `status in { Optimal, PrimalInfeasible, DualInfeasible, MaxIterations, NumericalError }`.
  `NumericalError` when a factorisation fails after regularisation or a step length collapses (< 1e-10).

## Linear algebra

- **KKT system** per iteration (scaled):
  ```
  [ 0   A^T   G^T   ] [dx]   [r_x]
  [ A   0     0     ] [dy] = [r_y]
  [ G   0   -W^T W  ] [dz]   [r_z]
  ```
  quasi-definite after static regularisation (`+delta` on the (1,1) block, `-delta` on the (2,2) block,
  delta ~ 1e-8 relative), solved by LDL^T without pivoting, followed by iterative refinement against the
  unregularised matrix (up to 3 steps). The same factorisation serves the affine and the combined
  direction, and the two solves for the HSDE `tau` direction.
- **Sparse LDL^T (own):** symbolic phase once per problem structure (elimination tree, column counts,
  pattern of L); numeric phase each iteration (up-looking, as in Davis' LDL). The pattern of the `W^T W`
  blocks is the dense cone block, fixed across iterations.
- **Ordering:** approximate minimum degree (AMD, own implementation after Amestoy/Davis/Duff), computed
  once in the symbolic phase.
- No heap allocation inside the iteration loop: all work arrays are sized in the setup.

## Determinism and performance

- Single-threaded, no randomness, no data-dependent allocation order: identical input gives bit-identical
  output (the flight simulation and its tests depend on reproducibility).
- Target: a problem with ~1000 variables and ~2000 constraint rows (size of sub-project 3) in < 50 ms and
  < 30 iterations (Release build, user's machine). A benchmark test reports the time; it fails only on
  iteration count, not on wall time (CI-independent).

## Files

| File | Responsibility |
|------|----------------|
| `optim/sparse_matrix.hpp/.cpp` | triplet -> CSC, transpose, matrix-vector products |
| `optim/amd.hpp/.cpp` | approximate minimum degree ordering |
| `optim/ldl.hpp/.cpp` | sparse LDL^T: symbolic + numeric factorisation, solve |
| `optim/cones.hpp/.cpp` | cone layout, NT scaling, cone products, max step to the boundary |
| `optim/kkt_system.hpp/.cpp` | assembles the KKT matrix, regularisation, iterative refinement |
| `optim/equilibration.hpp/.cpp` | Ruiz scaling / unscaling |
| `optim/socp_problem.hpp` | `SocpProblem`, `SocpSettings`, `SocpResult`, `SocpStatus` |
| `optim/socp_solver.hpp/.cpp` | HSDE predictor-corrector loop |
| `optim/socp_builder.hpp/.cpp` | `AffineExpr`, `VariableBlock`, `SocpBuilder` |

Each file stays small; the solver loop only orchestrates the pieces.

## Testing

Unit tests (Catch2, tag `[optim]`):
- Sparse matrix: triplet assembly with duplicates, transpose, products against dense computation.
- LDL^T: random sparse quasi-definite matrices (fixed seed) against a dense solve; residual <= 1e-12 relative.
- AMD: result is a permutation; fill of L with AMD <= fill without ordering on a banded trajectory-like
  pattern and on a 2-D grid Laplacian.
- Cones: NT scaling satisfies `W z = W^{-T} s` (= lambda); max step keeps the point in the cone.
- Builder: rows end up in cone order; expressions with constants land in `h`.

Analytic cases:
- LP with a known vertex optimum.
- Minimum Euclidean distance from a point to a plane (SOC): closed form.
- Minimum-norm point in a ball intersected with a half-space.
- Primal infeasible (contradictory bounds) -> `PrimalInfeasible`; unbounded LP -> `DualInfeasible`.

Reference fixtures (tag `[optim][validation]`):
- `tools/validation/gen_socp_fixtures.py` generates problems, solves them with CVXPY + Clarabel and writes
  `tests/validation/socp/*.json` (problem data + reference x and objective). Committed; regenerated only
  when the generator changes. CVXPY and Clarabel are Apache-2.0, recorded as tools (not runtime data).
- Cases: random feasible LPs and SOCPs (fixed seeds, sizes 10-200), a portfolio-style SOCP, and a
  G-FOLD-style 3-DOF landing problem (lossless convexification, ~40 nodes) of the size of sub-project 3.
- Tolerance: objective relative error <= 1e-6; solution `||x - x_ref|| / max(1, ||x_ref||) <= 1e-5`
  (for problems with a unique optimum).
- Determinism: solving the same problem twice gives identical results.
- Benchmark: the landing problem converges in < 30 iterations; time printed with `STARSIM_VERBOSE=1`.

## Out of scope

Quadratic objective as a native form; exponential, power and semidefinite cones; warm start; parallelism;
an expanded sparse representation of large cones (not needed for cones of dimension <= ~20).
