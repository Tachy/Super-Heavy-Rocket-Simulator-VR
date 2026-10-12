"""Generates reference SOCP problems for the optim module (core/src/optim) and solves them with Clarabel.

Writes tests/validation/socp/<name>.txt: the problem in our standard form
    min c'x  s.t.  A x = b,  G x + s = h,  s in R+^l x Q^{q1} x ...
plus Clarabel's solution. Offline tool only, not part of the build: CVXPY (Apache-2.0) builds the
landing and portfolio problems, Clarabel (Apache-2.0) solves all of them.

    python -m pip install cvxpy clarabel numpy scipy
    python tools/validation/gen_socp_fixtures.py
"""
import pathlib

import clarabel
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

OUT = pathlib.Path(__file__).resolve().parents[2] / "tests" / "validation" / "socp"


class Problem:
    def __init__(self, c, A, b, G, h, l, socs):
        self.c, self.A, self.b = np.asarray(c, float), sp.csc_matrix(A), np.asarray(b, float)
        self.G, self.h = sp.csc_matrix(G), np.asarray(h, float)
        self.l, self.socs = int(l), [int(q) for q in socs]


def cone_point(rng, l, socs):
    """Strictly interior point of R+^l x Q^{q1} x ..."""
    parts = [rng.uniform(0.5, 2.0, l)]
    for q in socs:
        tail = rng.standard_normal(q - 1)
        parts.append(np.concatenate([[np.linalg.norm(tail) + rng.uniform(0.5, 2.0)], tail]))
    return np.concatenate(parts)


def sparse_with_full_rows(rng, rows, cols, density):
    m = sp.random(rows, cols, density=density, random_state=rng, format="lil")
    for i in range(rows):
        m[i, i % cols] = rng.standard_normal() + 2.0
    return m.tocsc()


def random_problem(seed, n, p, l, socs):
    """Primal and dual strictly feasible by construction, hence a finite optimum."""
    rng = np.random.default_rng(seed)
    m = l + sum(socs)
    A = sparse_with_full_rows(rng, p, n, 0.3)
    G = sparse_with_full_rows(rng, m, n, 0.3)
    x0, y0 = rng.standard_normal(n), rng.standard_normal(p)
    s0, z0 = cone_point(rng, l, socs), cone_point(rng, l, socs)
    return Problem(-(A.T @ y0 + G.T @ z0), A, A @ x0, G, G @ x0 + s0, l, socs)


def from_cvxpy(problem):
    data, _, _ = problem.get_problem_data(cp.CLARABEL)
    dims = data["dims"]
    if dims.exp or dims.psd or getattr(dims, "p3d", []):
        raise RuntimeError("only zero, nonnegative and second-order cones are supported")
    A, b, z = sp.csc_matrix(data["A"]), np.asarray(data["b"], float), dims.zero
    return Problem(data["c"], A[:z], b[:z], A[z:], b[z:], dims.nonneg, list(dims.soc))


def portfolio_problem():
    rng = np.random.default_rng(5)
    n, k = 50, 5
    mu, F = rng.uniform(0.0, 0.2, n), rng.standard_normal((n, k)) * 0.1
    d = rng.uniform(0.01, 0.05, n)
    w = cp.Variable(n)
    risk = cp.norm(cp.hstack([F.T @ w, cp.multiply(d, w)]))
    return from_cvxpy(cp.Problem(cp.Maximize(mu @ w), [cp.sum(w) == 1, w >= 0, risk <= 0.05]))


def landing_problem():
    """3-DOF powered descent, lossless convexification (Acikmese & Ploen 2007), size of the in-flight
    guidance problem: 40 nodes, thrust-rate limit, tilt and glide-slope cones. Numbers: Super Heavy on its
    three centre engines (estimates)."""
    N, tf = 40, 18.0
    dt = tf / (N - 1)
    g = np.array([0.0, 0.0, -9.80665])
    m0, t_max = 2.6e5, 6.9e6
    t_min = 0.4 * t_max
    alpha = 1.0 / (330.0 * 9.80665)
    r0, v0 = np.array([200.0, -100.0, 500.0]), np.array([-10.0, 5.0, -60.0])
    tilt, glide, du_max = np.radians(15.0), np.radians(20.0), 3.0
    r, v, u = cp.Variable((3, N)), cp.Variable((3, N)), cp.Variable((3, N))
    z, s = cp.Variable(N), cp.Variable(N)
    cons = [r[:, 0] == r0, v[:, 0] == v0, z[0] == np.log(m0), r[:, -1] == 0, v[:, -1] == 0]
    for k in range(N - 1):
        cons += [
            v[:, k + 1] == v[:, k] + dt / 2 * (u[:, k] + u[:, k + 1]) + dt * g,
            r[:, k + 1] == r[:, k] + dt / 2 * (v[:, k] + v[:, k + 1]) + dt**2 / 12 * (u[:, k + 1] - u[:, k]),
            z[k + 1] == z[k] - alpha * dt / 2 * (s[k] + s[k + 1]),
            cp.norm(u[:, k + 1] - u[:, k]) <= du_max * dt,
        ]
    for k in range(N):
        t = k * dt
        z_ref = np.log(m0 - alpha * t_max * t)
        mu1, mu2 = t_min * np.exp(-z_ref), t_max * np.exp(-z_ref)
        cons += [
            cp.norm(u[:, k]) <= s[k],
            s[k] >= mu1 * (1 - (z[k] - z_ref) + cp.square(z[k] - z_ref) / 2),
            s[k] <= mu2 * (1 - (z[k] - z_ref)),
            z[k] >= z_ref,
            z[k] <= np.log(m0 - alpha * t_min * t),
            u[2, k] >= s[k] * np.cos(tilt),
            cp.norm(r[0:2, k]) * np.tan(glide) <= r[2, k],
        ]
    return from_cvxpy(cp.Problem(cp.Maximize(z[-1]), cons))


def solve_clarabel(prob):
    n, p = len(prob.c), prob.A.shape[0]
    cones = []
    if p:
        cones.append(clarabel.ZeroConeT(p))
    if prob.l:
        cones.append(clarabel.NonnegativeConeT(prob.l))
    cones += [clarabel.SecondOrderConeT(q) for q in prob.socs]
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-10
    solver = clarabel.DefaultSolver(sp.csc_matrix((n, n)), prob.c, sp.vstack([prob.A, prob.G]).tocsc(),
                                    np.concatenate([prob.b, prob.h]), cones, settings)
    solution = solver.solve()
    if solution.status != clarabel.SolverStatus.Solved:
        raise RuntimeError(f"Clarabel: {solution.status}")
    return np.array(solution.x), float(prob.c @ np.array(solution.x))


def write(name, prob, unique):
    x, objective = solve_clarabel(prob)
    OUT.mkdir(parents=True, exist_ok=True)

    def vec(key, values):
        return f"{key} {len(values)} " + " ".join(repr(float(v)) for v in values) + "\n"

    def mat(key, m):
        coo = m.tocoo()
        lines = [f"{key} {m.shape[0]} {m.shape[1]} {coo.nnz}\n"]
        lines += [f"{i} {j} {float(v)!r}\n" for i, j, v in zip(coo.row, coo.col, coo.data)]
        return "".join(lines)

    with open(OUT / f"{name}.txt", "w", newline="\n") as f:
        f.write(f"# {name}: generated by tools/validation/gen_socp_fixtures.py, reference Clarabel {clarabel.__version__}\n")
        f.write(f"variables {len(prob.c)}\n")
        f.write(f"nonnegative {prob.l}\n")
        f.write(f"second_order {len(prob.socs)}" + "".join(f" {q}" for q in prob.socs) + "\n")
        f.write(vec("c", prob.c) + mat("A", prob.A) + vec("b", prob.b) + mat("G", prob.G) + vec("h", prob.h))
        f.write(f"objective {objective!r}\n")
        f.write(vec("x", x))
        f.write(f"unique {int(unique)}\n")
    print(f"{name}: n={len(prob.c)} p={prob.A.shape[0]} m={prob.G.shape[0]} objective={objective:.10g}")


def main():
    write("lp_small", random_problem(1, n=10, p=3, l=20, socs=[]), unique=True)
    write("socp_small", random_problem(2, n=10, p=2, l=5, socs=[3, 4, 5]), unique=True)
    write("socp_medium", random_problem(3, n=100, p=20, l=60, socs=[3] * 20 + [5] * 10), unique=True)
    write("socp_large", random_problem(4, n=200, p=30, l=150, socs=[4] * 40), unique=True)
    write("portfolio", portfolio_problem(), unique=False)
    write("landing_gfold", landing_problem(), unique=False)


if __name__ == "__main__":
    main()
