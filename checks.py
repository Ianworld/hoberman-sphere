"""Clearance verification: 2D in-ring checks and 3D sampled gap checks."""

import numpy as np
from scipy.spatial import cKDTree
import trimesh

import geometry as G
import parts as PT


def ring2d_gaps(Ri, L, P):
    """Min 2D gap between same-layer link outlines within one ring.

    All rings are identical in-plane, and the -h layer is a mirror image of
    the +h layer, so checking link A_0 of one ring against A_1..A_19 covers
    every same-layer pair in the model.  Returns (min_gap, worst_k).
    """
    Ri, Ro, c = G.ring_state(Ri, L)
    step = 2 * np.pi / G.N_RAYS

    def outline(k):
        th0, th1, thb = k * step, (k + 1) * step, (k + 0.5) * step
        d = lambda t: np.array([np.cos(t), np.sin(t)])
        return PT.link_outline_2d(Ro * d(th0), c * d(thb), Ri * d(th1), P)

    a0 = outline(0)
    best, worst_k = np.inf, None
    for k in range(1, G.N_RAYS):
        g = a0.distance(outline(k))
        if g < best:
            best, worst_k = g, k
    return best, worst_k


def _sample(mesh, spacing):
    n = max(300, int(mesh.area / spacing ** 2))
    return trimesh.sample.sample_surface(mesh, n)[0]


def assembly_gaps(assembly, joint_min=0.35, free_min=0.45,
                  coarse=1.5, fine=0.4, refine_below=3.0, margin=2.0,
                  progress=None, cancel=None):
    """Sampled minimum gaps between all nearby part pairs.

    Returns (violations, stats); violations are (name_a, name_b, gap,
    shared_joint, contact_point).  Sampled distances overestimate the true
    gap by up to ~the sampling spacing; thresholds account for that.
    progress(i, n) is called periodically; if cancel() returns True the
    check aborts and returns (None, None).
    """
    n = len(assembly)
    lo = np.array([p['mesh'].bounds[0] for p in assembly])
    hi = np.array([p['mesh'].bounds[1] for p in assembly])
    overlap = (lo[:, None, :] <= hi[None, :, :] + margin).all(-1) & \
              (lo[None, :, :] <= hi[:, None, :] + margin).all(-1)
    ii, jj = np.where(np.triu(overlap, 1))

    coarse_pts = [_sample(p['mesh'], coarse) for p in assembly]
    fine_pts = {}

    def get_fine(i):
        if i not in fine_pts:
            fine_pts[i] = _sample(assembly[i]['mesh'], fine)
        return fine_pts[i]

    violations, joint_worst, free_worst = [], np.inf, np.inf
    for np_, (i, j) in enumerate(zip(ii, jj)):
        if cancel is not None and np_ % 25 == 0 and cancel():
            return None, None
        if progress is not None and np_ % 50 == 0:
            progress(np_, len(ii))
        point = None
        d = cKDTree(coarse_pts[i]).query(coarse_pts[j])[0].min()
        if d < refine_below:
            box_lo = np.maximum(lo[i], lo[j]) - 3.0
            box_hi = np.minimum(hi[i], hi[j]) + 3.0
            pi, pj = get_fine(i), get_fine(j)
            pi = pi[((pi >= box_lo) & (pi <= box_hi)).all(1)]
            pj = pj[((pj >= box_lo) & (pj <= box_hi)).all(1)]
            if len(pi) and len(pj):
                dd, idx = cKDTree(pi).query(pj)
                k = dd.argmin()
                d = dd[k]
                point = ((pj[k] + pi[idx[k]]) / 2).tolist()
        shared = assembly[i]['joints'] & assembly[j]['joints']
        thr = joint_min if shared else free_min
        if shared:
            joint_worst = min(joint_worst, d)
        else:
            free_worst = min(free_worst, d)
        if d < thr:
            violations.append((assembly[i]['name'], assembly[j]['name'],
                               float(d), bool(shared), point))
    stats = dict(pairs=len(ii), joint_worst=float(joint_worst),
                 free_worst=float(free_worst))
    return violations, stats
