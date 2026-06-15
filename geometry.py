"""Kinematics of a Hoberman sphere built on the icosidodecahedron.

The sphere is 6 great-circle scissor rings (planes normal to the 6 five-fold
axes of an icosahedron).  Every pair of rings crosses at exactly 2 of the 30
icosidodecahedron vertices, where 2-axis connectors couple them.

Each ring is a closed loop of N_RAYS/  angulated scissor units.  Classic
Hoberman ring math, unit half-angle alpha = pi/N_RAYS:

    Ri^2 - 2*Ri*Ro*cos(2a) + Ro^2 = 4 L^2 cos(a)^2     (deployment family)

with L the arm length (kink to joint), kink angle pi - 2a, and the kink
pivot sitting on the unit bisector at radius c = (Ri+Ro)/(2 cos a).
"""

import numpy as np

PHI = (1 + 5 ** 0.5) / 2
N_RAYS = 20                      # junction rays per ring (2 units per 36 deg arc)
ALPHA = np.pi / N_RAYS           # unit half-angle (9 deg)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def ring_normals():
    raw = [(0, 1, PHI), (0, 1, -PHI), (1, PHI, 0),
           (1, -PHI, 0), (PHI, 0, 1), (-PHI, 0, 1)]
    return [_unit(v) for v in raw]


def build_structure():
    """Return (rings, vertices).

    rings: list of dicts {n, e1, e2} -- plane normal and in-plane basis,
      chosen so the ring's 10 shared vertices sit at even junction rays
      (ray k at angle k*pi/10 from e1).
    vertices: list of dicts {u, inc} where u is the unit radial direction and
      inc = [(ring_idx, ray_idx), (ring_idx, ray_idx)] for the two rings.
    """
    normals = ring_normals()
    verts = []
    for i in range(6):
        for j in range(i + 1, 6):
            d = _unit(np.cross(normals[i], normals[j]))
            verts.append((d, i, j))
            verts.append((-d, i, j))
    assert len(verts) == 30

    rings = []
    for i, n in enumerate(normals):
        vs = [u for (u, a, b) in verts if i in (a, b)]
        assert len(vs) == 10
        e1 = _unit(vs[0])
        e2 = _unit(np.cross(n, e1))
        angs = sorted((np.arctan2(u @ e2, u @ e1) % (2 * np.pi)) for u in vs)
        gaps = np.diff(angs + [angs[0] + 2 * np.pi])
        assert np.allclose(gaps, np.pi / 5, atol=1e-9), gaps
        rings.append(dict(n=n, e1=e1, e2=e2))

    vertices = []
    for (u, i, j) in verts:
        inc = []
        for r in (i, j):
            R = rings[r]
            th = np.arctan2(u @ R['e2'], u @ R['e1']) % (2 * np.pi)
            k = th / (np.pi / 10)
            ki = int(round(k))
            assert abs(k - ki) < 1e-8, (k, ki)
            ki %= N_RAYS
            assert ki % 2 == 0
            inc.append((r, ki))
        vertices.append(dict(u=u, inc=inc))
    return rings, vertices


# ---------------------------------------------------------------- deployment

def solve_Ro(Ri, L, a=ALPHA):
    """Outer joint radius for a given inner radius on the deployment family."""
    s = 4 * L ** 2 * np.cos(a) ** 2 - Ri ** 2 * np.sin(2 * a) ** 2
    assert s >= 0, "state beyond full expansion"
    return Ri * np.cos(2 * a) + np.sqrt(s)


def L_for(Ro, Ri, a=ALPHA):
    """Arm length so that (Ri, Ro) lies on the deployment family."""
    return np.hypot(Ro - Ri * np.cos(2 * a), Ri * np.sin(2 * a)) / (2 * np.cos(a))


def Ro_max(L, a=ALPHA):
    return L / np.sin(a)


def solve_Ri(Ro, L, a=ALPHA):
    """Inner joint radius for a given outer radius (expansion branch)."""
    s = 4 * L ** 2 * np.cos(a) ** 2 - Ro ** 2 * np.sin(2 * a) ** 2
    assert s >= 0, "state beyond full expansion"
    return Ro * np.cos(2 * a) - np.sqrt(s)


def kink_radius(Ri, Ro, a=ALPHA):
    return (Ri + Ro) / (2 * np.cos(a))


def ring_state(Ri, L, a=ALPHA):
    """Validated (Ri, Ro, c) plus self-check of link rigidity."""
    Ro = solve_Ro(Ri, L, a)
    c = kink_radius(Ri, Ro, a)
    # rigid-body self check: arm lengths and kink angle
    A = Ro * np.array([np.cos(a), np.sin(a)])
    B = Ri * np.array([np.cos(a), -np.sin(a)])
    K = np.array([c, 0.0])
    la, lb = np.linalg.norm(A - K), np.linalg.norm(B - K)
    assert abs(la - L) < 1e-9 * L and abs(lb - L) < 1e-9 * L, (la, lb, L)
    cosk = (A - K) @ (B - K) / (la * lb)
    assert abs(np.arccos(np.clip(cosk, -1, 1)) - (np.pi - 2 * a)) < 1e-7
    return Ri, Ro, c
