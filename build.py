"""Assemble the full sphere at a given deployment state."""

import numpy as np
import trimesh

import geometry as G
import parts as PT


def _dir(th):
    return np.array([np.cos(th), np.sin(th)])


def build_assembly(rings, vertices, Ri, L, P):
    """Return list of part dicts {name, mesh, joints} at inner radius Ri.

    Joint ids: ('j', ring, ray, 'o'|'i') for ring junction joints and
    ('k', ring, unit) for scissor kink pivots.  Parts sharing an id are a
    pin/hole pair (designed clearance); all other nearby pairs are free and
    need full clearance.
    """
    Ri, Ro, c = G.ring_state(Ri, L)
    N = G.N_RAYS
    step = 2 * np.pi / N

    vertex_rays = {}  # (ring, ray) -> vertex idx
    for vi, v in enumerate(vertices):
        for (r, k) in v['inc']:
            vertex_rays[(r, k)] = vi

    out = []
    for ri, R in enumerate(rings):
        frame = PT._frame(R['e1'], R['e2'], R['n'])
        for k in range(N):
            th0, th1 = k * step, (k + 1) * step
            thb = th0 + step / 2
            Po0, Po1 = Ro * _dir(th0), Ro * _dir(th1)
            Pi0, Pi1 = Ri * _dir(th0), Ri * _dir(th1)
            K = c * _dir(thb)
            k1 = (k + 1) % N

            # link A: outer@k -> kink -> inner@k+1, layer +h.  Carries the
            # end pin at whichever end is a plain (odd-ray) junction; vertex
            # (even-ray) ends get holes.  The kink pin alternates by unit
            # parity (even: A pins down, odd: B pins up) so that the two kink
            # caps meeting near each shared vertex point to opposite sides.
            a_kink = k % 2 == 0
            pins = [(K, P['pin_k'], P['cap_k'])] if a_kink else []
            holes = [] if a_kink else [(K, P['hole_k'])]
            for ray, pt in ((k, Po0), (k1, Pi1)):
                if ray % 2 == 1:
                    pins.append((pt, P['pin_d'], P['cap_d']))
                else:
                    holes.append((pt, P['hole_d']))
            jog = 'a' if k % 2 == 0 else 'b'
            mesh = PT.link_mesh(Po0, K, Pi1, +P['h'], pins, holes, P,
                                frame, jog)
            out.append(dict(
                name=f'r{ri}_u{k:02d}_A', mesh=mesh,
                joints={('j', ri, k, 'o'), ('j', ri, k1, 'i'), ('k', ri, k)}))

            # link B: inner@k -> kink -> outer@k+1, layer -h.
            pins_b = [] if a_kink else [(K, P['pin_k'], P['cap_k'])]
            holes_b = [(Pi0, P['hole_d']), (Po1, P['hole_d'])]
            if a_kink:
                holes_b.insert(1, (K, P['hole_k']))
            mesh = PT.link_mesh(Pi0, K, Po1, -P['h'], pins_b, holes_b, P,
                                frame, jog)
            out.append(dict(
                name=f'r{ri}_u{k:02d}_B', mesh=mesh,
                joints={('j', ri, k, 'i'), ('j', ri, k1, 'o'), ('k', ri, k)}))

    for vi, v in enumerate(vertices):
        axes = [rings[r]['n'] for (r, _) in v['inc']]
        for tag, rad in (('o', Ro), ('i', Ri)):
            mesh = PT.connector_mesh(rad * v['u'], axes, P)
            out.append(dict(
                name=f'v{vi:02d}_{tag}', mesh=mesh,
                joints={('j', r, k, tag) for (r, k) in v['inc']}))
    return out


def combined(assembly):
    return trimesh.util.concatenate([p['mesh'] for p in assembly])
