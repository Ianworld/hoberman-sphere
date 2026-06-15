"""Scan print-state inner radius Ri for the tightest 3D-clean fold."""

import sys
import numpy as np

import geometry as G
import parts as PT
import build as B
import checks as C

TARGET = 320.0


def try_Ri(Ri, P, rings, vertices):
    extra = max(P['r_boss'], P['r_body']) + 0.5
    Ro_target = TARGET / 2 - extra
    for _ in range(3):
        L = G.L_for(Ro_target, Ri)
        asm = B.build_assembly(rings, vertices, Ri, L, P)
        ext = B.combined(asm).extents.max()
        if abs(ext - TARGET) < 0.3:
            break
        Ro_target += (TARGET - ext) / 2
    g2d, _ = C.ring2d_gaps(Ri, L, P)
    viol, st = C.assembly_gaps(asm)
    ratio = (2 * 0.99 * G.Ro_max(L) + 12) / TARGET
    print(f'Ri={Ri:5.1f} L={L:6.2f} box={ext:6.1f} 2D={g2d:4.2f} '
          f'viol={len(viol):4d} free_min={st["free_worst"]:5.2f} '
          f'joint_min={st["joint_worst"]:4.2f} ~ratio={ratio:.2f}')
    for v in sorted(viol, key=lambda v: v[2])[:8]:
        print('    ', v)
    return len(viol) == 0


if __name__ == '__main__':
    P = PT.default_params()
    rings, vertices = G.build_structure()
    Ris = []
    for a in sys.argv[1:]:
        if '=' in a:
            k, v = a.split('=')
            P[k] = float(v)
        else:
            Ris.append(float(a))
    print({k: v for k, v in P.items()})
    for Ri in Ris:
        try_Ri(Ri, P, rings, vertices)
