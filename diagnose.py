"""Where/how deep are the 3D violations at a given Ri?"""

import sys
import numpy as np
from scipy.spatial import cKDTree
import trimesh

import geometry as G
import parts as PT
import build as B
import checks as C

TARGET = 320.0
Ri = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0

P = PT.default_params()
rings, vertices = G.build_structure()
extra = max(P['r_boss'], P['r_body']) + 0.5
Ro_target = TARGET / 2 - extra
L = G.L_for(Ro_target, Ri)
asm = B.build_assembly(rings, vertices, Ri, L, P)
viol, st = C.assembly_gaps(asm)
print(f'Ri={Ri} L={L:.2f}: {len(viol)} violations')

name2idx = {p['name']: i for i, p in enumerate(asm)}
us = np.array([v['u'] for v in vertices])

rows = []
for (na, nb, d, shared, *_rest) in viol:
    A = asm[name2idx[na]]['mesh']
    Bm = asm[name2idx[nb]]['mesh']
    pa = trimesh.sample.sample_surface(A, 4000)[0]
    # penetration: how far inside B do A's points go (positive = inside)
    sd = trimesh.proximity.ProximityQuery(Bm).signed_distance(pa)
    pen = sd.max()
    loc = pa[sd.argmax()]
    r = np.linalg.norm(loc)
    ang = np.degrees(np.arccos(np.clip((us @ (loc / r)).max(), -1, 1)))
    rows.append((pen, r, ang, na, nb))

rows.sort(reverse=True)
print(' pen(mm)  radius  deg-to-nearest-vertex  pair')
for pen, r, ang, na, nb in rows[:15]:
    print(f'  {pen:6.2f}  {r:6.1f}  {ang:6.1f}   {na} | {nb}')
pens = np.array([r[0] for r in rows])
angs = np.array([r[2] for r in rows])
print(f'penetration: max {pens.max():.2f}  median {np.median(pens):.2f}  '
      f'>1mm: {(pens > 1).sum()}/{len(pens)}')
print(f'angle to nearest vertex: median {np.median(angs):.1f} deg, '
      f'max {angs.max():.1f} deg')
