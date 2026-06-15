"""Generate a print-in-place Hoberman sphere STL for SLS.

Printed in the compact state inside a TARGET^3 box; expands to ~2.5x.
Usage: python generate.py [--quick]   (--quick skips the 3D gap check)
"""

import sys
import numpy as np

import geometry as G
import parts as PT
import build as B
import checks as C

TARGET = 320.0          # compact bounding box, mm
RI_PRINT = 62.0         # starting inner joint radius at print state, mm
MIN_GAP_2D = 0.5        # required in-plane gap between separate links, mm
OPEN_FRAC = 0.99        # fraction of kinematic max expansion for preview


def size_linkage(P, Ri_print):
    """Pick arm length L so the compact assembly box is exactly TARGET."""
    extra = max(P['r_boss'], P['r_body']) + 0.5   # hardware beyond Ro
    Ro_target = TARGET / 2 - extra
    for _ in range(4):
        L = G.L_for(Ro_target, Ri_print)
        asm = B.build_assembly(*G.build_structure(), Ri_print, L, P)
        ext = B.combined(asm).extents.max()
        if abs(ext - TARGET) < 0.25:
            return L, asm, ext
        Ro_target += (TARGET - ext) / 2
    return L, asm, ext


def main():
    quick = '--quick' in sys.argv
    P = PT.default_params()
    rings, vertices = G.build_structure()

    # bump Ri_print until the folded ring passes the 2D self-clearance check
    Ri = RI_PRINT
    extra = max(P['r_boss'], P['r_body']) + 0.5
    while True:
        L = G.L_for(TARGET / 2 - extra, Ri)
        gap, worst_k = C.ring2d_gaps(Ri, L, P)
        print(f'Ri={Ri:6.1f}  L={L:6.2f}  2D gap={gap:5.2f} (vs A_{worst_k})')
        if gap >= MIN_GAP_2D:
            break
        Ri += 1.0
        assert Ri < 120, 'cannot satisfy 2D clearance'

    # 2D check across the whole deployment range
    Ro_open = OPEN_FRAC * G.Ro_max(L)
    Ri_open = G.solve_Ri(Ro_open, L)
    for f in np.linspace(0, 1, 9):
        r = Ri + f * (Ri_open - Ri)
        gap, _ = C.ring2d_gaps(r, L, P)
        assert gap >= MIN_GAP_2D - 0.05, f'2D gap {gap:.2f} at Ri={r:.1f}'
    print(f'2D clearance ok over full deployment (Ri {Ri:.1f} -> {Ri_open:.1f})')

    L, asm, ext = size_linkage(P, Ri)
    print(f'print state: L={L:.2f}  box={ext:.1f} mm  parts={len(asm)}')

    if not quick:
        mids = [Ri + f * (Ri_open - Ri) for f in (0.25, 0.5, 0.75)]
        states = [('print', Ri)] + [(f'mid{i}', m) for i, m in
                                    enumerate(mids)] + [('open', Ri_open)]
        for label, ri in states:
            a = asm if ri == Ri else B.build_assembly(rings, vertices, ri, L, P)
            viol, st = C.assembly_gaps(a)
            print(f'3D check [{label}]: {st["pairs"]} pairs, '
                  f'joint min {st["joint_worst"]:.2f}, '
                  f'free min {st["free_worst"]:.2f}, '
                  f'{len(viol)} violations')
            for v in sorted(viol, key=lambda v: v[2])[:20]:
                print('   ', v)
            if viol:
                sys.exit(1)

    mesh = B.combined(asm)
    mesh.export('hoberman_sphere_320_print.stl')
    open_asm = B.build_assembly(rings, vertices, Ri_open, L, P)
    open_mesh = B.combined(open_asm)
    open_mesh.export('hoberman_sphere_open_preview.stl')

    d_open = open_mesh.extents.max()
    vol = mesh.volume / 1000.0
    print(f'\ncompact box : {mesh.extents.round(1)} mm')
    print(f'expanded box: {open_mesh.extents.round(1)} mm '
          f'({d_open / ext:.2f}x expansion)')
    print(f'material    : {vol:.0f} cm^3  (~{vol * 1.01:.0f} g PA12)')
    print('wrote hoberman_sphere_320_print.stl (print this one) and '
          'hoberman_sphere_open_preview.stl')


if __name__ == '__main__':
    main()
