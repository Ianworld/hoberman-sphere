"""Core orchestration for the interactive designer.

Shared by app.py (native desktop window, no server) and server.py (browser).
Holds the latest preview build, and runs interference checks / full-quality
exports on background threads with pollable status.
"""

import os
import threading
import traceback

import numpy as np

import geometry as G
import parts as PT
import build as B
import checks as C

HERE = os.path.dirname(os.path.abspath(__file__))
RINGS, VERTICES = G.build_structure()

BUILD_LOCK = threading.Lock()      # serializes all mesh generation
CACHE = {}                         # latest preview build
CHECK = {'id': 0, 'running': False, 'stage': '', 'result': None,
         'gen': None, 'error': None}
EXPORT = {'running': False, 'message': '', 'done': False, 'error': None}

PREVIEW_SEG = 12

_UI_CACHE = {}


def load_ui(app_mode=False):
    """ui.html with the vendored three.js inlined (fully offline)."""
    key = bool(app_mode)
    if key not in _UI_CACHE:
        with open(os.path.join(HERE, 'ui.html')) as f:
            html = f.read()
        tags = []
        for name in ('three.min.js', 'OrbitControls.js', 'STLLoader.js'):
            with open(os.path.join(HERE, 'vendor', name)) as f:
                tags.append('<script>' + f.read() + '</script>')
        html = html.replace('<!--VENDOR_JS-->', '\n'.join(tags), 1)
        if app_mode:
            probe = ('<script>window.__APP_MODE=true;window.__errs=[];'
                     'window.onerror=function(m,s,l){window.__errs.push('
                     'm+" @"+(s||"")+":"+l);};'
                     'window.addEventListener("unhandledrejection",function(e)'
                     '{window.__errs.push("rejection: "+e.reason);});</script>')
            html = html.replace('<head>', '<head>' + probe, 1)
        _UI_CACHE[key] = html
    return _UI_CACHE[key]


def make_P(u):
    """UI params -> full dimension dict (derived sub-dimensions included)."""
    P = PT.default_params()
    for k in ('t', 'w', 'r_boss', 'pin_d', 'c_ax', 'h', 'jog', 'jog_len',
              'cap_t', 'r_body'):
        P[k] = float(u[k])
    clr = float(u['clr'])
    P['hole_d'] = P['pin_d'] + 2 * clr
    P['pin_k'] = max(1.8, P['pin_d'] - 0.4)
    P['hole_k'] = P['pin_k'] + 2 * clr
    P['r_kink'] = max(P['hole_k'] / 2 + 0.9, P['r_boss'] - 0.4)
    P['cap_d'] = P['pin_d'] + 3.5
    P['cap_k'] = P['pin_k'] + 3.0
    P['d_shoulder'] = P['pin_d'] + 3.0
    return P


def solve(u, P):
    box, Ri = float(u['box']), float(u['Ri'])
    of = float(u['open_frac'])
    extra = max(P['r_boss'], P['r_body']) + 0.5
    Ro_target = box / 2 - extra
    if Ri >= 0.85 * Ro_target:
        raise ValueError('fold depth Ri too large for this box size')
    L = G.L_for(Ro_target, Ri)
    P['jog_len'] = min(P['jog_len'], 0.55 * L)
    Ro_max = G.Ro_max(L)
    Ro_open = of * Ro_max
    if G.solve_Ro(Ri, L) >= Ro_open:
        raise ValueError('expansion limit is below the printed state')
    Ri_open = G.solve_Ri(Ro_open, L)
    return L, Ri, Ri_open, Ro_max


def build_assembly(Ri_state, L, P, seg=PREVIEW_SEG, fast=True):
    with BUILD_LOCK:
        old = PT.SEG, PT.FAST
        PT.SEG, PT.FAST = seg, fast
        try:
            return B.build_assembly(RINGS, VERTICES, Ri_state, L, P)
        finally:
            PT.SEG, PT.FAST = old


def do_build(u):
    """Build both preview states; returns derived values plus STL bytes."""
    CHECK['id'] += 1               # cancel any in-flight check (stale params)
    try:
        P = make_P(u)
        L, Ri, Ri_open, Ro_max = solve(u, P)
        gap2d, _ = C.ring2d_gaps(Ri, L, P)
        asm_p = build_assembly(Ri, L, P)
        asm_o = build_assembly(Ri_open, L, P)
        mesh_p, mesh_o = B.combined(asm_p), B.combined(asm_o)
        gen = CACHE.get('gen', 0) + 1
        CACHE.update(dict(
            gen=gen, P=P, L=L, Ri=Ri, Ri_open=Ri_open,
            asm={'print': asm_p, 'open': asm_o},
            stl={'print': mesh_p.export(file_type='stl'),
                 'open': mesh_o.export(file_type='stl')}))
        vol = sum(p['mesh'].volume for p in asm_p) / 1000.0
        derived = {
            'Arm length L': f'{L:.2f} mm',
            'Outer joint radius (printed)': f'{G.solve_Ro(Ri, L):.1f} mm',
            'Outer joint radius (kinematic max)': f'{Ro_max:.1f} mm',
            'Compact box (measured)': f'{mesh_p.extents.max():.1f} mm',
            'Expanded box (measured)': f'{mesh_o.extents.max():.1f} mm',
            'Expansion ratio':
                f'{mesh_o.extents.max() / mesh_p.extents.max():.2f} x',
            'In-ring 2D clearance (print)': f'{gap2d:.2f} mm',
            'Pin hole diameter': f'{P["hole_d"]:.2f} mm',
            'Kink pin / hole': f'{P["pin_k"]:.1f} / {P["hole_k"]:.1f} mm',
            'Ring slab half-thickness':
                f'{P["h"] + P["jog"] + P["t"] / 2 + P["c_ax"] + P["cap_t"]:.1f} mm',
            'Material (approx, PA12)': f'{vol * 1.01:.0f} g',
            'Bodies': f'{len(asm_p)}',
        }
        warn = []
        if gap2d < 0.5:
            warn.append(f'in-ring 2D clearance {gap2d:.2f} mm < 0.5 mm')
        wall = P['r_boss'] - P['hole_d'] / 2
        if wall < 0.8:
            warn.append(f'joint boss wall only {wall:.2f} mm -- increase '
                        'boss radius or reduce pin diameter/clearance')
        return dict(ok=True, gen=gen, derived=derived, warnings=warn,
                    stl_print=CACHE['stl']['print'],
                    stl_open=CACHE['stl']['open'])
    except Exception as e:
        traceback.print_exc()
        return dict(ok=False, error=str(e))


# ----------------------------------------------------- interference checking

def _run_check(my_id, snap):
    states = [('print', snap['Ri'], snap['asm']['print']),
              ('mid 50%', None, None), ('mid 75%', None, None),
              ('open', snap['Ri_open'], snap['asm']['open'])]
    fracs = {'mid 50%': 0.5, 'mid 75%': 0.75}
    out, per_state = [], []
    worst_free = worst_joint = np.inf

    def cancelled():
        return CHECK['id'] != my_id
    try:
        for label, ri, asm in states:
            if cancelled():
                return
            if asm is None:
                CHECK['stage'] = f'building {label} state...'
                ri = snap['Ri'] + fracs[label] * (snap['Ri_open'] - snap['Ri'])
                asm = build_assembly(ri, snap['L'], snap['P'])
            def prog(i, n, label=label):
                CHECK['stage'] = f'checking {label}  ({i}/{n} pairs)'
            viol, st = C.assembly_gaps(asm, coarse=2.0, fine=0.5,
                                       progress=prog, cancel=cancelled)
            if viol is None:
                return
            worst_free = min(worst_free, st['free_worst'])
            worst_joint = min(worst_joint, st['joint_worst'])
            per_state.append(dict(state=label, pairs=st['pairs'],
                                  freeMin=round(st['free_worst'], 2),
                                  jointMin=round(st['joint_worst'], 2),
                                  nViol=len(viol)))
            for (a, b, gap, joint, pt) in viol:
                out.append(dict(state=label, a=a, b=b, gap=round(gap, 3),
                                joint=joint, point=pt))
        out.sort(key=lambda v: v['gap'])
        if CHECK['id'] == my_id:
            CHECK['result'] = dict(
                ok=not out, total=len(out), violations=out[:15],
                perState=per_state, worstFree=round(float(worst_free), 2),
                worstJoint=round(float(worst_joint), 2))
            CHECK['gen'] = snap['gen']
    except Exception as e:
        traceback.print_exc()
        if CHECK['id'] == my_id:
            CHECK['error'] = str(e)
    finally:
        if CHECK['id'] == my_id:
            CHECK['running'] = False
            CHECK['stage'] = ''


def start_check():
    if not CACHE.get('gen'):
        return dict(ok=False, error='no build yet')
    CHECK['id'] += 1
    CHECK.update(running=True, stage='starting...', result=None,
                 error=None, gen=None)
    threading.Thread(target=_run_check, args=(CHECK['id'], dict(CACHE)),
                     daemon=True).start()
    return dict(ok=True, id=CHECK['id'])


def check_status():
    return dict(id=CHECK['id'], running=CHECK['running'],
                stage=CHECK['stage'], result=CHECK['result'],
                gen=CHECK['gen'], error=CHECK['error'])


# ------------------------------------------------------- full-quality export

def _run_export(snap):
    try:
        EXPORT['message'] = 'building full-quality compact state...'
        asm_p = build_assembly(snap['Ri'], snap['L'], snap['P'],
                               seg=48, fast=False)
        EXPORT['message'] = 'building full-quality expanded state...'
        asm_o = build_assembly(snap['Ri_open'], snap['L'], snap['P'],
                               seg=48, fast=False)
        f1 = os.path.join(HERE, 'hoberman_sphere_print.stl')
        f2 = os.path.join(HERE, 'hoberman_sphere_open_preview.stl')
        B.combined(asm_p).export(f1)
        B.combined(asm_o).export(f2)
        EXPORT['message'] = f'wrote {f1} and {f2}'
        EXPORT['done'] = True
    except Exception as e:
        traceback.print_exc()
        EXPORT['error'] = str(e)
    finally:
        EXPORT['running'] = False


def start_export():
    if EXPORT['running']:
        return dict(ok=False, error='export already running')
    if not CACHE.get('gen'):
        return dict(ok=False, error='no build yet')
    EXPORT.update(running=True, message='starting...', done=False, error=None)
    threading.Thread(target=_run_export, args=(dict(CACHE),),
                     daemon=True).start()
    return dict(ok=True)


def export_status():
    return dict(**EXPORT)
