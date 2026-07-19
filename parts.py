"""Mesh builders for links, pins and vertex connectors (all dimensions mm)."""

import numpy as np
import trimesh
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

SEG = 48     # circle resolution (lowered for interactive previews)
FAST = False  # skip boolean unions (concatenate shells) for speed


def default_params():
    return dict(
        t=2.6,        # link thickness
        w=4.0,        # link arm width
        r_boss=3.5,   # joint boss radius (link ends + kink)
        pin_d=3.0,    # pin diameter
        hole_d=4.0,   # hole diameter (0.5 mm radial clearance)
        cap_d=6.5,    # retaining cap diameter
        cap_t=1.6,    # retaining cap thickness
        c_ax=0.5,     # axial clearance between facing surfaces
        h=6.5,        # layer mid-plane offset from ring plane
        r_body=4.0,   # connector central ball radius
        rim=1.0,      # link top/bottom rim relief (stepped chamfer)
        r_kink=3.1,   # kink boss radius (slimmer than end bosses)
        pin_k=2.6,    # kink pin diameter
        hole_k=3.6,   # kink hole diameter
        cap_k=5.6,    # kink cap diameter
        jog=4.5,      # vertex-end offset away from ring plane
        jog_len=16.0,  # in-plane length of the jogged end segment
        web_len=3.0,  # length of the vertical web joining the two levels
        d_shoulder=6.0,  # connector stub shoulder diameter
    )


def _frame(e1, e2, n):
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = e1, e2, n
    return T


def _union(meshes):
    if FAST:
        return trimesh.util.concatenate(meshes)
    try:
        m = trimesh.boolean.union(meshes, engine='manifold')
        if m.is_volume:
            return m
    except BaseException:
        pass
    return trimesh.util.concatenate(meshes)


def link_outline_2d(p_a, p_k, p_b, P):
    """Shapely polygon of an angulated link in ring-plane coordinates."""
    bar = LineString([tuple(p_a), tuple(p_k), tuple(p_b)]).buffer(
        P['w'] / 2, quad_segs=SEG // 4)
    bosses = [Point(tuple(p_a)).buffer(P['r_boss'], quad_segs=SEG // 4),
              Point(tuple(p_k)).buffer(P['r_kink'], quad_segs=SEG // 4),
              Point(tuple(p_b)).buffer(P['r_boss'], quad_segs=SEG // 4)]
    return unary_union([bar] + bosses)


def _bands(poly, hole_pts, z_mid, P):
    """Extrude poly to thickness t at z_mid with stepped rim relief."""
    eroded = poly.buffer(-P['rim'], quad_segs=SEG // 4)
    t, e = P['t'], P['rim']
    bands = [(poly, z_mid - t / 2 + e, z_mid + t / 2 - e),
             (eroded, z_mid + t / 2 - e, z_mid + t / 2),
             (eroded, z_mid - t / 2, z_mid - t / 2 + e)]
    out = []
    for pl, z0, z1 in bands:
        for p, dia in hole_pts:
            pl = pl.difference(Point(tuple(p)).buffer(
                dia / 2, quad_segs=SEG // 4))
        # erosion/holes can split the outline into several pieces (or eat a
        # band entirely) when holes approach the arm width -- extrude each
        for piece in _polys(pl):
            slab = trimesh.creation.extrude_polygon(piece, height=z1 - z0)
            slab.apply_translation([0, 0, z0])
            out.append(slab)
    return out


def _polys(geom):
    """Individual non-empty Polygons of any shapely geometry."""
    if geom.is_empty:
        return []
    if geom.geom_type == 'Polygon':
        return [geom]
    return [g for g in getattr(geom, 'geoms', [])
            if g.geom_type == 'Polygon' and not g.is_empty]


def link_mesh(p_a, p_k, p_b, z_mid, pin_pts, hole_pts, P, frame, jog_end):
    """One angulated link with a jogged (offset) vertex end.

    p_a/p_k/p_b: 2D ring-plane coords of joint A, kink, joint B.
    z_mid: layer mid-plane (+h or -h).  pin_pts: list of (point, pin_dia,
    cap_dia) where this link carries an integral pin (shaft + cap, pointing
    toward the other layer).  hole_pts: list of (point, dia) through-holes.
    frame: 4x4 ring->world.  jog_end: 'a' or 'b' -- which end is a vertex
    joint; that end segment is offset a further P['jog'] away from the ring
    plane so the two rings' parts pass each other at the shared vertices.
    """
    p_a, p_k, p_b = (np.asarray(p, float) for p in (p_a, p_k, p_b))
    s_out = 1.0 if z_mid > 0 else -1.0
    z_jog = z_mid + s_out * P['jog']
    p_e, p_far = (p_a, p_b) if jog_end == 'a' else (p_b, p_a)
    u = (p_k - p_e) / np.linalg.norm(p_k - p_e)
    q1 = p_e + u * P['jog_len']
    q2 = p_e + u * (P['jog_len'] + P['web_len'])

    w2 = P['w'] / 2
    out_jog = unary_union([
        LineString([tuple(p_e), tuple(q1 + 0.5 * u)]).buffer(
            w2, quad_segs=SEG // 4),
        Point(tuple(p_e)).buffer(P['r_boss'], quad_segs=SEG // 4)])
    out_base = unary_union([
        LineString([tuple(q1), tuple(p_k), tuple(p_far)]).buffer(
            w2, quad_segs=SEG // 4),
        Point(tuple(p_k)).buffer(P['r_kink'], quad_segs=SEG // 4),
        Point(tuple(p_far)).buffer(P['r_boss'], quad_segs=SEG // 4)])
    out_web = LineString([tuple(q1 - 0.5 * u), tuple(q2)]).buffer(
        w2, quad_segs=SEG // 4)

    pieces = _bands(out_jog, hole_pts, z_jog, P)
    pieces += _bands(out_base, hole_pts, z_mid, P)
    zlo = min(z_mid, z_jog) - P['t'] / 2
    zhi = max(z_mid, z_jog) + P['t'] / 2
    web = trimesh.creation.extrude_polygon(out_web, height=zhi - zlo)
    web.apply_translation([0, 0, zlo])
    pieces.append(web)

    s = -np.sign(z_mid)  # pins point toward the other layer
    z_own_far = z_mid + (-s) * P['t'] / 2          # far face of own body
    z_other_far = -z_mid + s * P['t'] / 2          # far face of other layer
    z_cap0 = z_other_far + s * P['c_ax']           # cap starts past clearance
    for p, pin_dia, cap_dia in pin_pts:
        pieces += _pin(p, pin_dia, cap_dia, z_own_far, z_cap0, s, P)
    mesh = _union(pieces)
    mesh.apply_transform(frame)
    return mesh


def _pin(p2, pin_dia, cap_dia, z_start, z_cap0, s, P):
    """Pin shaft plus a stepped (tip-relieved) retaining cap."""
    shaft = _cyl(pin_dia / 2, z_start, z_cap0, p2)
    z1 = z_cap0 + s * P['cap_t'] * 0.6
    z2 = z1 + s * P['cap_t'] * 0.4
    cap = _cyl(cap_dia / 2, z_cap0, z1, p2)
    tip = _cyl(pin_dia / 2 + 0.4, z1, z2, p2)
    return [shaft, cap, tip]


def _cyl(r, z0, z1, p2):
    lo, hi = min(z0, z1), max(z0, z1)
    c = trimesh.creation.cylinder(radius=r, height=hi - lo, sections=SEG)
    c.apply_translation([p2[0], p2[1], (lo + hi) / 2])
    return c


def connector_mesh(P_world, axes, P):
    """Vertex connector: central ball + 2 pin axes (4 stubs with caps).

    P_world: joint position (on the vertex ray).  axes: the two ring plane
    normals (unit vectors).  A shoulder spaces the (jogged) link end off the
    central ball; the pin runs just past the link far face, then a cap.
    """
    z_link = P['h'] + P['jog']                     # link end mid-plane
    stub_end = z_link + P['t'] / 2 + P['c_ax']
    z_shoulder = z_link - P['t'] / 2 - P['c_ax']
    pieces = [trimesh.creation.icosphere(subdivisions=2, radius=P['r_body'])]
    for n in axes:
        n = np.asarray(n, float)
        for s in (1.0, -1.0):
            shoulder = trimesh.creation.cylinder(
                radius=P['d_shoulder'] / 2, sections=SEG,
                segment=[[0, 0, 0], list(s * z_shoulder * n)])
            pieces.append(shoulder)
            shaft = trimesh.creation.cylinder(
                radius=P['pin_d'] / 2, sections=SEG,
                segment=[[0, 0, 0], list(s * stub_end * n)])
            z1 = stub_end + P['cap_t'] * 0.6
            z2 = z1 + P['cap_t'] * 0.4
            cap = trimesh.creation.cylinder(
                radius=P['cap_d'] / 2, sections=SEG,
                segment=[list(s * stub_end * n), list(s * z1 * n)])
            tip = trimesh.creation.cylinder(
                radius=P['pin_d'] / 2 + 0.4, sections=SEG,
                segment=[list(s * z1 * n), list(s * z2 * n)])
            pieces += [shaft, cap, tip]
    mesh = _union(pieces)
    mesh.apply_translation(P_world)
    return mesh
