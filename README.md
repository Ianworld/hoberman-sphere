# Print-in-place Hoberman sphere for SLS

Generates `hoberman_sphere_320_print.stl`: a fully assembled, print-in-place
Hoberman sphere that prints in its compact state inside a 320 mm cube and
expands to ~641 mm (2.0x).

## Design

Faithful Hoberman architecture on the icosidodecahedron: 6 great-circle
scissor rings (20 angulated links each, kink angle 162 deg), crossing at 30
shared vertices.  Each vertex has two independent 2-axis connectors (outer
and inner) that couple the two crossing rings; all 300 bodies print in place
with integral pins and retaining caps.

Print-in-place features (all clearance-checked in code):
- 0.5 mm radial clearance on every pin/hole, 0.5 mm axial on every facing pair
- >= 0.5 mm free clearance between all non-mating bodies, verified by sampled
  3D distance checks at the print state, three mid-deployment states, and the
  fully open state (and 2D in-ring checks across the full sweep)
- stepped rim relief on link faces, tip-relieved caps, alternating kink-pin
  directions, and jogged vertex ends -- all needed to clear the crossing-ring
  contention near the 30 vertices

## Setup

The Python virtual environment is not checked in -- create it once after
cloning:

    python3 -m venv .venv
    ./.venv/bin/pip install -r requirements.txt

This installs numpy/scipy/shapely/trimesh/manifold3d/rtree (geometry +
clearance checks), flask (browser version), pywebview (desktop app), and
matplotlib (preview renders).  The app's launcher expects the env at
`.venv/` inside this directory.

## Usage

Desktop app (recommended -- no server, opens a native window):

    Double-click "Hoberman Designer.app"      # or: open "Hoberman Designer.app"

The page talks to Python directly through pywebview's JS bridge; nothing
listens on any port, and the app runs fully offline (three.js is vendored
in vendor/ and inlined into the page -- no CDN, no network access at all).

macOS note: launch via the .app bundle, not `python app.py` directly.  A
bare interpreter run from a terminal inherits a background activation
context, so the process runs but no window ever appears -- the bundle's
Info.plist is what registers it with LaunchServices as a foreground GUI
app.  The .app's executable just `exec`s ./.venv/bin/python app.py, so the
app is the real entry point.  ("Hoberman Designer.command" simply opens the
bundle, for those who prefer a clickable script.)

A browser version with identical UI is also available:

    ./.venv/bin/python server.py              # then open http://127.0.0.1:8765

Live side-by-side views of the compressed and expanded states; sliders +
number boxes (with editable slider end-points) for the geometry and
3D-printing dimensions; computed values (arm length, expansion ratio,
clearances, material) shown read-only.  An "auto interference check" box
re-runs the 4-state clearance check after every edit, with progress shown
and contact locations marked in red in the views.  "Export full-quality
STLs" writes print-ready files for the current settings.

Command line:

    ./.venv/bin/python generate.py            # full build + all checks (~2 min)
    ./.venv/bin/python generate.py --quick    # skip 3D checks
    ./.venv/bin/python tune.py [k=v ...] RI   # clearance scan at a given fold

Key knobs: `RI_PRINT` in generate.py (fold depth: smaller = more expansion
until clearances fail) and the dimension dict in `parts.default_params()`.

## Printing notes (SLS, PA12)

- 320 x 320 x 320 mm compact box -- needs a large-format SLS machine
  (does not fit a Fuse 1+ 165 x 165 x 300 chamber; at Fuse scale, set
  TARGET = 160 and expect ~0.25 mm clearances to be too tight -- rescale
  clearances accordingly).
- No supports needed; orient freely.  ~300 g / 297 cm^3 of material.
- Depowder thoroughly: 360 joints at 0.5 mm clearance hold powder; bead
  blast, then work the mechanism gently to free the joints.
- Links are 4 x 2.6 mm PA12: springy and toy-grade by design.

Files: `geometry.py` (kinematics), `parts.py` (part meshes), `build.py`
(assembly), `checks.py` (clearance verification), `generate.py` (main),
`tune.py` / `diagnose.py` (tuning tools).
