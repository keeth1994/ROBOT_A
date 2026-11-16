"""Regenerate the B10 M3 camera input crank with CadQuery 2.8.

Run with a CAD-enabled Python environment:
    python cad/B10/generate_camera_crank.py --output-dir /path/to/output

Exports the standalone part only. Editing it does not automatically update
the Fusion assembly or its clearance reports. Dimensions are millimetres.
"""
import argparse
from pathlib import Path
import cadquery as cq

HORN_FACE_ABOVE_CASE = 7.34
ORIGINAL_HORN_FACE_ABOVE_CASE = 8.0
ORIGINAL_ARM_THICKNESS = 3.5
WEB_THICKNESS = ORIGINAL_ARM_THICKNESS + ORIGINAL_HORN_FACE_ABOVE_CASE - HORN_FACE_ABOVE_CASE
BOLT_BOSS_THICKNESS = 5.8
HORN_HOLE_RADIUS = 7.0
CRANK_RADIUS = 22.0


def cylinder(x, y, radius, height):
    return cq.Solid.makeCylinder(radius, height, (x, y, 0), (0, 0, 1))


def crank():
    """Flat horn-facing back at Z=0; pushrod hole at X=22, Y=0."""
    q = cylinder(0, 0, 12.5, WEB_THICKNESS).fuse(
        cq.Solid.makeBox(13, 9.6, WEB_THICKNESS, (9, -4.8, 0)),
        cylinder(9, 0, 4.8, WEB_THICKNESS),
        cylinder(CRANK_RADIUS, 0, 4.8, WEB_THICKNESS),
        cylinder(-HORN_HOLE_RADIUS, 0, 3.3, BOLT_BOSS_THICKNESS),
        cylinder(HORN_HOLE_RADIUS, 0, 3.3, BOLT_BOSS_THICKNESS),
    ).clean()
    for x, radius in [(0, 3.45), (-HORN_HOLE_RADIUS, 1.7), (HORN_HOLE_RADIUS, 1.7), (CRANK_RADIUS, 1.6)]:
        q = q.cut(cq.Solid.makeCylinder(radius, 8, (x, 0, -1), (0, 0, 1)))
    q = q.clean()
    if not q.isValid() or len(q.Solids()) != 1:
        raise RuntimeError('The crank is not one valid solid')
    return q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    q = crank()
    cq.exporters.export(q, str(args.output_dir/'Camera_input_crank_M3.step'))
    cq.exporters.export(q, str(args.output_dir/'Camera_input_crank_M3.stl'), tolerance=.033, angularTolerance=.09)
    print(f'One valid solid, {q.Volume():.6f} mm3, exported to {args.output_dir}')


if __name__ == '__main__':
    main()
