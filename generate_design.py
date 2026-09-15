#!/usr/bin/env python3
# generate_design.py — Stage 1 of the TDCR pipeline (CAD -> STL + cable centerline)
#
# Runs under the CadQuery env ($CQ_PYTHON). MUST be run with PYTHONPATH
# unset or the system numpy leaks in and breaks the import — run_one.sh handles
# that isolation.
#
# Outputs: <out-stl> surface mesh for SOFA, and <out-cable> one cable centerline
# [[x,y,z], ...] that tdcr_model.py rotates into the full cable set at sim time
# (cable count = TDCR_TENDON_COUNT, not a CAD parameter).

import argparse
import json
import os

import cadquery as cq

from caddesign import create_model

# The exported model has the backbone along +Y with tendon 1 at (radius, y, 0).
TENDON_RADIUS_MM = 9.5
CABLE_SAMPLES = 25  # points along the cable centerline


def build_cable_centerline(y_lo, y_hi, radius=TENDON_RADIUS_MM, n=CABLE_SAMPLES):
    """Tendon-1 centerline in the exported (+Y) frame: straight, at `radius` in the
    x=radius, z=0 column, spanning the full backbone height y_lo..y_hi so the
    cable runs base to tip. It is pulled from below the base (see
    pipeline_config.CABLE_PULL_POINT_OFFSET_MM)."""
    return [[radius, round(y_lo + (y_hi - y_lo) * i / (n - 1), 4), 0.0]
            for i in range(n)]


def main():
    ap = argparse.ArgumentParser(description="Generate one TDCR design: STL + cable centerline")
    ap.add_argument("--id", required=True, help="design id (used for filenames)")
    ap.add_argument("--inner-dia", type=float, default=14)
    ap.add_argument("--disc-dia", type=float, default=30)
    ap.add_argument("--offset", type=float, default=2)
    ap.add_argument("--backbone-outer-dia", type=float, default=25)
    ap.add_argument("--backbone-length", type=float, default=100)
    ap.add_argument("--tendon-dist", type=float, default=9.5,
                    help="cable-path radius from centre (mm); used for the cable centerline")
    ap.add_argument("--out-stl", required=True)
    ap.add_argument("--out-cable", required=True)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out_stl)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.out_cable)), exist_ok=True)

    print(f"[CAD] Building '{args.id}': disc_dia={args.disc_dia} "
          f"offset={args.offset} outer_dia={args.backbone_outer_dia} length={args.backbone_length}")

    model = create_model(
        inner_dia=args.inner_dia,
        disc_dia=args.disc_dia,
        offset=args.offset,
        backbone_outer_dia=args.backbone_outer_dia,
        backbone_length=args.backbone_length,
    )

    # caddesign builds along +Z (XY workplane); rotate -90° about X so +Z → +Y,
    # aligning the backbone with the Y axis that SOFA's fixing box and cables expect.
    model = model.rotate((0, 0, 0), (1, 0, 0), -90).clean()

    # Full Y extent of the exported part (may exceed args.backbone_length — the
    # part can carry a plain base section). The cable spans exactly this.
    bb = model.findSolid().BoundingBox()
    y_lo, y_hi = bb.ymin, bb.ymax
    total_height = y_hi - y_lo

    cq.exporters.export(model, args.out_stl)
    stl_bytes = os.path.getsize(args.out_stl)
    print(f"[CAD] Wrote STL  : {args.out_stl} ({stl_bytes} bytes, "
          f"height {total_height:.1f} mm)")

    # Write sidecar dims.json — read by run_one.sh to populate design_log.csv
    dims = {
        "design_id":          args.id,
        "stl_file":           os.path.abspath(args.out_stl),
        "stl_size_bytes":     stl_bytes,
        "inner_dia":          args.inner_dia,
        "disc_dia":           args.disc_dia,
        "offset":             args.offset,
        "backbone_outer_dia": args.backbone_outer_dia,
        "backbone_length":    args.backbone_length,
        "total_height_mm":    round(total_height, 3),
        "tendon_dist":        args.tendon_dist,
    }
    dims_path = args.out_stl.replace(".stl", ".dims.json")
    with open(dims_path, "w") as f:
        json.dump(dims, f, indent=4)
    print(f"[CAD] Wrote dims : {dims_path}")

    cable1 = build_cable_centerline(y_lo, y_hi, radius=args.tendon_dist)
    with open(args.out_cable, "w") as f:
        json.dump(cable1, f, indent=4)
    print(f"[CAD] Wrote cable: {args.out_cable} ({len(cable1)} points @ r={args.tendon_dist}mm, "
          f"y {y_lo:.1f}..{y_hi:.1f})")


if __name__ == "__main__":
    main()
