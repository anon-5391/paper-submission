import cadquery as cq
import math
import os
import functools

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def create_model(
    inner_dia,
    disc_dia,
    offset,
    backbone_outer_dia=25,
    bottom_margin=25,      # FIXED, plain, PREPENDED before the cuts start -- no cuts here at all
    backbone_length=100,   # the cuts region -- top_margin is carved OUT of this, not added on top
    top_margin=3.5,        # keep this in the 2-5mm range -- carved from the end of backbone_length
    # tendon_hole_dist=9.5,
):
    """
        z=0 ---- bottom_margin ---- (bottom_margin+backbone_length)
             |<--- PLAIN, --->|<---------- backbone_length ---------->|
             |    no cuts     |<-- cuts, packed -->|<--top_margin-->|
             ^ z=0                                 ^ z = bottom_margin + backbone_length - top_margin

    total_length = bottom_margin + backbone_length  (e.g. 25 + 100 = 125 -- NOT 125+25)

    - bottom_margin (25mm) is prepended, plain, no cuts -- fixed regardless of disc_dia.
    - Cuts start immediately at z = bottom_margin, and run until
      z = bottom_margin + backbone_length - top_margin. top_margin (2-5mm) is
      carved OUT of backbone_length, not additional length on top of it.
    - top_margin is a hard constant regardless of disc_dia -- to land on it exactly,
      pitch between notches is stretched slightly and uniformly.

    All notch/hourglass cutting solids are collected and unioned into ONE compound,
    then subtracted in a SINGLE .cut() call for speed.
    """

    backbone_r = backbone_outer_dia / 2
    disc_r = disc_dia / 2
    center_dist = disc_dia * 1

    total_length = bottom_margin + backbone_length

    # =====================================
    # FULL SOLID (margin + cuts region, one piece to start)
    # =====================================
    result = (
        cq.Workplane("XY")
        .circle(backbone_r)
        .circle(inner_dia / 2)
        .extrude(total_length)
    )

    # Circle intersections with offset line
    z_bottom = math.sqrt(disc_r**2 - offset**2)
    z_top = center_dist - math.sqrt(disc_r**2 - offset**2)

    # Circle intersections with backbone wall
    z_right_bottom = math.sqrt(disc_r**2 - backbone_r**2)
    z_right_top = center_dist - math.sqrt(disc_r**2 - backbone_r**2)

    # =====================================
    # NOTCH POSITIONS: bottom_margin to (bottom_margin + backbone_length - top_margin)
    # =====================================
    clearance = 1.0

    cut_height = abs(z_right_bottom - z_right_top)
    nominal_pitch = cut_height + clearance  # used only to estimate how many notches fit

    usable_length = backbone_length - top_margin   # bottom_margin is NOT subtracted here -- it's separate
    num_notches = max(1, int((usable_length - cut_height) // nominal_pitch) + 1)

    first_center = bottom_margin + cut_height / 2
    last_center = bottom_margin + backbone_length - top_margin - cut_height / 2

    if num_notches == 1:
        positions = [first_center]
        actual_pitch = 0
    else:
        actual_pitch = (last_center - first_center) / (num_notches - 1)
        positions = [first_center + i * actual_pitch for i in range(num_notches)]

    hourglass_positions = [
        (positions[i] + positions[i + 1]) / 2
        for i in range(len(positions) - 1)
    ]

    print(f"num_notches = {num_notches}  nominal_pitch = {nominal_pitch:.2f}  actual_pitch = {actual_pitch:.2f}")
    print(f"total_length = {total_length}  (bottom_margin={bottom_margin} + backbone_length={backbone_length})")
    print(f"plain: 0.00 to {positions[0]-cut_height/2:.2f}   cuts: {positions[0]-cut_height/2:.2f} to {positions[-1]+cut_height/2:.2f}   plain: {positions[-1]+cut_height/2:.2f} to {total_length:.2f}")

    # =====================================
    # BUILD ALL CUTTING SOLIDS (not yet subtracted)
    # =====================================
    cut_tools = []

    for z in positions:
        tool = (
            cq.Workplane("YZ")
            .center(0, z - disc_r)
            .moveTo(offset, z_top)
            .radiusArc((backbone_r, z_right_top), -disc_r)
            .lineTo(backbone_r, z_right_bottom)
            .radiusArc((offset, z_bottom), -disc_r)
            .lineTo(offset, z_top)
            .close()
            .extrude(40)
        )
        cut_tools.append(tool)
        cut_tools.append(tool.mirror("YZ"))
        cut_tools.append(tool.mirror("XZ"))
        cut_tools.append(tool.mirror("YZ").mirror("XZ"))

    for z in hourglass_positions:
        hg = (
            cq.Workplane("XZ")
            .center(0, z - disc_r)
            .moveTo(offset, z_top)
            .radiusArc((backbone_r, z_right_top), -disc_r)
            .lineTo(backbone_r, z_right_bottom)
            .radiusArc((offset, z_bottom), -disc_r)
            .lineTo(offset, z_top)
            .close()
            .extrude(40, both=True)
        )
        cut_tools.append(hg)
        cut_tools.append(hg.mirror("YZ"))

    # =====================================
    # ONE union -> ONE cut (fast path)
    # =====================================
    combined_cutter = functools.reduce(lambda a, b: a.union(b), cut_tools)
    result = result.cut(combined_cutter)

    # =====================================
    # END CAP (flat solid at the very top of the sleeve)
    # =====================================
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=total_length - 5)
        .circle(backbone_r)
        .circle(inner_dia / 2)
        .extrude(5)
    )
    result = result.union(top_cap)

    # # =====================================
    # # TENDON HOLES (through the entire part, including the sleeve)
    # # =====================================
    # # NOT a uniform ring: two overlapping polar patterns (3-fold and 4-fold),
    # # both starting at angle 0, so they share one hole in common.
    # # 3 + 4 - 1 (shared) = 6 unique holes total, unevenly spaced.
    # tendon_holes_3 = (
    #     cq.Workplane("XY")
    #     .polarArray(radius=tendon_hole_dist, startAngle=0, angle=360, count=3)
    #     .circle(0)
    #     .extrude(total_length + 10)
    # )
    # result = result.cut(tendon_holes_3)

    # tendon_holes_4 = (
    #     cq.Workplane("XY")
    #     .polarArray(radius=tendon_hole_dist, startAngle=0, angle=360, count=4)
    #     .circle(0)
    #     .extrude(total_length + 10)
    # )
    # result = result.cut(tendon_holes_4)

    # result = result.clean()
    return result


if __name__ == "__main__":
    model = create_model(
        inner_dia=14,
        disc_dia=67.89,
        offset=2,
        backbone_outer_dia=25,
        bottom_margin=25,
        backbone_length=100,
        top_margin=3.5,
        # tendon_hole_dist=9.5,
    )
    out_path = os.path.join(OUTPUT_DIR, "backbone_final.stl")
    cq.exporters.export(model, out_path, tolerance=0.1, angularTolerance=0.5)
    print("Saved:", out_path)
