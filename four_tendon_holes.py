import cadquery as cq
import math


def create_model(
    inner_dia,
    disc_dia,
    offset,
    backbone_outer_dia=25,
    backbone_length=100
):

    # =====================================
    # DERIVED PARAMETERS
    # =====================================

    backbone_r = backbone_outer_dia / 2
    disc_r = disc_dia / 2

    center_dist = disc_dia * 1

    # =====================================
    # BACKBONE
    # =====================================

    result = (
        cq.Workplane("XY")
        .circle(backbone_r)
        .circle(inner_dia / 2)
        .polarArray(
            radius=9.5,
            startAngle=0,
            angle=360,
            count=4
        )
        .circle(1)
        .extrude(backbone_length)
    )

    # Circle intersections with offset line
    z_bottom = math.sqrt(disc_r**2 - offset**2)
    z_top = center_dist - math.sqrt(disc_r**2 - offset**2)

    # Circle intersections with backbone wall
    z_right_bottom = math.sqrt(disc_r**2 - backbone_r**2)
    z_right_top = center_dist - math.sqrt(disc_r**2 - backbone_r**2)

   
    # =====================================
    # AUTOMATIC POSITION GENERATION
    # =====================================

    bottom_margin = 5
    top_margin = 5

    clearance = 1.0

    cut_height = abs(z_right_bottom - z_right_top)

    pitch = cut_height + clearance

    usable_length = (
        backbone_length
        - bottom_margin
        - top_margin
    )

    num_notches = int(
        (usable_length - cut_height) // pitch
    ) + 1

    start_z = bottom_margin + cut_height / 2

    positions = [
        start_z + i * pitch
        for i in range(num_notches)
    ]

    hourglass_positions = [
        (positions[i] + positions[i + 1]) / 2
        for i in range(len(positions) - 1)
    ]

    print("num_notches =", num_notches)
    print("positions =", positions)

    # =====================================
    # NOTCHES
    # =====================================

    for z in positions:

        tool = (
            cq.Workplane("YZ")
            .center(0, z - 15)

            .moveTo(offset, z_top)

            .radiusArc(
                (backbone_r, z_right_top),
                -disc_r
            )

            .lineTo(
                backbone_r,
                z_right_bottom
            )

            .radiusArc(
                (offset, z_bottom),
                -disc_r
            )

            .lineTo(
                offset,
                z_top
            )

            .close()

            .extrude(40)
        )

        result = result.cut(tool)

        yz_tool = tool.mirror("YZ")
        result = result.cut(yz_tool)

        xz_tool = tool.mirror("XZ")
        result = result.cut(xz_tool)

        xz_yz_tool = yz_tool.mirror("XZ")
        result = result.cut(xz_yz_tool)

    # =====================================
    # HOURGLASS CUTS
    # =====================================

    for z in hourglass_positions:

        hourglass_cut = (
            cq.Workplane("XZ")
            .center(0, z - 15)

            .moveTo(offset, z_top)

            .radiusArc(
                (backbone_r, z_right_top),
                -disc_r
            )

            .lineTo(
                backbone_r,
                z_right_bottom
            )

            .radiusArc(
                (offset, z_bottom),
                -disc_r
            )

            .lineTo(
                offset,
                z_top
            )

            .close()

            .extrude(40, both=True)
        )

        result = result.cut(hourglass_cut)

        mirrored_hourglass = hourglass_cut.mirror("YZ")
        result = result.cut(mirrored_hourglass)

    # =====================================
    # TOP CAP
    # =====================================

    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=backbone_length - 5)
        .circle(backbone_r)
        .circle(inner_dia / 2)
        .extrude(5)
    )

    result = result.union(top_cap)

    # =====================================
    # FINISH
    # =====================================

    result = result.clean()

    return result


if __name__ == "__main__":

    # ==========================
    # DEFAULT VALUES
    # ==========================

    DEFAULT_INNER_DIA      = 14
    DEFAULT_DISC_DIA       = 30
    DEFAULT_OFFSET         = 2
    DEFAULT_BACKBONE_OUTER = 25
    DEFAULT_BACKBONE_LEN   = 100

    print("\n" + "=" * 55)
    print("  TDCR Rolling-Disc Backbone Generator")
    print("=" * 55)

    print("\n--- Parameter to Vary ---")
    print("1. Central Cylinder Diameter")
    print("2. Rolling Disc Diameter")
    print("3. Notch Offset")
    print("4. Backbone Diameter")
    print("5. Backbone Length")

    choice = int(input("\nEnter choice (1-5): "))

    # ==========================
    # ASK FOR THE OTHER 4 FIXED PARAMETERS
    # ==========================
    def get_fixed(prompt, default):
        value = input(f"{prompt} [Default = {default}] : ")
        if value.strip() == "":
            return default
        return float(value)

    print("\n--- Fixed Parameters ---")

    inner_dia = (
        DEFAULT_INNER_DIA if choice == 1
        else get_fixed("Central Cylinder Diameter  [backbone rod channel, mm]", DEFAULT_INNER_DIA)
    )

    disc_dia = (
        DEFAULT_DISC_DIA if choice == 2
        else get_fixed("Rolling Disc Diameter      [must be > Backbone Dia, mm]", DEFAULT_DISC_DIA)
    )

    offset = (
        DEFAULT_OFFSET if choice == 3
        else get_fixed("Notch Offset               [notch lateral offset, mm]", DEFAULT_OFFSET)
    )

    backbone_outer_dia = (
        DEFAULT_BACKBONE_OUTER if choice == 4
        else get_fixed("Backbone Diameter          [must be < Rolling Disc Dia, mm]", DEFAULT_BACKBONE_OUTER)
    )

    backbone_length = (
        DEFAULT_BACKBONE_LEN if choice == 5
        else get_fixed("Backbone Length            [must be > Rolling Disc Dia, mm]", DEFAULT_BACKBONE_LEN)
    )

    # ==========================
    # RANGE FOR THE VARYING PARAMETER
    # ==========================
    print("\n--- Range to Generate ---")
    start_val = float(input("Start value: "))
    end_val   = float(input("End value: "))

    print("Generate by:")
    print("1. Step Size")
    print("2. Number of STL files")

    mode = int(input("Enter choice (1 or 2): "))

    values = []

    if mode == 1:

        step = float(input("Step size: "))

        current = start_val

        while current <= end_val:
            values.append(round(current, 6))
            current += step

    elif mode == 2:

        n_files = int(input("Number of STL files: "))

        if n_files == 1:
            values = [start_val]
        else:
            step = (end_val - start_val) / (n_files - 1)

            values = [
                round(start_val + i * step, 6)
                for i in range(n_files)
            ]

    else:
        print("Invalid option")
        quit()

    print("\nGenerating files...\n")

    for value in values:

        id_val  = inner_dia
        dd_val  = disc_dia
        off_val = offset
        bod_val = backbone_outer_dia
        bl_val  = backbone_length

        if choice == 1:
            id_val = value
            filename = f"CD_{value:.2f}.stl"

        elif choice == 2:
            dd_val = value
            filename = f"RD_{value:.2f}.stl"

        elif choice == 3:
            off_val = value
            filename = f"OF_{value:.2f}.stl"

        elif choice == 4:
            bod_val = value
            filename = f"BD_{value:.2f}.stl"

        elif choice == 5:
            bl_val = value
            filename = f"BL_{value:.1f}.stl"

        else:
            print("Invalid parameter choice")
            quit()

        model = create_model(
            inner_dia=id_val,
            disc_dia=dd_val,
            offset=off_val,
            backbone_outer_dia=bod_val,
            backbone_length=bl_val
        )

        cq.exporters.export(model, filename)

        print(f"  Saved: {filename}")

    print("\nAll files saved successfully.")
    