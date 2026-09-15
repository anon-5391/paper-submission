import cadquery as cq
import math


def create_model(
    inner_dia,
    disc_dia,
    offset,
    backbone_outer_dia=25,
    backbone_length=100,
    tendon_hole_dist=9.5
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

    usable_length = backbone_length - bottom_margin - top_margin

    # Estimate notch count using nominal pitch
    nominal_pitch = cut_height + clearance
    num_notches = max(1, int((usable_length - cut_height) // nominal_pitch) + 1)

    # Force first and last notch edges to be exactly 5 mm from the ends
    first_center = bottom_margin + cut_height / 2
    last_center = backbone_length - top_margin - cut_height / 2

    if num_notches == 1:
        positions = [first_center]
        pitch = 0
    else:
        pitch = (last_center - first_center) / (num_notches - 1)
        positions = [
            first_center + i * pitch
            for i in range(num_notches)
        ]

    hourglass_positions = [
        (positions[i] + positions[i + 1]) / 2
        for i in range(len(positions) - 1)
    ]

    print("num_notches =", num_notches)
    print("pitch =", pitch)
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
    # TENDON HOLES (CUT THROUGH ENTIRE LENGTH)
    # =====================================

    tendon_holes = (
        cq.Workplane("XY")
        .polarArray(
            radius=tendon_hole_dist,
            startAngle=0,
            angle=360,
            count=4
        )
        .circle(1)
        .extrude(backbone_length + 10)
    )

    result = result.cut(tendon_holes)


    
    # FINISH
   
   
    result = result.clean()

    return result


def validate_all(id_val, dd_val, off_val, bod_val, bl_val, thd_val):
    """
    Validates all parameter constraints to prevent structural and CAD geometry errors.
    """
    errors = []
    
    # 1. Central Cylinder vs Outer Disc
    if id_val >= dd_val:
        errors.append(f"Central Cylinder Diameter ({id_val} mm) must be less than Outer Disc Diameter ({dd_val} mm).")
    
    # 2. Central Cylinder vs Backbone Outer Diameter
    if id_val >= bod_val:
        errors.append(f"Central Cylinder Diameter ({id_val} mm) must be less than Backbone Outer Diameter ({bod_val} mm).")
        
    # 3. Rolling Disc vs Backbone Outer Diameter
    if dd_val <= bod_val:
        errors.append(f"Rolling Disc Diameter ({dd_val} mm) must be greater than Backbone Outer Diameter ({bod_val} mm).")
        
    # 4. Tendon Hole Distance vs Backbone Radius (min 2mm less than outer radius)
    outer_r = bod_val / 2
    max_tendon = outer_r - 2.0
    if thd_val > max_tendon:
        errors.append(f"Tendon Hole Distance ({thd_val} mm) must be at least 2 mm less than the backbone outer radius ({outer_r:.2f} mm). Maximum allowed distance is {max_tendon:.2f} mm.")
        
    # 5. Tendon Hole Distance vs Central Cylinder Radius (must clear central cylinder wall + tendon hole radius)
    inner_r = id_val / 2
    min_tendon = inner_r + 1.0
    if thd_val < min_tendon:
        errors.append(f"Tendon Hole Distance ({thd_val} mm) must be greater than the central cylinder radius ({inner_r:.2f} mm) plus the tendon hole radius (1.0 mm). Minimum allowed distance is {min_tendon:.2f} mm.")
        
    # 6. Backbone Length vs Disc Diameter
    if bl_val <= dd_val:
        errors.append(f"Backbone Length ({bl_val} mm) must be greater than Rolling Disc Diameter ({dd_val} mm).")
        
    return errors


if __name__ == "__main__":

    # DEFAULT VALUES
    

    DEFAULT_INNER_DIA      = 14
    DEFAULT_DISC_DIA       = 30
    DEFAULT_OFFSET         = 2
    DEFAULT_BACKBONE_OUTER = 25
    DEFAULT_BACKBONE_LEN   = 100
    DEFAULT_TENDON_DIST    = 9.5

    print("\n" + "=" * 55)
    print("  TDCR Rolling-Disc Backbone Generator (Parametric)")
    print("=" * 55)

    print("\n--- Parameter to Vary ---")
    print("1. Central Cylinder Diameter")
    print("2. Rolling Disc Diameter")
    print("3. Notch Offset")
    print("4. Backbone Diameter")
    print("5. Backbone Length")
    print("6. Tendon Hole Distance from Center")

    choice = int(input("\nEnter choice (1-6): "))

    # Helper to ask for inputs
    def get_fixed(prompt, default):
        value = input(f"{prompt} [Default = {default}] : ")
        if value.strip() == "":
            return default
        return float(value)

    print("\n--- Fixed Parameters ---")

    # Temporary variable initialization
    inner_dia = DEFAULT_INNER_DIA
    disc_dia = DEFAULT_DISC_DIA
    offset = DEFAULT_OFFSET
    backbone_outer_dia = DEFAULT_BACKBONE_OUTER
    backbone_length = DEFAULT_BACKBONE_LEN
    tendon_hole_dist = DEFAULT_TENDON_DIST

    # Ask in an order that enables dynamic constraints displaying integer values
    if choice != 4:
        backbone_outer_dia = get_fixed(
            "Backbone Diameter (Outer)                    ", 
            DEFAULT_BACKBONE_OUTER
        )

    if choice != 2:
        min_disc_dia = math.ceil(backbone_outer_dia)
        disc_dia = get_fixed(
            f"Rolling Disc Diameter      [must be > {min_disc_dia} mm] ", 
            DEFAULT_DISC_DIA
        )

    if choice != 1:
        max_inner_dia = math.floor(backbone_outer_dia)
        inner_dia = get_fixed(
            f"Central Cylinder Diameter  [must be < {max_inner_dia} mm] ", 
            DEFAULT_INNER_DIA
        )

    if choice != 6:
        min_tendon = inner_dia / 2 + 1.0
        max_tendon = backbone_outer_dia / 2 - 2.0
        tendon_hole_dist = get_fixed(
            f"Tendon Hole Distance       [must be {min_tendon:.1f} - {max_tendon:.1f} mm]", 
            DEFAULT_TENDON_DIST
        )

    if choice != 3:
        offset = get_fixed(
            "Notch Offset                                 ", 
            DEFAULT_OFFSET
        )

    if choice != 5:
        min_len = math.ceil(disc_dia)
        backbone_length = get_fixed(
            f"Backbone Length            [must be > {min_len} mm] ", 
            DEFAULT_BACKBONE_LEN
        )

    # ==========================
    # RANGE FOR THE VARYING PARAMETER
    # ==========================
    print("\n--- Range to Generate ---")
    if choice == 1:
        print("Varying: Central Cylinder Diameter (Inner Diameter)")
        max_allowed_inner = min(backbone_outer_dia - 1.0, (tendon_hole_dist - 1.0) * 2)
        print(f"Constraints: Must be < Backbone Dia ({backbone_outer_dia} mm) and < {max_allowed_inner:.1f} mm.")
    elif choice == 2:
        print("Varying: Rolling Disc Diameter")
        print(f"Constraints: Must be > Backbone Dia ({backbone_outer_dia} mm) and < Backbone Length ({backbone_length} mm).")
    elif choice == 3:
        print("Varying: Notch Offset")
    elif choice == 4:
        print("Varying: Backbone Diameter (Outer Diameter)")
        min_allowed_outer = max(inner_dia + 1.0, (tendon_hole_dist + 2.0) * 2)
        print(f"Constraints: Must be > Central Cylinder Dia ({inner_dia} mm) and > {min_allowed_outer:.1f} mm.")
    elif choice == 5:
        print("Varying: Backbone Length")
        print(f"Constraints: Must be > Rolling Disc Diameter ({disc_dia} mm).")
    elif choice == 6:
        print("Varying: Tendon Hole Distance from Center")
        min_tendon = inner_dia / 2 + 1.0
        max_tendon = backbone_outer_dia / 2 - 2.0
        print(f"Constraints: Must be between {min_tendon:.1f} mm and {max_tendon:.1f} mm.")

    start_val = float(input("\nStart value: "))
    end_val   = float(input("End value: "))

    # Pre-validate start and end bounds of the range
    test_start_errors = []
    test_end_errors = []

    if choice == 1:
        test_start_errors = validate_all(start_val, disc_dia, offset, backbone_outer_dia, backbone_length, tendon_hole_dist)
        test_end_errors = validate_all(end_val, disc_dia, offset, backbone_outer_dia, backbone_length, tendon_hole_dist)
    elif choice == 2:
        test_start_errors = validate_all(inner_dia, start_val, offset, backbone_outer_dia, backbone_length, tendon_hole_dist)
        test_end_errors = validate_all(inner_dia, end_val, offset, backbone_outer_dia, backbone_length, tendon_hole_dist)
    elif choice == 3:
        test_start_errors = validate_all(inner_dia, disc_dia, start_val, backbone_outer_dia, backbone_length, tendon_hole_dist)
        test_end_errors = validate_all(inner_dia, disc_dia, end_val, backbone_outer_dia, backbone_length, tendon_hole_dist)
    elif choice == 4:
        test_start_errors = validate_all(inner_dia, disc_dia, offset, start_val, backbone_length, tendon_hole_dist)
        test_end_errors = validate_all(inner_dia, disc_dia, offset, end_val, backbone_length, tendon_hole_dist)
    elif choice == 5:
        test_start_errors = validate_all(inner_dia, disc_dia, offset, backbone_outer_dia, start_val, tendon_hole_dist)
        test_end_errors = validate_all(inner_dia, disc_dia, offset, backbone_outer_dia, end_val, tendon_hole_dist)
    elif choice == 6:
        test_start_errors = validate_all(inner_dia, disc_dia, offset, backbone_outer_dia, backbone_length, start_val)
        test_end_errors = validate_all(inner_dia, disc_dia, offset, backbone_outer_dia, backbone_length, end_val)

    all_errors = list(set(test_start_errors + test_end_errors))
    if all_errors:
        print("\n[ERROR] Invalid parameter range or configuration:")
        for err in all_errors:
            print(f"  - {err}")
        print("\nPlease run the script again and input valid values.")
        quit()

    # ==========================
    # GENERATION LOGIC
    # ==========================
    print("\nGenerate by:")
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
        thd_val = tendon_hole_dist

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
        elif choice == 6:
            thd_val = value
            filename = f"THD_{value:.2f}.stl"

        model = create_model(
            inner_dia=id_val,
            disc_dia=dd_val,
            offset=off_val,
            backbone_outer_dia=bod_val,
            backbone_length=bl_val,
            tendon_hole_dist=thd_val
        )

        cq.exporters.export(model, filename)
        print(f"  Saved: {filename}")

    print("\nAll files saved successfully.")