#!/usr/bin/env python3
"""
Import Custom Drone
===================
Automates the integration of a custom UAV model into the PX4 SITL + Python
simulation pipeline.

Given:
  - A Gazebo SDF model directory (from the mechanical team)
  - An optional PX4 airframe file

This script will:
  1. Copy the SDF model into ~/PX4-Autopilot/Tools/simulation/gz/models/
  2. Copy the airframe file into ~/PX4-Autopilot/ROMFS/.../airframes/
  3. Parse the SDF to extract physical parameters (mass, inertia, arm_length)
  4. Generate/update src/uav_model/parameters.yaml with the extracted values

Usage:
  python3 scripts/import_custom_drone.py \\
      --model-dir /path/to/my_drone_model/ \\
      [--airframe /path/to/4099_my_drone] \\
      [--output-yaml src/uav_model/parameters.yaml] \\
      [--dry-run]
"""

import argparse
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


def find_px4_autopilot() -> Path:
    """Locate the PX4-Autopilot installation directory."""
    candidates = [
        Path.home() / "PX4-Autopilot",
        Path("/opt/PX4-Autopilot"),
    ]
    for p in candidates:
        if p.is_dir():
            return p
    return None


def parse_sdf_parameters(sdf_path: Path) -> dict:
    """
    Extract physical parameters from a Gazebo SDF model file.

    Searches for:
      - <mass> element → mass (kg)
      - <inertia> elements → Ixx, Iyy, Izz (kg·m²)
      - <joint> elements with 'rotor' in the name → arm_length estimate
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Handle SDF namespace
    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}")[0] + "}"

    params = {
        "mass": None,
        "inertia": {"Ixx": None, "Iyy": None, "Izz": None},
        "arm_length": 0.25,  # default
        "gravity": 9.81,
        "drag_coefficient": 0.01,
        "motor": {"k_thrust": 1.0e-5, "k_drag": 1.2e-7},
        "limits": {"max_tilt_angle": 0.7854, "max_velocity": 12.0},
        "simulation": {"dt": 0.005},
    }

    # Find all <link> elements and pick the one with the largest mass
    # (usually base_link)
    max_mass = 0.0
    best_inertial = None

    for link in root.iter(f"{ns}link"):
        inertial = link.find(f"{ns}inertial")
        if inertial is None:
            continue
        mass_el = inertial.find(f"{ns}mass")
        if mass_el is not None:
            m = float(mass_el.text)
            if m > max_mass:
                max_mass = m
                best_inertial = inertial

    if best_inertial is not None:
        params["mass"] = max_mass
        inertia_el = best_inertial.find(f"{ns}inertia")
        if inertia_el is not None:
            for tag in ["ixx", "iyy", "izz"]:
                el = inertia_el.find(f"{ns}{tag}")
                if el is not None:
                    params["inertia"][tag.capitalize()] = float(el.text)

    # Try to estimate arm_length from joint positions
    joints = list(root.iter(f"{ns}joint"))
    rotor_positions = []
    for joint in joints:
        name = joint.get("name", "")
        if "rotor" in name.lower() or "motor" in name.lower():
            child_link = joint.find(f"{ns}child")
            if child_link is not None:
                pose = joint.find(f"{ns}pose")
                if pose is not None:
                    parts = pose.text.strip().split()
                    if len(parts) >= 3:
                        x, y = float(parts[0]), float(parts[1])
                        dist = (x**2 + y**2) ** 0.5
                        rotor_positions.append(dist)

    if rotor_positions:
        params["arm_length"] = round(sum(rotor_positions) / len(rotor_positions), 4)

    return params


def write_parameters_yaml(params: dict, output_path: Path):
    """Write extracted parameters to a YAML file."""
    # Clean None values
    clean = {}
    for k, v in params.items():
        if isinstance(v, dict):
            clean[k] = {kk: vv for kk, vv in v.items() if vv is not None}
        elif v is not None:
            clean[k] = v

    with open(output_path, "w") as f:
        yaml.dump(clean, f, default_flow_style=False, sort_keys=False)
    print(f"  ✅ Parameters written to: {output_path}")


def copy_model(model_dir: Path, px4_dir: Path, dry_run: bool = False):
    """Copy the SDF model directory into PX4's Gazebo models folder."""
    dest = px4_dir / "Tools" / "simulation" / "gz" / "models" / model_dir.name
    if dry_run:
        print(f"  [DRY-RUN] Would copy {model_dir} → {dest}")
        return dest
    if dest.exists():
        print(f"  ⚠️  Destination exists, overwriting: {dest}")
        shutil.rmtree(dest)
    shutil.copytree(model_dir, dest)
    print(f"  ✅ Model copied to: {dest}")
    return dest


def copy_airframe(airframe_path: Path, px4_dir: Path, dry_run: bool = False):
    """Copy the airframe file into PX4's airframes directory."""
    dest_dir = (
        px4_dir / "ROMFS" / "px4fmu_common" / "init.d-posix" / "airframes"
    )
    dest = dest_dir / airframe_path.name
    if dry_run:
        print(f"  [DRY-RUN] Would copy {airframe_path} → {dest}")
        return
    shutil.copy2(airframe_path, dest)
    print(f"  ✅ Airframe copied to: {dest}")


def main():
    parser = argparse.ArgumentParser(
        description="Import a custom UAV model into PX4 SITL + Python pipeline.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Import model + extract parameters:
  python3 scripts/import_custom_drone.py --model-dir ~/my_drone_model/

  # Import with custom airframe:
  python3 scripts/import_custom_drone.py \\
      --model-dir ~/my_drone_model/ \\
      --airframe ~/4099_my_drone

  # Dry-run (preview only):
  python3 scripts/import_custom_drone.py --model-dir ~/my_drone_model/ --dry-run
        """,
    )
    parser.add_argument(
        "--model-dir",
        type=Path,
        required=True,
        help="Path to the Gazebo SDF model directory (must contain model.sdf)",
    )
    parser.add_argument(
        "--airframe",
        type=Path,
        default=None,
        help="Path to the PX4 airframe file (optional)",
    )
    parser.add_argument(
        "--output-yaml",
        type=Path,
        default=None,
        help="Output path for parameters.yaml (default: src/uav_model/parameters.yaml)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview actions without making changes",
    )

    args = parser.parse_args()

    # Resolve project root
    project_root = Path(__file__).resolve().parents[1]
    if args.output_yaml is None:
        args.output_yaml = project_root / "src" / "uav_model" / "parameters.yaml"

    print("=" * 60)
    print("  Import Custom Drone into PX4 SITL Pipeline")
    print("=" * 60)

    # Validate model directory
    if not args.model_dir.is_dir():
        print(f"❌ Error: Model directory not found: {args.model_dir}")
        sys.exit(1)

    sdf_file = args.model_dir / "model.sdf"
    if not sdf_file.exists():
        print(f"❌ Error: model.sdf not found in {args.model_dir}")
        sys.exit(1)

    # Find PX4-Autopilot
    px4_dir = find_px4_autopilot()
    if px4_dir is None:
        print("⚠️  PX4-Autopilot not found. Skipping model/airframe copy.")
        print("   Set PX4_DIR environment variable or install to ~/PX4-Autopilot")
    else:
        print(f"\n📂 PX4-Autopilot found at: {px4_dir}")

    # Step 1: Parse SDF
    print(f"\n🔍 Parsing SDF model: {sdf_file}")
    params = parse_sdf_parameters(sdf_file)

    if params["mass"] is not None:
        print(f"   Mass:       {params['mass']:.3f} kg")
        print(f"   Ixx:        {params['inertia'].get('Ixx', 'N/A')}")
        print(f"   Iyy:        {params['inertia'].get('Iyy', 'N/A')}")
        print(f"   Izz:        {params['inertia'].get('Izz', 'N/A')}")
        print(f"   Arm length: {params['arm_length']:.4f} m")
    else:
        print("   ⚠️  Could not extract mass from SDF. Using defaults.")
        params["mass"] = 1.535

    # Step 2: Write parameters.yaml
    print("\n📝 Writing parameters.yaml")
    if not args.dry_run:
        write_parameters_yaml(params, args.output_yaml)
    else:
        print(f"  [DRY-RUN] Would write to: {args.output_yaml}")

    # Step 3: Copy model to PX4
    if px4_dir:
        print("\n📦 Copying model to PX4-Autopilot")
        copy_model(args.model_dir, px4_dir, dry_run=args.dry_run)

    # Step 4: Copy airframe (if provided)
    if args.airframe:
        if not args.airframe.is_file():
            print(f"❌ Error: Airframe file not found: {args.airframe}")
            sys.exit(1)
        if px4_dir:
            print("\n📦 Copying airframe to PX4-Autopilot")
            copy_airframe(args.airframe, px4_dir, dry_run=args.dry_run)
        else:
            print("   ⚠️  Skipping airframe copy (PX4-Autopilot not found)")

    print("\n" + "=" * 60)
    print("  ✅ Import complete!")
    print(f"     Next: rebuild PX4 with 'make px4_sitl gz_{args.model_dir.name}'")
    print("=" * 60)


if __name__ == "__main__":
    main()
