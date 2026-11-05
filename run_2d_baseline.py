"""2D Planar Baseline Router for PICBench Benes 4x4 Topology."""

import json
from pathlib import Path
import gdsfactory as gf

def build_2d_benes4x4():
    # Activate generic technology PDK
    gf.generic_tech.get_generic_pdk().activate()

    # Load PICBench netlist
    netlist_path = Path("external/PICBench/testcases/Benes_4x4/Benes_4x4_ref.json")
    with open(netlist_path, "r") as f:
        data = json.load(f)["netlist"]

    c = gf.Component("benes_4x4_2d_baseline")

    # 1. Component Placement
    # In a Benes 4x4: 3 stages of 2 OSUs each.
    # Stage 0: OSU1, OSU2 | Stage 1: OSU3, OSU4 | Stage 2: OSU5, OSU6
    stage_x_pitch = 300.0  # um between stages
    osu_y_pitch   = 100.0  # um between switches in a stage

    osu_coords = {
        "OSU1": (0.0 * stage_x_pitch,  0.5 * osu_y_pitch),
        "OSU2": (0.0 * stage_x_pitch, -0.5 * osu_y_pitch),
        "OSU3": (1.0 * stage_x_pitch,  0.5 * osu_y_pitch),
        "OSU4": (1.0 * stage_x_pitch, -0.5 * osu_y_pitch),
        "OSU5": (2.0 * stage_x_pitch,  0.5 * osu_y_pitch),
        "OSU6": (2.0 * stage_x_pitch, -0.5 * osu_y_pitch),
    }

    # Use a standard 2x2 MZI switch cell
    switch_cell = gf.components.mzi2x2_2x2()
    instances = {}
    for name, (x, y) in osu_coords.items():
        ref = c.add_ref(switch_cell, name=name)
        ref.move((x, y))
        instances[name] = ref

    # 2. 2D Routing
    # Map netlist connections: e.g. "OSU1,O1": "OSU3,I1"
    # Port mapping for mzi2x2: inputs are o1, o2 (west); outputs are o3, o4 (east)
    port_map = {"I1": "o1", "I2": "o2", "O1": "o3", "O2": "o4"}

    routes = []
    for src_str, dst_str in data["connections"].items():
        src_inst, src_p = src_str.split(",")
        dst_inst, dst_p = dst_str.split(",")

        p1 = instances[src_inst].ports[port_map[src_p]]
        p2 = instances[dst_inst].ports[port_map[dst_p]]

        # Standard 2D curvy route
        route = gf.routing.route_single(c, p1, p2, radius=10.0, layer = 0, route_width = 5)
        routes.append(route)

    # 3. Export to GDSII
    out_dir = Path("output")
    out_dir.mkdir(exist_ok=True)
    out_gds = out_dir / "benes_4x4_2d_baseline.gds"
    c.write_gds(out_gds)
    print(f"[SUCCESS] 2D Baseline exported to: {out_gds.resolve()}")
    return out_gds

if __name__ == "__main__":
    build_2d_benes4x4()