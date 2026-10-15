"""Minimal 2-Layer 3D Optical Router Toolchain Smoke Test

Verifies:
1. gdsfactory imports and can construct multi-layer photonic cells.
2. GDSII export succeeds and writes valid geometry to disk.
3. KLayout's database engine (klayout.db) successfully parses and verifies the GDSII file.
"""

from pathlib import Path
import gdsfactory as gf
import klayout.db as kdb


def test_preliminary_3d_export():
    # Activate generic PDK for layer resolution
    try:
        gf.generic_tech.get_generic_pdk().activate()
    except AttributeError:
        gf.gpdk.PDK.activate()

    output_dir = Path("output")
    output_dir.mkdir(parents=True, exist_ok=True)
    gds_path = output_dir / "preliminary_3d_test.gds"

    # Create top-level component
    c = gf.Component("preliminary_3d_test")

    # Define two photonic tiers as distinct GDS layers: (Layer, DataType)
    LAYER_TIER1 = (1, 0)  # Lower photonic tier
    LAYER_TIER2 = (2, 0)  # Upper photonic tier
    LAYER_VIA   = (3, 0)  # Inter-tier optical coupler / via

    # Tier 1 geometry: lower waveguide (e.g. 500nm width, 20um length)
    c.add_polygon(
        points=[(0.0, -0.25), (20.0, -0.25), (20.0, 0.25), (0.0, 0.25)],
        layer=LAYER_TIER1,
    )

    # Tier 2 geometry: upper waveguide overlapping and routing orthogonal
    c.add_polygon(
        points=[(10.0, 0.0), (10.0, 20.0), (10.5, 20.0), (10.5, 0.0)],
        layer=LAYER_TIER2,
    )

    # Inter-tier vertical coupler window
    c.add_polygon(
        points=[(9.75, -0.5), (10.75, -0.5), (10.75, 0.5), (9.75, 0.5)],
        layer=LAYER_VIA,
    )

    # Write GDSII
    c.write_gds(gds_path)
    assert gds_path.exists(), f"GDSII file not found at {gds_path}"
    assert gds_path.stat().st_size > 0, "GDSII file is empty"

    # Verify KLayout database parsing without shared library or syntax issues
    layout = kdb.Layout()
    layout.read(str(gds_path))
    top_cell = layout.top_cell()
    assert top_cell is not None, "KLayout could not find top cell in exported GDSII"
    assert top_cell.name == "preliminary_3d_test", f"Top cell name mismatch: {top_cell.name}"

    # Verify that layers 1, 2, and 3 exist in the layout
    layer_indices = {layer_info.layer: layer_info for layer_info in layout.layer_infos()}
    assert 1 in layer_indices, "Layer 1 (Tier 1) missing in KLayout database"
    assert 2 in layer_indices, "Layer 2 (Tier 2) missing in KLayout database"
    assert 3 in layer_indices, "Layer 3 (Inter-tier via) missing in KLayout database"

    print(f"\n[SUCCESS] GDSII smoke test passed! Output written to {gds_path.resolve()}")
    print(f"Top cell: {top_cell.name}, Bounding Box: {top_cell.bbox()}")
    print(f"Verified layers: {list(layer_indices.keys())}")


if __name__ == "__main__":
    test_preliminary_3d_export()
