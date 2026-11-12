# 3D Optical Router: Partner Onboarding & Engineering Roadmap
**Course:** ECE 544 Silicon Photonics Design  
**Project:** Automated Multi-Tier 3D Silicon Photonic Detailed Routing (Si + SiN)  
**Repository:** `3d_optical_router`  
**Current Branch:** `main`

---

## 1. Executive Summary & Project Mission

### The Core Problem
In Electronic Integrated Circuits (EICs), routing wires can turn at sharp $90^\circ$ angles and jump between 10+ metal layers with vertical vias. In **Photonic Integrated Circuits (PICs)**:
1. **Waveguides must curve smoothly** (Euler/circular bends) with a strict minimum bend radius ($R_{\text{min}}$) to prevent optical radiation loss.
2. In standard **2D single-layer PICs**, waveguides cannot jump layers. When two optical paths intersect, an optical **waveguide crossing component** must be inserted. Each planar crossing incurs $\sim 0.2\text{--}0.5\,\text{dB}$ insertion loss and optical crosstalk. For large switch topologies (Benes, Clements, Spanke, Crossbar), the crossing penalty accumulates rapidly, destroying optical signal-to-noise ratio (OSNR).

### Our 3D Solution
We are developing an automated **Electronic-Photonic Design Automation (EPDA)** router for **3D Multi-Tier Silicon Photonics**:
- **Layer 0 (Silicon - Active Tier):** Contains active devices (MZIs, thermal phase shifters, photodetectors). High index contrast ($\Delta n$), compact $R_{\text{min}} \approx 5\,\mu\text{m}$.
- **Layer 1 (Silicon Nitride - Passive Routing Tier):** Ultra-low propagation loss ($< 0.1\,\text{dB/cm}$), zero two-photon absorption at high optical power. Moderate index contrast ($R_{\text{min}} \approx 20\text{--}25\,\mu\text{m}$).
- **Inter-tier Transitions (3D Optical Vias):** Specialized tapered vertical couplers that transfer optical modes between Si and SiN with low loss ($\sim 0.1\text{--}0.2\,\text{dB}$) and near-zero crosstalk.

> **Project Goal:** Build and validate an automated router (`router3d`) that routes complex PIC netlists across Si and SiN tiers, replacing lossy 2D waveguide crossings with low-loss 3D inter-layer transitions to minimize total path insertion loss.

---

## 2. Platform Comparison: Si vs. SiN Asymmetric Layers

Because the two physical layers have drastically different material properties, our router cannot treat layers identically (unlike electronic metal layers):

| Parameter | Bottom Tier (Silicon Core) | Top Tier (Silicon Nitride) | Inter-Tier Coupler (3D Via) |
| :--- | :--- | :--- | :--- |
| **Material** | Crystalline Silicon ($\text{Si}$, $n \approx 3.47$) | Silicon Nitride ($\text{Si}_3\text{N}_4$, $n \approx 2.0$) | Adiabatic / Directional Taper |
| **GDS Layer** | Layer `(1, 0)` | Layer `(2, 0)` | Overlaps `(1, 0)` & `(2, 0)` |
| **Min Bend Radius ($R_{\text{min}}$)** | $\mathbf{5.0\,\mu m}$ | $\mathbf{20.0\text{--}25.0\,\mu m}$ | N/A (Linear / Parabolic taper) |
| **Propagation Loss** | $\sim 1.5\text{--}2.0\,\text{dB/cm}$ | $\mathbf{< 0.1\,\text{dB/cm}}$ | $\sim 0.1\text{--}0.2\,\text{dB/transition}$ |
| **Waveguide Width** | $0.5\,\mu\text{m}$ (single mode @ 1550 nm) | $0.8\text{--}1.0\,\mu\text{m}$ | $0.5\,\mu\text{m} \to 0.15\,\mu\text{m}$ tip |
| **Coupler Footprint** | N/A | N/A | $\approx 20\text{--}40\,\mu\text{m}$ length |
| **Primary Function** | Active devices, input/output ports | Overpass routing, long hauls | Layer transfer |

---

## 3. Current Codebase Architecture

```
3d_optical_router/
├── external/
│   ├── LiDAR/               # 2D reference router (ISPD 2025). UNTOUCHED BASELINE.
│   └── PICBench/            # Benchmark topologies (Benes, Clements, Crossbar, etc.)
├── run_2d_baseline.py       # Baseline script: GDSFactory point-to-point (shows 2D collisions)
├── run_3d_router.py         # Root launcher for our 3D router
└── src/
    └── router3d/            # OUR ACTIVE WORKING PACKAGE (adapted from LiDAR)
        ├── config/          # Router parameters (radii, grid resolution, loss weights)
        ├── database/        # Schematic netlist, macro definitions, port coordinates
        ├── drc/             # Spatial occupancy grid (Bitmap) and design rules
        ├── routing/         # A* pathfinding and Negotiated Rip-Up & Reroute (NRR)
        ├── queue/           # Priority queues for net ordering
        └── utils/           # Euler clothoid smoothing and s-bend port connections
```

### The Three Routing Stages in the Repo
```mermaid
flowchart LR
    A["Level 1: Naive 2D<br>(run_2d_baseline.py)"] -->|Add 2D DRC & Crossing Insertion| B["Level 2: SOTA 2D LiDAR<br>(external/LiDAR)"]
    B -->|Extend to (x,y,θ,z) + Asymmetric Tiers| C["Level 3: Our 3D Router<br>(src/router3d)"]
```

1. **`run_2d_baseline.py` (Naive 2D):** Uses `gdsfactory.routing.route_single`. Fast, but waveguides cross each other without crossing components, producing physical short-circuits.
2. **`external/LiDAR` (SOTA 2D Planar):** Curvy-aware A* on $(x, y, \theta)$. Correctly places 2D crossing components, but incurs heavy crossing losses on dense topologies.
3. **`src/router3d` (Our Work):** Extends graph search into 3D space $(x, y, \theta, z)$, optimizing layer transitions to replace crossings with vertical couplers.

---

## 4. Technical Work Breakdown Structure (What We Need to Build)

### Milestone 1: Multi-Tier Grid & DRC Engine
- **Files to modify:** [`src/router3d/drc/bitmap.py`](file:///home/randy/school/ECE544SiliconPhotonicsDesign/3d_optical_router/src/router3d/drc/bitmap.py) and [`src/router3d/drc/drcmanager.py`](file:///home/randy/school/ECE544SiliconPhotonicsDesign/3d_optical_router/src/router3d/drc/drcmanager.py)
- **Objective:** Support 3D occupancy and tier-specific design rules.
- **Tasks:**
  1. Upgrade `Bitmap` from 2D `(width, height)` to multi-tier `(width, height, num_layers=2)`.
  2. Implement tier-specific obstacle checks:
     - On Layer 0 (Si): Blocks include active devices + Si waveguides + vertical coupler landing pads.
     - On Layer 1 (SiN): Blocks include SiN waveguides + vertical coupler upper pads.
  3. Model the **3D Via Keep-Out**: When a vertical coupler is placed, mark the grid cells as occupied on **both Layer 0 and Layer 1** simultaneously.

---

### Milestone 2: 3D Curvy-Aware A* Search
- **Files to modify:** [`src/router3d/routing/astarsearch.py`](file:///home/randy/school/ECE544SiliconPhotonicsDesign/3d_optical_router/src/router3d/routing/astarsearch.py)
- **Objective:** Allow the router to explore $(x, y, \theta, z)$ states with asymmetric tier constraints.
- **Tasks:**
  1. **Node State:** Change node identifier from `(x, y, orient)` to `(x, y, orient, layer)`.
  2. **Neighbor Generation:**
     - **Intra-tier moves ($z \to z$):** Straight, $45^\circ$, or bend. Bend radius check uses $R_{\text{min}}(z)$ ($5\,\mu\text{m}$ for Si, $25\,\mu\text{m}$ for SiN).
     - **Inter-tier transition ($z \to 1-z$):** Step through a 3D via. Check via length footprint ($\sim 20\text{--}40\,\mu\text{m}$ linear clearance along current orientation) and verify clearance on both layers.
  3. **Cost Objective Function:**
     $$\text{Cost}(n) = g(n) + h(n)$$
     $$g(n) = \sum \left( \alpha_z \cdot L_{\text{prop}, z} + \beta_z \cdot N_{\text{bends}, z} + \gamma \cdot N_{\text{vias}} + \delta \cdot N_{\text{crossings}} \right)$$
     Set $\gamma$ (via cost $\approx 0.15\,\text{dB}$) substantially lower than $\delta$ (crossing cost $\approx 0.5\,\text{dB}$) so A* prefers jumping layers over cutting through waveguides.

---

### Milestone 3: 3D Post-Processing & GDSII Multi-Layer Export
- **Files to modify:** [`src/router3d/routing/drgridroute.py`](file:///home/randy/school/ECE544SiliconPhotonicsDesign/3d_optical_router/src/router3d/routing/drgridroute.py)
- **Objective:** Convert discrete grid path solution into clean multi-layer GDSII polygons.
- **Tasks:**
  1. Decompose each net's routed path into continuous single-layer sub-paths separated by 3D via nodes.
  2. For Layer 0 sub-paths: Apply Euler curve smoothing with $R = 5\,\mu\text{m}$ and extrude to GDS Layer `(1, 0)`.
  3. For Layer 1 sub-paths: Apply Euler curve smoothing with $R = 25\,\mu\text{m}$ and extrude to GDS Layer `(2, 0)`.
  4. At each layer transition: Instantiate a vertical coupler cell (from GDSFactory / UBCPDK or parameterized adiabatic taper) connecting Layer 0 and Layer 1.

---

### Milestone 4: Benchmarking & Quantitative Evaluation
- **Target Circuits (from `external/PICBench`):**
  - Benes 4x4 & 8x8 switch matrices
  - Clements 4x4 & 8x8 unitary optical meshes
  - Crossbar 4x4 & 8x8 routing networks
- **Metrics to Measure & Report:**
  1. **Total Insertion Loss (dB):** Worst-case and average path loss across all ports.
  2. **Planar Crossing Count:** Target: 0 (or near-zero) in the 3D router vs. tens/hundreds in 2D.
  3. **3D Via Count:** Number of inter-tier transitions used.
  4. **Optical Crosstalk (dB):** Signal isolation improvement over 2D crossing-heavy layouts.
  5. **Total Footprint Area ($\mu\text{m}^2$):** Die area comparison.

---

## 5. Developer Quickstart for Partner

### Step 1: Clone and Set Up Environment
```bash
git clone git@github.com:peircerandy/3d_optical_router.git
cd 3d_optical_router

# Set up Python 3.11 environment
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Step 2: Run the 2D Baseline (GDSFactory)
```bash
python3 run_2d_baseline.py
# Output saved to output/benes_4x4_2d_baseline.gds
# Inspect in KLayout: observe waveguides overlapping without crossings
```

### Step 3: Run the Router Baseline (router3d)
```bash
python3 run_3d_router.py \
  --benchmark=external/LiDAR/src/picroute/benchmarks/clements_8x8/clements_8x8.yml \
  --config=src/router3d/config/router3d_default.yml
# Output saved to output/router3d_result.gds
```

---

## 6. Suggested Division of Labor

```
┌───────────────────────────────────────┬───────────────────────────────────────┐
│              Partner A                │              Partner B                │
├───────────────────────────────────────┼───────────────────────────────────────┤
│ • Milestone 1: Multi-layer Bitmap     │ • Milestone 2: 3D A* Search Engine    │
│   (bitmap.py & drcmanager.py)         │   (astarsearch.py state & neighbors)  │
│ • Modeling 3D Via keep-outs           │ • Asymmetric cost function tuning     │
│ • Netlist converter (PICBench JSON    │ • Milestone 3: GDSFactory multi-layer │
│   to router3d YAML)                   │   instantiation & coupler cell export │
│ • Optical simulation (SAX/JAX loss)   │ • Benchmark layout generation & plots │
└───────────────────────────────────────┴───────────────────────────────────────┘
```
