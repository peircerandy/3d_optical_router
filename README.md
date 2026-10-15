# 3D Optical Router: Multi-Tier Silicon Photonic Detailed Routing

An Electronic-Photonic Design Automation (EPDA) framework for multi-tier 3D silicon photonics routing, developed for **ECE 544: Silicon Photonics Design**.

---

## 1. Quick Setup

Make sure you are using **Python 3.11**. You can set up your environment using either **Conda** or a standard **virtual environment (`venv`)**.

### Option A: Using Conda (Recommended)

```bash
# 1. Create environment from environment.yml
conda env create -f environment.yml

# 2. Activate environment
conda activate lidar3d

# 3. Clone external reference repositories (LiDAR & PICBench)
bash scripts/setup_externals.sh
```

### Option B: Using Python `venv`

```bash
# 1. Ensure Python 3.11 is available
python3.11 -m venv .venv

# 2. Activate virtual environment
source .venv/bin/activate

# 3. Install core dependencies
pip install --upgrade pip
pip install -r requirements.txt

# 4. Clone external reference repositories (LiDAR & PICBench)
bash scripts/setup_externals.sh
```

---

## 2. Toolchain Smoke Test

To verify that `gdsfactory`, `klayout`, `sax`, and the export pipeline function without missing C++ shared libraries:

```bash
pytest tests/test_preliminary.py
```

This exports a 2-tier preliminary layout to `output/preliminary_3d_test.gds` and verifies it against the KLayout database engine.

---

## 3. Project Structure

```
3d_optical_router/
├── environment.yml          # Conda environment definition (Python 3.11)
├── requirements.txt         # Pip requirements definition
├── .gitignore               # Git ignore rules
├── README.md                # Project documentation and onboarding
├── scripts/
│   └── setup_externals.sh   # Clones LiDAR and PICBench into external/
├── tests/
│   └── test_preliminary.py  # 2-layer GDSII export and KLayout smoke test
├── output/
│   └── preliminary_3d_test.gds  # Verified multi-tier GDSII layout
└── external/
    ├── LiDAR/               # ScopeX Lab curvy-aware detailed router
    └── PICBench/            # Benchmark circuit netlists & topologies
```

---

## 4. Key Dependencies

- **`gdsfactory` & `ubcpdk`:** Core photonic parametric layout generation and UBC SiEPIC PDK components.
- **`klayout`:** High-performance GDSII/OASIS layout verification and database engine.
- **`sax` & `jax`:** S-parameter frequency response and insertion loss circuit simulation.
- **`scipy`, `networkx`, `shapely`:** Graph modeling, A* pathfinding, and polygon intersection testing.
- **`numba`:** JIT compilation for routing performance.
