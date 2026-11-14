# Architecture Decision Record: Physical Loss Calibration & Multi-Tier Schema

## 1. Context & Motivation
In 2D LiDAR (Zhou et al., ISPD 2025), cost optimization relied on a mixture of arbitrary integer weights and decoupled physical post-evaluation metrics:
- A* search used arbitrary dimensionless weights (`loss_bending: 50`, `loss_congestion: 500`, `historyCost: 1000`, `loss_crossing: 0` or `5000`).
- Final post-route evaluation used a separate parameter (`il_cross: 0.5 dB`).

For **`router3d`**, our objective is to make the A* search **directly minimize physical optical insertion loss in decibels (dB)**, matching the mathematical cost function presented in our project slides:

$$\min \sum_{i \in \mathcal{N}} \mathcal{L}(p_i)$$
$$\mathcal{L}(p_i) = \alpha_{\text{Si}} L_{\text{Si}}^{(i)} + \alpha_{\text{SiN}} L_{\text{SiN}}^{(i)} + N_{\text{cross}}^{(i)} \cdot \mathcal{L}_{\text{cross}} + N_{\text{taper}}^{(i)} \cdot \mathcal{L}_{\text{taper}} + \sum \mathcal{L}_{\text{bend}}(R_b, z_b)$$

---

## 2. Comparison: LiDAR (Legacy 2D) vs. Router3D (Multi-Tier Physical)

| Parameter | Legacy LiDAR (Arbitrary Units) | Router3D (Physical Units) | Physical Mechanism / Literature Reference |
| :--- | :--- | :--- | :--- |
| **Silicon Propagation ($\alpha_{\text{Si}}$)** | `1.5` | `1.5 dB/cm` ($1.5 \times 10^{-4}\,\text{dB}/\mu\text{m}$) | Sacher et al. [1] |
| **SiN Propagation ($\alpha_{\text{SiN}}$)** | *N/A (single tier)* | `0.15 dB/cm` ($1.5 \times 10^{-5}\,\text{dB}/\mu\text{m}$) | Xia et al. [10] |
| **Si Bend Loss ($\mathcal{L}_{\text{bend, Si}}$)** | `50` (arbitrary) | `0.03 dB / 90°` | Cherchi et al. [7] ($R_{\text{eff}} \ge 5.4\,\mu\text{m}$) |
| **SiN Bend Loss ($\mathcal{L}_{\text{bend, SiN}}$)**| *N/A (single tier)* | `0.01 dB / 90°` | Low-loss circular/Euler SiN bends |
| **Planar Crossing ($\mathcal{L}_{\text{cross}}$)** | `il_cross: 0.5`, `loss_crossing: 0` | `0.18 dB` | Zhou et al. (LiDAR) [6] calibrated |
| **Interlayer Taper ($\mathcal{L}_{\text{taper}}$)** | *N/A (single tier)* | `0.08 dB` | Li et al. [9] ($\eta = 97.6\%\text{--}99.2\%$) |
| **Coupler Footprint** | *N/A* | $55\,\mu\text{m} \times 2\,\mu\text{m}$ | Adiabatic Si/SiN taper with 700 nm oxide gap |
| **Congestion Penalty** | `500` (arbitrary) | `1.0 dB` equivalent | Equivalent to $\approx 5.5$ crossings or 12 tapers |
| **History Cost** | `1000` (arbitrary) | `2.0 dB` equivalent | Equivalent to $\approx 11$ crossings or 25 tapers |

---

## 3. Key Design Decisions

1. **Direct Decibel Optimization:**
   All routing costs accumulated in `GridAstarNode.costG` represent genuine decibels of insertion loss. No arbitrary scaling is required because Python's 64-bit IEEE 754 floats have sufficient precision ($10^{-16}$) to track $10^{-4}\,\text{dB}$ increments without roundoff drift.
2. **Layer-Indexed Geometry:**
   Because Silicon ($R_{\text{min}} = 5\,\mu\text{m}$) and Silicon Nitride ($R_{\text{min}} = 25\,\mu\text{m}$) have fundamentally different index contrasts, `bend_radius`, `waveguide_width`, and `loss_bending` are stored per layer in `layers[z]` rather than globally.
3. **The Layer Hop Rule:**
   Transitioning $\text{Si} \to \text{SiN} \to \text{Si}$ incurs $2 \times \mathcal{L}_{\text{taper}} \approx 0.16\,\text{dB}$. Because this is less than two planar crossings ($2 \times 0.18 = 0.36\,\text{dB}$), A* will naturally elevate a net to the SiN overpass whenever it bypasses $\ge 2$ planar crossings.
4. **Backwards Compatibility:**
   `il_cross: 0.18` is preserved as a top-level fallback key in the configuration YAML to prevent breaking unmodified functions in `drgridroute.py` during ongoing development.
