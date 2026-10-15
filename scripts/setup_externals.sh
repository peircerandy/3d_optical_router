#!/usr/bin/env bash
set -euo pipefail

# Clone external sub-projects if not already present
mkdir -p external

if [ ! -d "external/LiDAR" ]; then
    echo "Cloning ScopeX LiDAR repository..."
    git clone https://github.com/ScopeX-ASU/LiDAR.git external/LiDAR
else
    echo "external/LiDAR already exists."
fi

if [ ! -d "external/PICBench" ]; then
    echo "Cloning PICBench repository..."
    git clone https://github.com/PICDA/PICBench.git external/PICBench
else
    echo "external/PICBench already exists."
fi

echo "[SUCCESS] External repositories ready."
