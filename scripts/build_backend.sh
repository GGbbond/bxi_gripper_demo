#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
driver_dir="${BXI_PCI_DRV_DIR:-$project_dir/backend/bxi_pci_drv}"

cmake -S "$project_dir" -B "$project_dir/build" \
    -DBXI_PCI_DRV_DIR="$driver_dir" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$project_dir/build" --parallel
echo "Backend built: $project_dir/build/bin/gripper_backend"
