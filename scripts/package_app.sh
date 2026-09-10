#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")/.." && pwd)"
cd "$project_dir"

bash scripts/build_backend.sh
if ! python3 -m PyInstaller --version >/dev/null 2>&1; then
    echo "缺少 PyInstaller，请在当前 Python 环境执行：" >&2
    echo "python3 -m pip install pyinstaller" >&2
    exit 1
fi

python3 -m PyInstaller \
    --noconfirm \
    --clean \
    --windowed \
    --onedir \
    --name bxi_gripper_demo \
    --add-data "build/bin/gripper_backend:build/bin" \
    --add-data "assets/gripper_demo.svg:assets" \
    --add-data "assets/checkbox_check.svg:assets" \
    gripper_demo.py

stage="$project_dir/dist/package_stage/bxi_gripper_demo"
rm -rf -- "$project_dir/dist/package_stage"
mkdir -p "$stage/app"
cp -a "$project_dir/dist/bxi_gripper_demo/." "$stage/app/"
cp "$project_dir/packaging/install.sh" "$stage/install.sh"
cp "$project_dir/README.md" "$stage/README.md"
cp "$project_dir/assets/gripper_demo.svg" "$stage/gripper_demo.svg"
chmod +x "$stage/install.sh" "$stage/app/bxi_gripper_demo" \
    "$stage/app/_internal/build/bin/gripper_backend"

archive="$project_dir/dist/bxi_gripper_demo-linux-x86_64.tar.gz"
installer="$project_dir/dist/bxi_gripper_demo-linux-x86_64-installer.run"
tar -czf "$archive" -C "$project_dir/dist/package_stage" bxi_gripper_demo
cp "$project_dir/packaging/installer-header.sh" "$installer"
cat "$archive" >> "$installer"
chmod +x "$installer"

echo "客户安装包：$installer"
echo "便携压缩包：$archive"
