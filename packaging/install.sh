#!/usr/bin/env bash
set -euo pipefail

app_id="bxi_gripper_demo"
app_name="BXI 夹爪控制演示"
install_dir="$HOME/.local/share/$app_id"
desktop_dir="$HOME/.local/share/applications"
script_dir="$(cd "$(dirname "$0")" && pwd)"

if [[ "${1:-}" == "--uninstall" ]]; then
    rm -rf -- "$install_dir"
    rm -f -- "$desktop_dir/$app_id.desktop"
    echo "$app_name 已卸载。用户动作文件和本机设置未删除。"
    exit 0
fi

if [[ ! -x "$script_dir/app/bxi_gripper_demo" ]]; then
    echo "安装包内容不完整：找不到 app/bxi_gripper_demo" >&2
    exit 1
fi
if ! command -v pkexec >/dev/null 2>&1; then
    echo "缺少 pkexec。请先安装 polkit。" >&2
    exit 1
fi

mkdir -p "$install_dir" "$desktop_dir"
rm -rf -- "$install_dir/app"
cp -a "$script_dir/app" "$install_dir/app"

desktop_file="$desktop_dir/$app_id.desktop"
{
    echo "[Desktop Entry]"
    echo "Type=Application"
    echo "Name=$app_name"
    echo "Comment=BXI 50L 电机夹爪控制与动作演示"
    echo "Exec=$install_dir/app/bxi_gripper_demo"
    echo "Icon=$install_dir/gripper_demo.svg"
    echo "Terminal=false"
    echo "Categories=Utility;Development;"
} > "$desktop_file"
cp "$script_dir/gripper_demo.svg" "$install_dir/gripper_demo.svg"
chmod +x "$desktop_file"

echo "$app_name 安装完成。"
echo "可从应用菜单启动，或执行："
echo "$install_dir/app/bxi_gripper_demo"
