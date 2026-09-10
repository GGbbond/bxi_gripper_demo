#!/usr/bin/env bash
set -euo pipefail

marker="__BXI_GRIPPER_ARCHIVE_BELOW__"
line="$(awk -v marker="$marker" '$0 == marker { print NR + 1; exit }' "$0")"
if [[ -z "$line" ]]; then
    echo "安装包损坏：未找到数据段。" >&2
    exit 1
fi

temp_dir="$(mktemp -d)"
cleanup() { rm -rf -- "$temp_dir"; }
trap cleanup EXIT
tail -n "+$line" "$0" | tar -xz -C "$temp_dir"
bash "$temp_dir/bxi_gripper_demo/install.sh" "$@"
exit 0
__BXI_GRIPPER_ARCHIVE_BELOW__
