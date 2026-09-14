#!/usr/bin/env bash
# ============================================================================
#  Kéo 2 repo bên thứ ba về src/ và áp patch.
#  Chạy trên HOST (chỉ cần git), không cần container.
#
#  Hai repo này nằm trong .gitignore nên KHÔNG có trong repo của bạn.
#  Sau khi clone repo này về máy mới, chạy script này trước khi build.
# ============================================================================
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$WS/src"

clone_or_update () {
    local url=$1 branch=$2 dir=$3
    if [ -d "$SRC/$dir/.git" ]; then
        echo ">>> [$dir] đã có, cập nhật..."
        git -C "$SRC/$dir" fetch --all --tags
        git -C "$SRC/$dir" checkout "$branch"
        git -C "$SRC/$dir" submodule update --init --recursive
    else
        echo ">>> [$dir] clone nhánh $branch ..."
        git clone -b "$branch" --recurse-submodules "$url" "$SRC/$dir"
    fi
}

# --- 1. Driver Ouster --------------------------------------------------------
# BẮT BUỘC nhánh 'ros2-foxy'. Nhánh 'ros2' (mặc định) chỉ hỗ trợ Humble trở lên
# và KHÔNG build được trên Foxy.
# Nhánh ros2-foxy đang ở v0.12.7, dùng ouster_client v0.10.0 — có profile FuSA
# cho firmware 3.1+, tức chạy được sensor Rev7 như OS-1-32-U2-SR (FW 3.1.0).
clone_or_update https://github.com/ouster-lidar/ouster-ros.git ros2-foxy ouster-ros

# --- 2. FAST-LIO2 ------------------------------------------------------------
# Nhánh 'ROS2' VIẾT HOA. Nhánh mặc định là bản ROS 1.
# --recurse-submodules bắt buộc: ikd-Tree là submodule và là trái tim của
# FAST-LIO2 (cây k-d tăng dần).
clone_or_update https://github.com/hku-mars/FAST_LIO.git ROS2 FAST_LIO

# --- 3. Áp patch cho FAST_LIO ------------------------------------------------
# 3 sửa đổi bắt buộc để chạy trên Foxy với Ouster. Xem patches/README.md.
PATCH="$WS/patches/0001-fast_lio-foxy-ouster.patch"
if git -C "$SRC/FAST_LIO" apply --check "$PATCH" 2>/dev/null; then
    git -C "$SRC/FAST_LIO" apply "$PATCH"
    echo ">>> Đã áp patch FAST_LIO"
elif git -C "$SRC/FAST_LIO" apply --reverse --check "$PATCH" 2>/dev/null; then
    echo ">>> Patch FAST_LIO đã được áp từ trước, bỏ qua"
else
    echo ">>> CẢNH BÁO: không áp được patch. Upstream có thể đã đổi." >&2
    echo "    Xem patches/README.md để sửa tay 3 chỗ." >&2
fi

echo
echo ">>> Xong. Nội dung src/:"
ls -1 "$SRC"
