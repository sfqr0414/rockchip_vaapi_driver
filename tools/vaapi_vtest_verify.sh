#!/usr/bin/env bash
# build_test_vaapi.sh —— 重新编译 + 安装 rockchip VAAPI 驱动 + 测试视频
# 放置位置：工程根目录或 tools/ 下均可

set -euo pipefail

# ------------------------------------------------------------------
# 0. 参数解析（放在提权之前，--help 不需要 sudo）
# ------------------------------------------------------------------
DO_BUILD=1
DO_TEST=1
for arg in "$@"; do
    case "${arg}" in
        --no-build) DO_BUILD=0 ;;
        --no-test)  DO_TEST=0 ;;
        -h|--help)
            echo "用法: $0 [--no-build] [--no-test]"
            echo "  --no-build  跳过编译，只安装/测试已有 build 产物"
            echo "  --no-test   只编译安装，不跑测试视频"
            exit 0
            ;;
        *)
            echo "未知参数: ${arg}" >&2
            exit 1
            ;;
    esac
done

# ------------------------------------------------------------------
# 1. 权限处理：非 root 则 sudo + exec 重新执行本脚本
# ------------------------------------------------------------------
if [ "$(id -u)" -ne 0 ]; then
    echo "当前用户不是 root，正在通过 sudo 重新执行本脚本..."
    exec sudo -- bash "$(readlink -f "${BASH_SOURCE[0]}")" "$@"
fi

# ------------------------------------------------------------------
# 2. 从脚本所在目录向上查找工程根目录（含 CMakeLists.txt）
# ------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PROJECT_ROOT=""
d="${SCRIPT_DIR}"
while [ "${d}" != "/" ]; do
    if [ -f "${d}/CMakeLists.txt" ]; then
        PROJECT_ROOT="${d}"
        break
    fi
    d="$(dirname "${d}")"
done

if [ -z "${PROJECT_ROOT}" ]; then
    echo "错误：从 ${SCRIPT_DIR} 向上未找到 CMakeLists.txt" >&2
    exit 1
fi

cd "${PROJECT_ROOT}"

BUILD_DIR="${PROJECT_ROOT}/build"
DEST_DIR="/usr/lib/aarch64-linux-gnu/dri"
DEST_SO="${DEST_DIR}/rockchip_drv_video.so"
TEST_BIN="${BUILD_DIR}/vaapi_decode_test"

VIDEOS=(
    "${PROJECT_ROOT}/videos/Test Jellyfin 1080p AV1 10bit 20M.mp4"
    "${PROJECT_ROOT}/videos/Test Jellyfin 1080p AVC 20M.mp4"
    "${PROJECT_ROOT}/videos/Test Jellyfin 1080p HEVC 8bit 20M.mp4"
)

echo "脚本目录:   ${SCRIPT_DIR}"
echo "工程根目录: ${PROJECT_ROOT}"
echo

# ------------------------------------------------------------------
# 3. 编译
# ------------------------------------------------------------------
if [ "${DO_BUILD}" -eq 1 ]; then
    echo "===== [1/3] 清理并重新编译 ====="
    rm -rf "${BUILD_DIR}"
    cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
    cmake --build "${BUILD_DIR}" -j"$(nproc)"
    echo
else
    echo "===== [1/3] 跳过编译（--no-build） ====="
    echo
fi

# ------------------------------------------------------------------
# 4. 安装驱动
# ------------------------------------------------------------------
echo "===== [2/3] 安装 VAAPI 驱动 ====="
if [ ! -f "${BUILD_DIR}/rockchip_drv_video.so" ]; then
    echo "错误：未找到 ${BUILD_DIR}/rockchip_drv_video.so" >&2
    exit 1
fi
mkdir -p "${DEST_DIR}"
install -m 0755 "${BUILD_DIR}/rockchip_drv_video.so" "${DEST_SO}"
echo "已安装: ${DEST_SO}"
ls -l "${DEST_SO}"
echo

# ------------------------------------------------------------------
# 5. 测试视频
# ------------------------------------------------------------------
if [ "${DO_TEST}" -eq 0 ]; then
    echo "===== [3/3] 跳过测试（--no-test） ====="
    echo "===== 全部完成 ====="
    exit 0
fi

echo "===== [3/3] 测试视频 ====="

# 以调用者（普通用户）身份跑测试，更接近 Jellyfin/Firefox 的真实运行环境。
# 若原用户是 root，则直接用 root 跑。
TEST_USER="${SUDO_USER:-root}"
USE_RUNUSER=0
if [ "${TEST_USER}" != "root" ] && command -v runuser >/dev/null 2>&1; then
    USE_RUNUSER=1
fi

run_test() {
    local video="$1"
    if [ "${USE_RUNUSER}" -eq 1 ]; then
        runuser -u "${TEST_USER}" -- \
            env "LIBVA_DRIVER_NAME=rockchip" \
                "LIBVA_DRIVERS_PATH=${DEST_DIR}" \
                "${TEST_BIN}" "${video}"
    else
        env "LIBVA_DRIVER_NAME=rockchip" \
            "LIBVA_DRIVERS_PATH=${DEST_DIR}" \
            "${TEST_BIN}" "${video}"
    fi
}

pass=0
fail=0
for video in "${VIDEOS[@]}"; do
    echo "----- 测试文件：$(basename "${video}") -----"
    if [ ! -f "${video}" ]; then
        echo "跳过：文件不存在 -> ${video}"
        fail=$((fail + 1))
        echo
        continue
    fi
    if run_test "${video}"; then
        echo "结果：成功"
        pass=$((pass + 1))
    else
        echo "结果：失败"
        fail=$((fail + 1))
    fi
    echo
done

echo "===== 汇总：成功 ${pass} 个，失败 ${fail} 个 ====="
[ "${fail}" -eq 0 ]
