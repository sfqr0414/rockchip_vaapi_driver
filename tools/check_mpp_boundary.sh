#!/usr/bin/env bash
# Enforce the bridge rule: the VA-API driver may only talk to MPP through the
# installed public headers and the librockchip_mpp MPI ABI.
set -euo pipefail
repo_dir=${1:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}
cd "$repo_dir"

fail=0
report() {
    printf 'mpp-boundary: %s\n' "$*" >&2
    fail=1
}

mapfile -t sources < <(find src -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -print)

# Only the installed public rockchip headers are allowed.
for file in "${sources[@]}"; do
    while IFS= read -r include; do
        case "$include" in
            rockchip/rk_mpi.h|rockchip/mpp_buffer.h|rockchip/mpp_frame.h|rockchip/mpp_packet.h|\
            rockchip/mpp_meta.h|rockchip/mpp_log.h|rockchip/mpp_err.h|rockchip/mpp_bit.h|\
            rockchip/mpp_compat.h|rockchip/mpp_log_def.h|rockchip/rk_type.h)
                ;;
            *)
                report "$file includes non-public MPP header <$include>"
                ;;
        esac
    done < <(sed -nE 's/^[[:space:]]*#[[:space:]]*include[[:space:]]*[<"]([^">]+)[">].*/\1/p' "$file" |
             grep -E '(^|/)(mpp|rockchip)/|(^|/)(osal|mpp_impl|mpp_buf_slot|mpp_sys_cfg|hal_|vproc|legacy)/' || true)
done

# No source-tree or build-tree include path may be baked into the build.
if grep -RIn --include='CMakeLists.txt' --include='*.cmake' \
       -E '/home/pi/Share/mpp|MPP_SOURCE|mpp/(inc|mpp|osal|hal|vproc|legacy)' . \
       --exclude-dir=.git --exclude-dir=build --exclude-dir=build-work \
       --exclude-dir=firefox-src --exclude-dir=mesa_ref --exclude-dir=output; then
    report "build files reference MPP source paths"
fi

# The bridge must resolve MPP through the shared library, not static/source objects.
lib=$(find build-work build -maxdepth 1 -name 'rockchip_drv_video.so' -print -quit 2>/dev/null || true)
if [[ -n "$lib" ]]; then
    if ! readelf -d "$lib" | grep -q 'librockchip_mpp\.so'; then
        report "$lib is not linked against librockchip_mpp.so"
    fi
    if readelf -d "$lib" | grep -q 'librockchip_mpp\.a'; then
        report "$lib links a static MPP archive"
    fi
fi

if (( fail )); then
    exit 1
fi
printf 'mpp-boundary: OK (public headers + librockchip_mpp MPI ABI only)\n'
