#!/usr/bin/env bash

set -euo pipefail

# Build and install the VA-API driver for Rockchip MPP.
# Requires: cmake, make, sudo (for installation step).

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BUILD_DIR="${ROOT_DIR}/build"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${ROOT_DIR}" -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"

# Install the driver into the system VA-API driver folder.
# Adjust this path if your distro uses a different location.
sudo make install

cat <<'EOF'

✅ Rockchip VA-API driver installed.

To run Firefox with Wayland + VA-API hardware decode:

  export LIBVA_DRIVER_NAME=rockchip
  export MOZ_ENABLE_WAYLAND=1
  firefox

If Firefox still falls back to software decoding, check:
- `LIBVA_DRIVER_NAME` is set correctly
- `/usr/lib/aarch64-linux-gnu/dri/librockchip_drv_video.so` exists
- Permissions allow access to /dev/mpp_service and DRM nodes

EOF
