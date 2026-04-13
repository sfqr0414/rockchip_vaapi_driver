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
  export ROCKCHIP_VAAPI_AV1_EXPORT_P010=1
  firefox

Required Firefox prefs for this driver:

  media.ffmpeg.vaapi.enabled=true
  media.ffmpeg.vaapi.force-surface-zero-copy=true
  media.ffmpeg.dmabuf-textures.enabled=true
  media.hardware-video-decoding.enabled=true
  media.hardware-video-decoding.force-enabled=true
  media.rdd-ffmpeg.enabled=true
  widget.dmabuf.force-enabled=true
  gfx.webrender.all=true
  gfx.webrender.compositor.force-enabled=true
  layers.gpu-process.enabled=true
  gfx.blocklist.all=-1
  media.ffvpx.enabled=false

Example user.js snippet:

  user_pref("media.ffmpeg.vaapi.enabled", true);
  user_pref("media.ffmpeg.vaapi.force-surface-zero-copy", true);
  user_pref("media.ffmpeg.dmabuf-textures.enabled", true);
  user_pref("media.hardware-video-decoding.enabled", true);
  user_pref("media.hardware-video-decoding.force-enabled", true);
  user_pref("media.rdd-ffmpeg.enabled", true);
  user_pref("widget.dmabuf.force-enabled", true);
  user_pref("gfx.webrender.all", true);
  user_pref("gfx.webrender.compositor.force-enabled", true);
  user_pref("layers.gpu-process.enabled", true);
  user_pref("gfx.blocklist.all", -1);
  user_pref("media.ffvpx.enabled", false);

If Firefox still falls back to software decoding, check:
- `LIBVA_DRIVER_NAME` is set correctly
- `/usr/lib/aarch64-linux-gnu/dri/rockchip_drv_video.so` exists and points at this build
- Permissions allow access to /dev/mpp_service and DRM nodes
- Firefox is running on Wayland and not X11/XWayland
- The above prefs are present in the active profile, not only in a different profile

If AV1 10-bit hardware decode shows posterization, color shifts, or corrupted chroma, verify:
- `ROCKCHIP_VAAPI_AV1_EXPORT_P010=1` is present for the Firefox process
- the active Firefox session was restarted after changing the env or prefs

EOF
