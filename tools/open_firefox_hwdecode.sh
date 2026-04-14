#!/usr/bin/env bash

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=${ROCKCHIP_VAAPI_BUILD_DIR:-$repo_dir/build}
profile_dir=${ROCKCHIP_FIREFOX_PROFILE_DIR:-$HOME/.config/rockchip-firefox-hwdecode-profile}

pick_drm_device() {
	if [[ -n "${MOZ_DRM_DEVICE:-}" ]]; then
		printf '%s\n' "$MOZ_DRM_DEVICE"
		return
	fi

	local node
	for node in /dev/dri/renderD*; do
		[[ -e "$node" ]] || continue
		if udevadm info -q property -n "$node" | rg -q 'ID_PATH_TAG=.*gpu-panthor'; then
			printf '%s\n' "$node"
			return
		fi
	done

	printf '%s\n' /dev/dri/renderD129
}

mkdir -p "$profile_dir"
install -m 0644 "$repo_dir/tools/firefox_hwdecode_user.js" "$profile_dir/user.js"

drm_device=$(pick_drm_device)
export MOZ_ENABLE_WAYLAND=${MOZ_ENABLE_WAYLAND:-1}
export MOZ_DISABLE_RDD_SANDBOX=${MOZ_DISABLE_RDD_SANDBOX:-1}
export MOZ_WEBRENDER=${MOZ_WEBRENDER:-1}
export LIBVA_DRIVERS_PATH=${LIBVA_DRIVERS_PATH:-$build_dir}
export LIBVA_DRIVER_NAME=${LIBVA_DRIVER_NAME:-rockchip}
export LIBVA_DRM_DEVICE=${LIBVA_DRM_DEVICE:-$drm_device}
export MOZ_DRM_DEVICE=${MOZ_DRM_DEVICE:-$drm_device}
export ROCKCHIP_VAAPI_AV1_EXPORT_P010=${ROCKCHIP_VAAPI_AV1_EXPORT_P010:-1}

firefox_bin=${FIREFOX_BIN:-firefox}
target_url=${1:-about:blank}

exec "$firefox_bin" --new-instance --no-remote -profile "$profile_dir" "$target_url"