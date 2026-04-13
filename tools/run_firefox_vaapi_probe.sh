#!/usr/bin/env bash

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
profile_dir=$(mktemp -d /tmp/rockchip-firefox-profile.XXXXXX)
trap 'rm -rf "$profile_dir"' EXIT

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

drm_device=$(pick_drm_device)

cat >"$profile_dir/user.js" <<'EOF'
user_pref("media.ffmpeg.vaapi.enabled", true);
user_pref("media.ffmpeg.vaapi.force-surface-zero-copy", true);
user_pref("media.hardware-video-decoding.enabled", true);
user_pref("media.hardware-video-decoding.force-enabled", true);
user_pref("media.rdd-ffmpeg.enabled", true);
user_pref("widget.dmabuf.force-enabled", true);
user_pref("gfx.webrender.all", true);
user_pref("gfx.webrender.compositor.force-enabled", true);
user_pref("layers.gpu-process.enabled", true);
user_pref("gfx.blocklist.all", -1);
user_pref("media.ffvpx.enabled", false);
EOF

export MOZ_ENABLE_WAYLAND=${MOZ_ENABLE_WAYLAND:-1}
export MOZ_DISABLE_RDD_SANDBOX=${MOZ_DISABLE_RDD_SANDBOX:-1}
export LIBVA_DRIVER_NAME=${LIBVA_DRIVER_NAME:-rockchip}
export MOZ_DRM_DEVICE=$drm_device
export LIBVA_DRM_DEVICE=${LIBVA_DRM_DEVICE:-$drm_device}
export MOZ_WEBRENDER=${MOZ_WEBRENDER:-1}
export MOZ_LOG=${MOZ_LOG:-PlatformDecoderModule:5,FFmpegVideo:5,Dmabuf:5,VAAPI:5}

firefox_bin=${FIREFOX_BIN:-firefox}
target_url=${1:-"file://$repo_dir/tools/firefox_headless_video_test.html"}

exec "$firefox_bin" --no-remote -profile "$profile_dir" "$target_url"