#!/usr/bin/env bash

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
profile_dir=$(mktemp -d /tmp/rockchip-firefox-profile.XXXXXX)
server_state_dir=/tmp/rockchip-firefox-http-server
mkdir -p "$server_state_dir"
status_file="$server_state_dir/playback-status.jsonl"

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
vaapi_enabled=${FIREFOX_VAAPI_ENABLED:-true}
force_zero_copy=${FIREFOX_VAAPI_FORCE_ZERO_COPY:-true}
hwdecode_enabled=${FIREFOX_HWDECODE_ENABLED:-true}
test_video=${FIREFOX_TEST_VIDEO:-Test Jellyfin 1080p AVC 20M.mp4}

pick_http_port() {
	python3 - <<'PY'
import socket

with socket.socket() as sock:
	sock.bind(("127.0.0.1", 0))
	print(sock.getsockname()[1])
PY
}

start_http_server() {
	local port=$1
	local server_log="$server_state_dir/server.log"
	: >"$status_file"
	nohup python3 "$repo_dir/tools/firefox_playback_server.py" \
		--bind 127.0.0.1 \
		--port "$port" \
		--directory "$repo_dir" \
		--status-file "$status_file" >"$server_log" 2>&1 &
	local server_pid=$!
	echo "$server_pid" >"$server_state_dir/pid"
	echo "$port" >"$server_state_dir/port"
	echo "$server_log" >"$server_state_dir/log"
	echo "$status_file" >"$server_state_dir/status_file"

	for _ in $(seq 1 50); do
		if curl -fsS "http://127.0.0.1:$port/" >/dev/null 2>&1; then
			return 0
		fi
		sleep 0.1
	done

	echo "failed to start local HTTP server on port $port" >&2
	return 1
}

ensure_http_server() {
	local port=${FIREFOX_TEST_PORT:-}
	local pid_file="$server_state_dir/pid"
	local port_file="$server_state_dir/port"
	local existing_ok=0

	if [[ -z "$port" && -f "$port_file" ]]; then
		port=$(<"$port_file")
	fi

	if [[ -n "$port" ]] && [[ -f "$pid_file" ]]; then
		local old_pid
		old_pid=$(<"$pid_file")
		if ps -p "$old_pid" -o cmd= 2>/dev/null | rg -q 'firefox_playback_server.py'; then
			if curl -fsS "http://127.0.0.1:$port/" >/dev/null 2>&1; then
				existing_ok=1
			fi
		fi
	fi

	if [[ "$existing_ok" -eq 1 ]]; then
		printf '%s\n' "$port"
		return
	fi

	if [[ -f "$pid_file" ]]; then
		local old_pid
		old_pid=$(<"$pid_file")
		kill "$old_pid" >/dev/null 2>&1 || true
	fi

	if [[ -z "$port" ]]; then
		port=$(pick_http_port)
	fi

	start_http_server "$port"
	printf '%s\n' "$port"
}

server_port=$(ensure_http_server)
encoded_video=$(python3 -c 'import sys, urllib.parse; print(urllib.parse.quote(sys.argv[1]))' "$test_video")

cat >"$profile_dir/user.js" <<'EOF'
user_pref("media.ffmpeg.vaapi.enabled", __VAAPI_ENABLED__);
user_pref("media.ffmpeg.vaapi.force-surface-zero-copy", __FORCE_ZERO_COPY__);
user_pref("media.ffmpeg.dmabuf-textures.enabled", true);
user_pref("media.hardware-video-decoding.enabled", __HWDECODE_ENABLED__);
user_pref("media.hardware-video-decoding.force-enabled", __HWDECODE_ENABLED__);
user_pref("media.rdd-ffmpeg.enabled", true);
user_pref("widget.dmabuf.force-enabled", true);
user_pref("gfx.webrender.all", true);
user_pref("gfx.webrender.compositor.force-enabled", true);
user_pref("layers.gpu-process.enabled", true);
user_pref("gfx.blocklist.all", -1);
user_pref("media.ffvpx.enabled", false);
EOF

sed -i \
	-e "s/__VAAPI_ENABLED__/$vaapi_enabled/g" \
	-e "s/__FORCE_ZERO_COPY__/$force_zero_copy/g" \
	-e "s/__HWDECODE_ENABLED__/$hwdecode_enabled/g" \
	"$profile_dir/user.js"

export MOZ_ENABLE_WAYLAND=${MOZ_ENABLE_WAYLAND:-1}
export MOZ_DISABLE_RDD_SANDBOX=${MOZ_DISABLE_RDD_SANDBOX:-1}
export LIBVA_DRIVER_NAME=${LIBVA_DRIVER_NAME:-rockchip}
export MOZ_DRM_DEVICE=$drm_device
export LIBVA_DRM_DEVICE=${LIBVA_DRM_DEVICE:-$drm_device}
export MOZ_WEBRENDER=${MOZ_WEBRENDER:-1}
export MOZ_LOG=${MOZ_LOG:-PlatformDecoderModule:5,FFmpegVideo:5,Dmabuf:5,VAAPI:5}

firefox_bin=${FIREFOX_BIN:-firefox}
target_url=${1:-"http://127.0.0.1:$server_port/tools/firefox_headless_video_test.html?video=$encoded_video"}

printf 'PROFILE=%s\n' "$profile_dir"
printf 'URL=%s\n' "$target_url"
printf 'HTTP_SERVER=%s\n' "http://127.0.0.1:$server_port/"
printf 'STATUS_FILE=%s\n' "$status_file"

"$firefox_bin" --new-instance --no-remote -profile "$profile_dir" "$target_url"