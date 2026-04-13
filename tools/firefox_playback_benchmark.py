#!/usr/bin/env python3

import argparse
import contextlib
import http.server
import json
import os
import shutil
import socketserver
import subprocess
import tempfile
import threading
import time
from pathlib import Path

from selenium import webdriver
from selenium.webdriver.firefox.options import Options
from selenium.webdriver.firefox.service import Service


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


def start_http_server(repo_dir: Path):
    class ReusableTCPServer(socketserver.TCPServer):
        allow_reuse_address = True

    handler = lambda *args, **kwargs: QuietHandler(*args, directory=str(repo_dir), **kwargs)
    server = ReusableTCPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def build_options(args, profile_dir: Path) -> Options:
    options = Options()
    options.add_argument("--new-instance")
    options.add_argument("--profile")
    options.add_argument(str(profile_dir))

    prefs = {
        "media.ffmpeg.vaapi.enabled": args.hwdecode,
        "media.ffmpeg.vaapi.force-surface-zero-copy": args.hwdecode,
        "media.ffmpeg.dmabuf-textures.enabled": True,
        "media.hardware-video-decoding.enabled": True,
        "media.hardware-video-decoding.force-enabled": True,
        "media.rdd-ffmpeg.enabled": True,
        "widget.dmabuf.force-enabled": True,
        "gfx.webrender.all": True,
        "gfx.webrender.compositor.force-enabled": True,
        "layers.gpu-process.enabled": True,
        "gfx.blocklist.all": -1,
        "media.ffvpx.enabled": False,
        "media.autoplay.default": 0,
        "media.autoplay.blocking_policy": 0,
        "permissions.default.autoplay-media": 0,
    }
    for key, value in prefs.items():
        options.set_preference(key, value)
    return options


def collect_samples(driver, seconds: float, interval: float):
    samples = []
    end_at = time.time() + seconds
    script = """
const video = document.getElementById('sample');
const quality = video.getVideoPlaybackQuality ? video.getVideoPlaybackQuality() : null;
return {
  currentTime: video.currentTime,
  paused: video.paused,
  ended: video.ended,
  readyState: video.readyState,
  networkState: video.networkState,
  videoWidth: video.videoWidth,
  videoHeight: video.videoHeight,
  decodedVideoFrames: quality ? quality.totalVideoFrames : null,
  droppedVideoFrames: quality ? quality.droppedVideoFrames : null,
  corruptedVideoFrames: quality ? quality.corruptedVideoFrames : null,
  totalFrameDelay: quality ? quality.totalFrameDelay : null,
  src: video.currentSrc,
};
"""
    while time.time() < end_at:
        samples.append(driver.execute_script(script))
        time.sleep(interval)
    return samples


def summarize(samples):
    first = samples[0]
    last = samples[-1]
    current_delta = last["currentTime"] - first["currentTime"]
    decoded_delta = None
    dropped_delta = None
    if first["decodedVideoFrames"] is not None and last["decodedVideoFrames"] is not None:
        decoded_delta = last["decodedVideoFrames"] - first["decodedVideoFrames"]
    if first["droppedVideoFrames"] is not None and last["droppedVideoFrames"] is not None:
        dropped_delta = last["droppedVideoFrames"] - first["droppedVideoFrames"]
    return {
        "first": first,
        "last": last,
        "sample_count": len(samples),
        "current_time_delta": current_delta,
        "decoded_frames_delta": decoded_delta,
        "dropped_frames_delta": dropped_delta,
        "final_total_frame_delay": last["totalFrameDelay"],
    }


def ensure_driver_binary() -> str:
    path = shutil.which("geckodriver")
    if path:
        return path
    raise SystemExit("geckodriver not found in PATH")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--hwdecode", action="store_true")
    parser.add_argument("--video", default="videos/Test Jellyfin 1080p AVC 20M.mp4")
    args = parser.parse_args()

    repo_dir = Path(__file__).resolve().parents[1]
    profile_dir = Path(tempfile.mkdtemp(prefix="rockchip-firefox-bench."))
    geckodriver_path = ensure_driver_binary()
    env = os.environ.copy()
    env.setdefault("MOZ_ENABLE_WAYLAND", "1")
    env.setdefault("MOZ_DISABLE_RDD_SANDBOX", "1")
    env.setdefault("MOZ_WEBRENDER", "1")
    env.setdefault("LIBVA_DRIVER_NAME", "rockchip")
    env.setdefault("MOZ_DRM_DEVICE", "/dev/dri/renderD129")
    env.setdefault("LIBVA_DRM_DEVICE", env["MOZ_DRM_DEVICE"])

    server, _ = start_http_server(repo_dir)
    url = f"http://127.0.0.1:{server.server_address[1]}/tools/firefox_headless_video_test.html"

    service = Service(executable_path=geckodriver_path, log_output=subprocess.DEVNULL, env=env)
    options = build_options(args, profile_dir)
    driver = None
    try:
        driver = webdriver.Firefox(service=service, options=options)
        driver.get(url)
        driver.execute_script(
            "document.getElementById('sample').src = arguments[0]; return true;",
            f"http://127.0.0.1:{server.server_address[1]}/{args.video}",
        )
        driver.execute_script("return document.getElementById('sample').play();")
        time.sleep(1.0)
        samples = collect_samples(driver, args.duration, args.interval)
        print(json.dumps(summarize(samples), indent=2))
    finally:
        with contextlib.suppress(Exception):
            if driver is not None:
                driver.quit()
        server.shutdown()
        server.server_close()
        shutil.rmtree(profile_dir, ignore_errors=True)


if __name__ == "__main__":
    main()