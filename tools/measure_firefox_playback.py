#!/usr/bin/env python3

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parent.parent
STATUS_FILE = Path("/tmp/rockchip-firefox-http-server/playback-status.jsonl")


def clear_harness_processes() -> None:
    subprocess.run(
        ["bash", "-lc", "pkill -f '/tmp/rockchip-firefox-profile\\.' || true; pkill -f 'bash ./tools/run_firefox_vaapi_probe.sh' || true"],
        cwd=REPO_DIR,
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def read_status_lines(status_file: Path):
    if not status_file.exists():
        return []
    lines = []
    for line in status_file.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            lines.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return lines


def compute_summary(samples):
    if not samples:
        return {"error": "no telemetry samples"}

    usable = [sample for sample in samples if sample.get("duration")]
    first = usable[0] if usable else samples[0]
    last = usable[-1] if usable else samples[-1]

    result = {
        "sample_count": len(samples),
        "first_timestamp": first.get("timestamp"),
        "last_timestamp": last.get("timestamp"),
        "first_currentTime": first.get("currentTime"),
        "last_currentTime": last.get("currentTime"),
        "currentTime_delta": None,
        "first_decoded": first.get("decodedVideoFrames"),
        "last_decoded": last.get("decodedVideoFrames"),
        "decoded_delta": None,
        "first_dropped": first.get("droppedVideoFrames"),
        "last_dropped": last.get("droppedVideoFrames"),
        "dropped_delta": None,
        "last_readyState": last.get("readyState"),
        "last_paused": last.get("paused"),
        "last_ended": last.get("ended"),
        "ended_seen": any(bool(sample.get("ended")) for sample in samples),
        "last_error": last.get("error"),
        "last_events": last.get("events"),
        "seek_enabled": any(bool(sample.get("seekEnabled")) for sample in samples),
        "seek_triggered": any(bool(sample.get("seekTriggered")) for sample in samples),
        "seek_completed": any(bool(sample.get("seekCompleted")) for sample in samples),
        "seek_target": next((sample.get("seekTarget") for sample in reversed(samples) if sample.get("seekTarget") is not None), None),
    }

    if first.get("currentTime") is not None and last.get("currentTime") is not None:
        result["currentTime_delta"] = round(last["currentTime"] - first["currentTime"], 3)
    if first.get("decodedVideoFrames") is not None and last.get("decodedVideoFrames") is not None:
        result["decoded_delta"] = last["decodedVideoFrames"] - first["decodedVideoFrames"]
    if first.get("droppedVideoFrames") is not None and last.get("droppedVideoFrames") is not None:
        result["dropped_delta"] = last["droppedVideoFrames"] - first["droppedVideoFrames"]

    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--mode", choices=["hw", "sw"], required=True)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--stable-export", action="store_true")
    parser.add_argument("--seek-at", type=float)
    parser.add_argument("--seek-to", type=float)
    parser.add_argument("--stdout-file")
    parser.add_argument("--summary-file")
    parser.add_argument("--last-sample-file")
    args = parser.parse_args()

    clear_harness_processes()

    env = os.environ.copy()
    env["FIREFOX_TEST_VIDEO"] = args.video
    env["FIREFOX_VAAPI_ENABLED"] = "true" if args.mode == "hw" else "false"
    env["FIREFOX_HWDECODE_ENABLED"] = "true" if args.mode == "hw" else "false"
    env["FIREFOX_VAAPI_FORCE_ZERO_COPY"] = "true" if args.mode == "hw" else "false"
    if args.seek_at is not None:
        env["FIREFOX_TEST_SEEK_AT"] = str(args.seek_at)
    else:
        env.pop("FIREFOX_TEST_SEEK_AT", None)
    if args.seek_to is not None:
        env["FIREFOX_TEST_SEEK_TO"] = str(args.seek_to)
    else:
        env.pop("FIREFOX_TEST_SEEK_TO", None)
    if args.stable_export:
        env["ROCKCHIP_VAAPI_STABLE_EXPORT"] = "1"
    else:
        env.pop("ROCKCHIP_VAAPI_STABLE_EXPORT", None)

    if STATUS_FILE.exists():
        STATUS_FILE.unlink()

    stdout_handle = None
    if args.stdout_file:
        stdout_path = Path(args.stdout_file)
        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stdout_handle = stdout_path.open("w", encoding="utf-8")

    process = subprocess.Popen(
        ["bash", "./tools/run_firefox_vaapi_probe.sh"],
        cwd=REPO_DIR,
        env=env,
        stdout=stdout_handle if stdout_handle else subprocess.DEVNULL,
        stderr=subprocess.STDOUT if stdout_handle else subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    start = time.monotonic()
    deadline = start + max(args.duration, 1.0)

    try:
        samples = []
        while time.monotonic() < deadline:
            samples = read_status_lines(STATUS_FILE)
            time.sleep(0.2)

        summary = compute_summary(samples)
        summary.update(
            {
                "video": args.video,
                "mode": args.mode,
                "stable_export": args.stable_export,
                "seek_at": args.seek_at,
                "seek_to": args.seek_to,
                "duration_seconds": args.duration,
            }
        )
        if args.summary_file:
            summary_path = Path(args.summary_file)
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            summary_path.write_text(json.dumps(summary, ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
        if args.last_sample_file and samples:
            last_sample_path = Path(args.last_sample_file)
            last_sample_path.parent.mkdir(parents=True, exist_ok=True)
            last_sample_path.write_text(json.dumps(samples[-1], ensure_ascii=True, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, ensure_ascii=True, indent=2))
        return 0
    finally:
        if stdout_handle:
            stdout_handle.flush()
            stdout_handle.close()
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        clear_harness_processes()


if __name__ == "__main__":
    sys.exit(main())