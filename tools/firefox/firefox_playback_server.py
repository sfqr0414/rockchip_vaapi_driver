#!/usr/bin/env python3

import argparse
import http.server
import json
from pathlib import Path
from socketserver import ThreadingMixIn


class ThreadingHTTPServer(ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


class PlaybackHandler(http.server.SimpleHTTPRequestHandler):
    server_version = "RockchipPlaybackHTTP/1.0"

    def __init__(self, *args, directory=None, status_file=None, telemetry_file=None, **kwargs):
        self._status_file = Path(status_file) if status_file else None
        self._telemetry_file = Path(telemetry_file) if telemetry_file else None
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/__telemetry__":
            payload = {"available": False}
            if self._telemetry_file and self._telemetry_file.exists():
                try:
                    payload = json.loads(self._telemetry_file.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    payload = {"available": False, "error": "invalid telemetry json"}

            encoded = (json.dumps(payload, ensure_ascii=True) + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(encoded)
            return

        super().do_GET()

    def do_POST(self):
        if self.path != "/__status__":
            self.send_error(404)
            return

        length = int(self.headers.get("Content-Length", "0"))
        payload = self.rfile.read(length)
        if self._status_file:
            self._status_file.parent.mkdir(parents=True, exist_ok=True)
            with self._status_file.open("ab") as handle:
                handle.write(payload)
                handle.write(b"\n")

        self.send_response(204)
        self.end_headers()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--status-file", required=True)
    parser.add_argument("--telemetry-file")
    args = parser.parse_args()

    handler = lambda *h_args, **h_kwargs: PlaybackHandler(
        *h_args,
        directory=args.directory,
        status_file=args.status_file,
        telemetry_file=args.telemetry_file,
        **h_kwargs,
    )
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()