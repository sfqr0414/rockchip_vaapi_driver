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

    def __init__(self, *args, directory=None, status_file=None, **kwargs):
        self._status_file = Path(status_file) if status_file else None
        super().__init__(*args, directory=directory, **kwargs)

    def log_message(self, format, *args):
        pass

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
    args = parser.parse_args()

    handler = lambda *h_args, **h_kwargs: PlaybackHandler(
        *h_args,
        directory=args.directory,
        status_file=args.status_file,
        **h_kwargs,
    )
    server = ThreadingHTTPServer((args.bind, args.port), handler)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()