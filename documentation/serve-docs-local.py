#!/usr/bin/env python3
"""Serve workspace ``docs/`` with esp-hal ``/latest/`` deep-link redirects (nginx parity).

Espressif's nginx maps::

    .../esp-hal/latest/<suffix>

to::

    .../esp-hal/latest/?path=<suffix>

Plain ``python -m http.server`` does not; use this script when testing Issue #4411-style
redirects locally.

Usage (from repo root)::

    python3 documentation/serve-docs-local.py
    python3 documentation/serve-docs-local.py --port 8766

Then open::

    http://127.0.0.1:8765/esp-hal/latest/esp32c6/esp_hal/index.html

which redirects to ``.../latest/?path=esp32c6/esp_hal/index.html``, then the generated
``latest/index.html`` script resolves the semver folder.

Requires Python 3.7+ (``directory=`` on :class:`http.server.SimpleHTTPRequestHandler`).
"""

from __future__ import annotations

import argparse
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import urllib.parse


def _repo_docs_root() -> Path:
    return Path(__file__).resolve().parent.parent / "docs"


def _handler_class(docs_dir: Path):
    latest_prefix = "/esp-hal/latest/"

    class DocsHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(docs_dir), **kwargs)

        def _redirect_latest_deep_link(self) -> bool:
            parsed = urllib.parse.urlparse(self.path)
            path = urllib.parse.unquote(parsed.path)

            if path.startswith(latest_prefix):
                rest = path[len(latest_prefix) :].lstrip("/")
                if rest:
                    query = urllib.parse.parse_qs(parsed.query)
                    if "path" not in query:
                        q = urllib.parse.quote(rest, safe="/")
                        dest = f"{latest_prefix}?path={q}"
                        self.send_response(302)
                        self.send_header("Location", dest)
                        self.end_headers()
                        return True
            return False

        def do_GET(self) -> None:  # noqa: N802 — stdlib API
            if self._redirect_latest_deep_link():
                return
            super().do_GET()

        def do_HEAD(self) -> None:  # noqa: N802 — stdlib API (`curl -I`)
            if self._redirect_latest_deep_link():
                return
            super().do_HEAD()

    return DocsHandler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Bind address (default: %(default)s)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="TCP port (default: %(default)s)",
    )
    parser.add_argument(
        "--docs",
        type=Path,
        default=_repo_docs_root(),
        help="Directory to serve (default: <repo>/docs)",
    )
    args = parser.parse_args()

    docs_dir = args.docs.resolve()
    if not docs_dir.is_dir():
        raise SystemExit(f"Not a directory: {docs_dir}")

    handler = _handler_class(docs_dir)
    server = HTTPServer((args.host, args.port), handler)

    print(f"Serving {docs_dir} at http://{args.host}:{args.port}/")
    print("Deep links under /esp-hal/latest/<…> redirect like production nginx.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
