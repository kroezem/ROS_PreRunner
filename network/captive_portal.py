#!/usr/bin/env python3
"""Minimal captive-discovery landing page for the Runner field AP."""

from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import os


PADDOCK_URL = os.environ.get(
    'RUNNER_PADDOCK_URL', 'http://10.42.0.1:8000/'
)
LANDING_PAGE = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta name="theme-color" content="#15191f">
  <title>Runner</title>
  <style>
    html {{ color-scheme: dark; font-family: system-ui, sans-serif; }}
    body {{ margin: 0; background: #15191f; color: #f3f5f7; }}
    main {{ max-width: 28rem; margin: 12vh auto; padding: 2rem; text-align: center; }}
    a {{ display: block; margin: 2rem 0 1rem; padding: 1rem; border-radius: .6rem;
         background: #f5c542; color: #121212; font-weight: 800;
         text-decoration: none; letter-spacing: .04em; }}
    p {{ color: #c7ccd1; line-height: 1.45; }}
    code {{ color: #fff; }}
  </style>
</head>
<body><main>
  <h1>Runner connected</h1>
  <p>Paddock is available locally; internet access is not required.</p>
  <a href="{PADDOCK_URL}" target="_blank" rel="external noopener">OPEN PADDOCK</a>
  <p>If this Wi-Fi panel stays open, close it and open<br>
     <code>{PADDOCK_URL}</code> in Safari or your normal browser.</p>
</main></body>
</html>
""".encode('utf-8')


class CaptivePortalHandler(BaseHTTPRequestHandler):
    """Return the landing page for all OS captive-network probe paths."""

    server_version = 'RunnerDiscovery/1'

    def do_GET(self) -> None:  # noqa: N802 (HTTP method name)
        """Serve landing HTML, intentionally differing from probe success."""
        self._send(include_body=True)

    def do_HEAD(self) -> None:  # noqa: N802 (HTTP method name)
        """Serve the same headers without a response body."""
        self._send(include_body=False)

    def _send(self, *, include_body: bool) -> None:
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(LANDING_PAGE)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.send_header(
            'Content-Security-Policy',
            "default-src 'none'; style-src 'unsafe-inline'; "
            "base-uri 'none'; form-action 'none'",
        )
        self.end_headers()
        if include_body:
            self.wfile.write(LANDING_PAGE)

    def log_message(self, format: str, *args: object) -> None:
        """Keep one concise request record in the systemd journal."""
        super().log_message(format, *args)


def main() -> None:
    """Listen on HTTP port 80 for captive-network probes."""
    address = os.environ.get('RUNNER_CAPTIVE_BIND', '0.0.0.0')
    port = int(os.environ.get('RUNNER_CAPTIVE_PORT', '80'))
    server = ThreadingHTTPServer((address, port), CaptivePortalHandler)
    server.serve_forever()


if __name__ == '__main__':
    main()
