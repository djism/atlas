"""Coordinator: an HTTP JSON front end to the Store.

Endpoints (all JSON):

    POST /jobs      {"spec": {...}}                     201 {"job_id", "created"} | 429 queue full
    POST /lease     {"worker": "w1"}                    200 {"job_id", "token", "spec", "attempt", "lease_seconds"} | 204
    POST /heartbeat {"job_id", "token"}                 200 {"expires"} | 409 lease lost
    POST /complete  {"job_id", "token", "worker", "result": {...}, "artifacts": {"name": base64}}
                                                        200 {"result_hash"} | 409 lease lost
    POST /fail      {"job_id", "token", "worker", "error"}  200 {"state"} | 409
    GET  /status                                        200 {"counts", "jobs"}

No authentication: it binds to 127.0.0.1 by default and is meant for one
machine or a trusted network. A background thread requeues expired leases
every second, so a crashed worker's job comes back even if nobody calls.
All state lives in the Store's directory; restarting the coordinator on the
same directory continues where it stopped.
"""
from __future__ import annotations

import base64
import binascii
import json
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .store import QueueFull, StaleLease, Store, UnknownJob

MAX_BODY = 64 * 1024 * 1024


def log(message: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} coordinator  {message}", file=sys.stderr, flush=True)


def make_handler(store: Store):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):  # quiet: events are logged by meaning, below
            pass

        def _send(self, status: int, body=None) -> None:
            data = b"" if body is None else json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self) -> dict:
            length = int(self.headers.get("Content-Length", "0"))
            if length > MAX_BODY:
                raise ValueError("request too large")
            data = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(data, dict):
                raise ValueError("expected a JSON object")
            return data

        def do_GET(self):
            if self.path == "/status":
                self._send(200, {"counts": store.counts(), "jobs": store.jobs()})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            try:
                body = self._body()
                if self.path == "/jobs":
                    if not isinstance(body.get("spec"), dict):
                        raise ValueError("spec must be an object")
                    job_id, created = store.submit(body["spec"])
                    self._send(201, {"job_id": job_id, "created": created})
                elif self.path == "/lease":
                    worker = str(body.get("worker", "?"))
                    lease = store.lease(worker)
                    if lease is None:
                        self._send(204)
                    else:
                        log(f"lease  {lease.job_id} -> {worker} (attempt {lease.attempt})")
                        self._send(200, {"job_id": lease.job_id, "token": lease.token, "spec": lease.spec,
                                         "attempt": lease.attempt, "lease_seconds": store.lease_seconds})
                elif self.path == "/heartbeat":
                    self._send(200, {"expires": store.heartbeat(body["job_id"], body["token"])})
                elif self.path == "/complete":
                    artifacts = {k: base64.b64decode(v, validate=True) for k, v in (body.get("artifacts") or {}).items()}
                    digest = store.complete(body["job_id"], body["token"], body["result"], artifacts, body.get("worker"))
                    log(f"done   {body['job_id']} by {body.get('worker')} -> result {digest[:12]}")
                    self._send(200, {"result_hash": digest})
                elif self.path == "/fail":
                    state = store.fail(body["job_id"], body["token"], str(body.get("error", "")), body.get("worker"))
                    log(f"failed {body['job_id']} by {body.get('worker')}: {str(body.get('error'))[:120]} -> {state}")
                    self._send(200, {"state": state})
                else:
                    self._send(404, {"error": "not found"})
            except QueueFull as e:
                self._send(429, {"error": str(e)})
            except StaleLease as e:
                log(f"REJECT {e}")
                self._send(409, {"error": str(e)})
            except UnknownJob as e:
                self._send(404, {"error": f"unknown job {e}"})
            except (KeyError, ValueError, binascii.Error) as e:
                self._send(400, {"error": f"bad request: {e}"})

    return Handler


class _Server(ThreadingHTTPServer):
    daemon_threads = True

    def server_bind(self):
        # The standard HTTPServer.server_bind looks up the host's DNS name
        # (socket.getfqdn), which can hang for tens of seconds; the name is
        # never used here, so bind without it.
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


def serve(store: Store, host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    """Starts serving in background threads and returns the server (call .shutdown() to stop)."""
    server = _Server((host, port), make_handler(store))
    extended = store.grant_restart_grace()
    if extended:
        log(f"restart grace: {extended} live lease(s) extended by {store.lease_seconds:g} s")
    stop = threading.Event()

    def reaper():
        while not stop.wait(1.0):
            n = store.reap()
            if n:
                log(f"requeued {n} job(s) whose lease expired")

    threading.Thread(target=server.serve_forever, daemon=True).start()
    threading.Thread(target=reaper, daemon=True).start()
    original_shutdown = server.shutdown

    def shutdown():
        stop.set()
        original_shutdown()
        server.server_close()

    server.shutdown = shutdown  # type: ignore[method-assign]
    return server
