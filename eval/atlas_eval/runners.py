"""What a worker does with a job, chosen by spec["kind"].

- "kitti-map-odometry": runs `atlas_kitti run` (the C++ engine) on one KITTI
  sequence with the spec's parameters; the result is the engine's JSON line
  and the estimated poses are an artifact.
- "test": sleeps, optionally fails its first attempts, returns n * n. Used by
  the tests and the CI demo, which have no KITTI data.

A runner gets the spec, the attempt number, and a `cancel` event; it must stop
promptly when `cancel` is set (the worker lost its lease).
"""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass, field


class Cancelled(Exception):
    """The worker lost its lease; the run was stopped and nothing is reported."""


def file_hash(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()[:16]


def kitti_spec(engine_hash: str, sequence: str, params: dict) -> dict:
    """The job spec for one configured run. The engine's hash is part of it, so
    results are tied to the exact binary, and a rebuilt engine means new jobs."""
    return {"kind": "kitti-map-odometry", "engine": engine_hash, "sequence": sequence,
            "params": {k: params[k] for k in sorted(params)}}


@dataclass
class KittiRunner:
    engine: str                      # path to atlas_kitti
    odometry: str                    # .../odometry/dataset
    velodyne: dict[str, str]         # sequence -> directory of scans
    engine_hash: str = field(init=False)

    def __post_init__(self):
        self.engine_hash = file_hash(self.engine)

    def __call__(self, spec: dict, attempt: int, cancel: threading.Event):
        if spec["engine"] != self.engine_hash:
            raise RuntimeError(f"engine mismatch: job wants {spec['engine']}, this worker has {self.engine_hash}")
        seq = spec["sequence"]
        if seq not in self.velodyne:
            raise RuntimeError(f"this worker has no scans for sequence {seq}")
        args = [f"{k}={v}" for k, v in spec["params"].items()]
        with tempfile.TemporaryDirectory(prefix="atlas-job-") as tmp:
            poses = os.path.join(tmp, "poses.txt")
            proc = subprocess.Popen([self.engine, "run", self.odometry, seq, self.velodyne[seq], poses, *args],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            while proc.poll() is None:
                if cancel.wait(0.2):
                    proc.kill()
                    proc.wait()
                    raise Cancelled()
            out, err = proc.communicate()
            if proc.returncode != 0:
                raise RuntimeError(f"atlas_kitti exited {proc.returncode}: {err.strip()[-400:]}")
            result = json.loads(out.strip().splitlines()[-1])
            with open(poses, "rb") as f:
                return result, {"poses.txt": f.read()}


def test_runner(spec: dict, attempt: int, cancel: threading.Event):
    """spec: {"kind": "test", "n": int, "seconds": float, "fail_attempts": int}."""
    deadline = time.monotonic() + float(spec.get("seconds", 0.5))
    while time.monotonic() < deadline:
        if cancel.wait(0.05):
            raise Cancelled()
    if attempt <= int(spec.get("fail_attempts", 0)):
        raise RuntimeError(f"planned failure on attempt {attempt}")
    n = int(spec["n"])
    return {"n": n, "square": n * n}, {}
