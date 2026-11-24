"""Expand a parameter grid into job specs and submit them with backpressure."""
from __future__ import annotations

import itertools
import random
import time

from .runners import kitti_spec
from .worker import Client, log


def parse_grid(items: list[str]) -> dict[str, list]:
    """["voxel=0.75,1.0", "deskew=0,1"] -> {"voxel": [0.75, 1.0], "deskew": [0, 1]}."""
    grid = {}
    for item in items:
        key, _, values = item.partition("=")
        if not key or not values:
            raise ValueError(f"expected key=v1,v2,...: {item!r}")
        grid[key] = [float(v) if "." in v else int(v) for v in values.split(",")]
    return grid


def expand(grid: dict[str, list]) -> list[dict]:
    keys = sorted(grid)
    return [dict(zip(keys, combo)) for combo in itertools.product(*(grid[k] for k in keys))]


def kitti_specs(engine_hash: str, sequences: list[str], grid: dict[str, list]) -> list[dict]:
    return [kitti_spec(engine_hash, seq, params) for seq in sequences for params in expand(grid)]


def submit_all(url: str, specs: list[dict], patience: float = 120.0, max_wait: float = 30.0) -> tuple[int, int]:
    """Submits every spec; on 429 (queue full) waits with backoff and tries again.
    Returns (new jobs, already present)."""
    client = Client(url, patience=patience)
    new = existing = 0
    for spec in specs:
        delay = 0.5
        while True:
            status, body = client.call("POST", "/jobs", {"spec": spec})
            if status == 201:
                new += body["created"]
                existing += not body["created"]
                break
            if status == 429:
                time.sleep(delay * random.uniform(0.5, 1.0))
                delay = min(delay * 2, max_wait)
                continue
            raise RuntimeError(f"submit: HTTP {status} {body}")
    log("submitter", f"submitted {len(specs)} specs: {new} new, {existing} already known")
    return new, existing
