"""System tests with real HTTP and real processes. Run from the repository root:
python3 -m unittest discover -s eval/tests -v   (about 45 s)"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from atlas_eval.coordinator import serve  # noqa: E402
from atlas_eval.runners import test_runner  # noqa: E402
from atlas_eval.store import Store  # noqa: E402
from atlas_eval.worker import run_worker  # noqa: E402

CLI = os.path.join(HERE, "..", "cli.py")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def post(url, path, body):
    req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            raw = r.read()
            return r.status, (json.loads(raw) if raw else {})
    except urllib.error.HTTPError as e:
        with e:
            return e.code, json.loads(e.read() or b"{}")


class HttpTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.store = Store(self.dir.name, capacity=2, lease_seconds=1.0, backoff_base=0.1)
        self.server = serve(self.store, "127.0.0.1", 0)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.store.close()
        self.dir.cleanup()

    def test_backpressure_over_http(self):
        self.assertEqual(post(self.url, "/jobs", {"spec": {"n": 1}})[0], 201)
        self.assertEqual(post(self.url, "/jobs", {"spec": {"n": 2}})[0], 201)
        self.assertEqual(post(self.url, "/jobs", {"spec": {"n": 3}})[0], 429)
        self.assertEqual(post(self.url, "/jobs", {"spec": {"n": 1}}), (201, {"job_id": self.store.jobs()[0]["id"], "created": False}))

    def test_a_zombie_worker_cannot_publish(self):
        post(self.url, "/jobs", {"spec": {"n": 1}})
        _, zombie = post(self.url, "/lease", {"worker": "zombie"})
        time.sleep(1.3)  # lease (1 s) runs out without heartbeats; the reaper requeues it
        deadline = time.time() + 5
        while (lease := post(self.url, "/lease", {"worker": "w2"}))[0] != 200 and time.time() < deadline:
            time.sleep(0.05)  # wait out the backoff
        self.assertEqual(lease[0], 200)
        self.assertEqual(lease[1]["attempt"], 2)
        late = {"job_id": zombie["job_id"], "token": zombie["token"], "worker": "zombie", "result": {"v": "zombie"}}
        self.assertEqual(post(self.url, "/complete", late)[0], 409)
        good = {"job_id": lease[1]["job_id"], "token": lease[1]["token"], "worker": "w2", "result": {"v": "w2"}}
        self.assertEqual(post(self.url, "/complete", good)[0], 200)
        self.assertEqual(post(self.url, "/complete", late)[0], 409, "still rejected after the job is done")
        self.assertEqual(post(self.url, "/complete", good)[0], 200, "a retried completion is idempotent")
        self.assertEqual([r["result"] for r in self.store.results()], [{"v": "w2"}])

    def test_bad_requests_are_refused(self):
        self.assertEqual(post(self.url, "/jobs", {"spec": "not an object"})[0], 400)
        self.assertEqual(post(self.url, "/heartbeat", {"job_id": "nope", "token": "x"})[0], 404)
        self.assertEqual(post(self.url, "/nowhere", {})[0], 404)


class RestartTest(unittest.TestCase):
    def test_a_restarted_coordinator_honours_leases_from_before_the_outage(self):
        with tempfile.TemporaryDirectory() as tmp:
            clock = [1000.0]
            store = Store(tmp, lease_seconds=5, clock=lambda: clock[0])
            store.submit({"n": 1})
            lease = store.lease("w1")
            store.close()
            clock[0] += 60  # down for a minute
            store = Store(tmp, lease_seconds=5, clock=lambda: clock[0])
            server = serve(store, "127.0.0.1", 0)
            url = f"http://127.0.0.1:{server.server_address[1]}"
            status, _ = post(url, "/heartbeat", {"job_id": lease.job_id, "token": lease.token})
            server.shutdown()
            store.close()
            self.assertEqual(status, 200)


class SlowHeartbeatStore(Store):
    """Heartbeats take 0.4 s to answer, so one is often in flight when a job ends."""
    def heartbeat(self, job_id, token):
        time.sleep(0.4)
        return super().heartbeat(job_id, token)


class WorkerTest(unittest.TestCase):
    def test_a_finished_jobs_heartbeat_cannot_cancel_the_next_job(self):
        # Regression: the heartbeat thread read the job's variables late, so a
        # heartbeat still in flight when job k ended got 409 ("job k is done")
        # and cancelled job k+1, whose lease was fine (seen in the KITTI demo).
        with tempfile.TemporaryDirectory() as tmp:
            store = SlowHeartbeatStore(tmp, lease_seconds=1.5)
            server = serve(store, "127.0.0.1", 0)
            url = f"http://127.0.0.1:{server.server_address[1]}"
            for n in range(10):
                store.submit({"kind": "test", "n": n, "seconds": 0.6})
            published = run_worker(url, "w1", {"test": test_runner}, exit_when_idle=True, poll=0.1)
            kinds = [e["kind"] for e in store.events()]
            server.shutdown()
            self.assertEqual(published, 10)
            self.assertEqual(kinds.count("lease_expired"), 0, "no job was abandoned")
            self.assertEqual(kinds.count("leased"), 10, "every job ran once")
            store.close()


class CrashRecoveryTest(unittest.TestCase):
    def test_demo_survives_worker_crash_pause_and_coordinator_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            store_dir = os.path.join(tmp, "run")
            proc = subprocess.run([sys.executable, CLI, "demo", "--store", store_dir, "--port", str(free_port()),
                                   "--lease", "2", "--test-jobs", "40", "--test-seconds", "2", "--timeout", "240"],
                                  capture_output=True, text=True, timeout=300)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            for step in ("CRASH: kill -9 w1", "CRASH: kill -9 the coordinator", "coordinator restarted", "PAUSE: SIGSTOP w2"):
                self.assertIn(step, proc.stdout)
            store = Store(store_dir)
            kinds = [e["kind"] for e in store.events()]
            completed = [e["job_id"] for e in store.events() if e["kind"] == "completed"]
            self.assertEqual(store.counts()["done"], 40)
            self.assertEqual(len(completed), 40)
            self.assertEqual(len(set(completed)), 40, "one accepted result per job")
            self.assertGreaterEqual(kinds.count("lease_expired"), 2, "the crashed and the paused worker's leases expired")
            self.assertGreaterEqual(kinds.count("stale_heartbeat_rejected") + kinds.count("stale_result_rejected"), 1,
                                    "the paused worker was fenced off when it woke up")
            self.assertEqual(sorted(r["result"]["square"] for r in store.results()), sorted(n * n for n in range(40)))
            store.close()


if __name__ == "__main__":
    unittest.main()
