"""Unit tests for the job store: leases, fencing, retries, durability.
Run from the repository root: python3 -m unittest discover -s eval/tests -v
A fake clock makes lease timing exact."""
import json
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from atlas_eval.store import QueueFull, StaleLease, Store, canonical, job_id_for  # noqa: E402


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class StoreTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.store = self.make()

    def tearDown(self):
        self.store.close()
        self.dir.cleanup()

    def make(self, **kw):
        options = dict(capacity=4, lease_seconds=10, max_attempts=3, backoff_base=2, backoff_cap=60,
                       clock=self.clock, rng=random.Random(1))
        options.update(kw)
        return Store(self.dir.name, **options)

    def kinds(self):
        return [e["kind"] for e in self.store.events()]

    def test_submit_is_idempotent_and_ids_come_from_the_spec(self):
        a, created = self.store.submit({"n": 1, "x": [1, 2]})
        b, again = self.store.submit({"x": [1, 2], "n": 1})  # same content, other key order
        self.assertTrue(created)
        self.assertFalse(again)
        self.assertEqual(a, b)
        self.assertEqual(a, job_id_for({"n": 1, "x": [1, 2]}))
        self.assertEqual(self.store.counts()["queued"], 1)

    def test_queue_is_bounded(self):
        for n in range(4):
            self.store.submit({"n": n})
        with self.assertRaises(QueueFull):
            self.store.submit({"n": 4})
        lease = self.store.lease("w")
        self.store.complete(lease.job_id, lease.token, {"ok": 1})
        self.store.submit({"n": 4})  # a finished job frees a place
        self.assertEqual(self.store.submit({"n": 0})[1], False, "known jobs are accepted even when full")

    def test_leases_go_oldest_first_and_only_once(self):
        first, _ = self.store.submit({"n": 1})
        second, _ = self.store.submit({"n": 2})
        a, b = self.store.lease("w1"), self.store.lease("w2")
        self.assertEqual((a.job_id, b.job_id), (first, second))
        self.assertIsNone(self.store.lease("w3"))
        self.assertNotEqual(a.token, b.token)

    def test_heartbeat_keeps_a_lease_alive(self):
        self.store.submit({"n": 1})
        lease = self.store.lease("w1")
        for _ in range(5):
            self.clock.now += 8  # under the 10 s lease each time; 40 s in all
            self.store.heartbeat(lease.job_id, lease.token)
        self.assertEqual(self.store.counts()["leased"], 1)
        self.store.complete(lease.job_id, lease.token, {"ok": 1})
        self.assertEqual(self.store.counts()["done"], 1)

    def test_expired_lease_is_requeued_with_backoff_and_old_token_is_fenced(self):
        job, _ = self.store.submit({"n": 1})
        old = self.store.lease("w1")
        self.clock.now += 10.5  # w1 crashed or stalled: no heartbeat
        self.assertEqual(self.store.reap(), 1)
        self.assertIsNone(self.store.lease("w2"), "not before the backoff delay")
        self.assertEqual(self.store.jobs()[0]["state"], "queued")
        self.clock.now += 2.0  # backoff after attempt 1 is 2 s * [0.5, 1]
        new = self.store.lease("w2")
        self.assertEqual((new.job_id, new.attempt), (job, 2))
        with self.assertRaises(StaleLease):
            self.store.heartbeat(job, old.token)
        with self.assertRaises(StaleLease):
            self.store.complete(job, old.token, {"from": "w1"})
        digest = self.store.complete(job, new.token, {"from": "w2"})
        results = self.store.results()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["result"], {"from": "w2"})
        self.assertEqual(self.store.jobs()[0]["result_hash"], digest)
        self.assertIn("stale_result_rejected", self.kinds())
        self.assertIn("stale_heartbeat_rejected", self.kinds())

    def test_late_result_after_another_worker_finished_is_rejected(self):
        job, _ = self.store.submit({"n": 1})
        old = self.store.lease("w1")
        self.clock.now += 11
        self.store.reap()
        self.clock.now += 5
        new = self.store.lease("w2")
        self.store.complete(job, new.token, {"from": "w2"})
        with self.assertRaises(StaleLease):  # w1 wakes up and tries to publish
            self.store.complete(job, old.token, {"from": "w1"})
        self.assertEqual([r["result"] for r in self.store.results()], [{"from": "w2"}])
        self.assertEqual(self.kinds().count("completed"), 1)

    def test_repeating_a_completion_is_harmless(self):
        job, _ = self.store.submit({"n": 1})
        lease = self.store.lease("w1")
        a = self.store.complete(job, lease.token, {"v": 1})
        b = self.store.complete(job, lease.token, {"v": 1})  # reply was lost; worker retried
        self.assertEqual(a, b)
        with self.assertRaises(StaleLease):  # same lease, different result: not allowed
            self.store.complete(job, lease.token, {"v": 2})
        self.assertEqual(self.kinds().count("completed"), 1)

    def test_failures_retry_with_growing_backoff_then_give_up(self):
        job, _ = self.store.submit({"n": 1})
        delays = []
        for attempt in (1, 2, 3):
            lease = self.store.lease("w")
            self.assertEqual(lease.attempt, attempt)
            state = self.store.fail(job, lease.token, f"boom {attempt}")
            if state == "queued":
                row = self.store.jobs()[0]
                ready = [e for e in self.store.events() if e["kind"] == "requeued"][-1]
                delays.append(float(ready["detail"].split()[2]))
                self.clock.now += 1000  # wait out the backoff
        self.assertEqual(state, "failed")
        self.assertEqual(self.store.counts()["failed"], 1)
        self.assertTrue(1.0 <= delays[0] <= 2.0 and 2.0 <= delays[1] <= 4.0, delays)
        self.assertIn("gave_up", self.kinds())
        self.assertEqual(row["attempts"], 2)

    def test_backoff_is_capped(self):
        store = self.make(backoff_cap=5)
        self.assertTrue(all(store.backoff(k) <= 5 for k in range(1, 30)))
        store.close()

    def test_results_and_artifacts_are_content_addressed_and_survive_a_restart(self):
        job, _ = self.store.submit({"n": 1})
        lease = self.store.lease("w")
        digest = self.store.complete(job, lease.token, {"t_err": 0.43}, {"poses.txt": b"1 0 0 0\n"})
        self.store.close()
        self.store = self.make()  # the coordinator restarts on the same directory
        doc = self.store.results()[0]
        self.assertEqual(doc["result"], {"t_err": 0.43})
        self.assertEqual(self.store.get_object(doc["artifacts"]["poses.txt"]), b"1 0 0 0\n")
        self.assertEqual(digest, __import__("hashlib").sha256(canonical(doc)).hexdigest())
        self.assertEqual(self.store.put_object(b"1 0 0 0\n"), doc["artifacts"]["poses.txt"], "same bytes, same name")

    def test_corrupt_result_file_is_detected(self):
        job, _ = self.store.submit({"n": 1})
        lease = self.store.lease("w")
        digest = self.store.complete(job, lease.token, {"v": 1})
        path = os.path.join(self.dir.name, "objects", digest[:2], digest)
        with open(path, "r+b") as f:
            f.write(b"X")
        with self.assertRaises(IOError):
            self.store.results()

    def test_leases_survive_a_restart_and_still_expire(self):
        job, _ = self.store.submit({"n": 1})
        lease = self.store.lease("w1")
        self.store.close()
        self.store = self.make()
        self.store.heartbeat(job, lease.token)  # still valid after the restart
        self.clock.now += 11
        self.assertEqual(self.store.reap(), 1)
        self.assertEqual(json.loads(json.dumps(self.store.counts()))["queued"], 1)


    def test_restart_grace_saves_leases_that_could_not_heartbeat(self):
        job, _ = self.store.submit({"n": 1})
        lease = self.store.lease("w1")
        self.store.close()
        self.clock.now += 25  # coordinator down longer than the 10 s lease
        self.store = self.make()
        self.assertEqual(self.store.grant_restart_grace(), 1)
        self.store.heartbeat(job, lease.token)  # the worker reconnects: still its job
        self.clock.now += 10.5  # but a worker that never comes back still loses it
        self.assertEqual(self.store.reap(), 1)
        self.assertIn("restart_grace", self.kinds())

if __name__ == "__main__":
    unittest.main()
