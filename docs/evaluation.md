# Distributed evaluation (Phase 4)

## Why

Every Phase 3 number came from one command run by hand. A real question,
such as "how sensitive is Atlas to voxel size, deskewing and the KITTI scan
correction, on both sequences?", is 24 runs of up to a minute and a half
each. That is a job for several workers, and running it has to survive the
things that go wrong on real fleets: a worker dies, a worker stalls, the
coordinator restarts, a reply is lost. Phase 4 builds the smallest system
that handles those, and demonstrates each one on purpose.

The estimation core is unchanged: workers run the same C++ binary
(`atlas_kitti run`). Python (standard library only, 3.9+) does the
coordination; no new dependencies.

## The pieces

```
  submitter --POST /jobs--> coordinator --(SQLite + result files)--> store/
   (backs off on 429)          |   ^                                   |
                    POST /lease|   |POST /heartbeat, /complete, /fail  |
                               v   |                                   |
                  worker w1, w2, ... --runs--> atlas_kitti run ...     |
                                                                       v
                                          report.md <-- built from the store alone
```

| File | Role |
| --- | --- |
| `eval/atlas_eval/store.py` | Durable job state (SQLite, WAL, synchronous=FULL) and content-addressed result files. All the rules live here. |
| `eval/atlas_eval/coordinator.py` | HTTP JSON front end; a reaper thread requeues expired leases every second. |
| `eval/atlas_eval/worker.py` | Lease, run with heartbeats, publish; retries the coordinator with backoff. |
| `eval/atlas_eval/runners.py` | What a job does: `atlas_kitti run` for KITTI, or a sleep-and-square test job. |
| `eval/atlas_eval/sweep.py` | Parameter grid to job specs; submits with backpressure. |
| `eval/atlas_eval/report.py` | Markdown report from the stored results. |
| `eval/atlas_eval/demo.py` | The end-to-end run with deliberate crashes. |
| `eval/cli.py` | Command line for all of the above. |
| `apps/kitti.cpp` `run` | One configured scan-to-map run, streaming; prints one JSON line. |

## The rules, in plain words

**A job's name is its content.** The job id is the SHA-256 of its spec
(sorted-key JSON): sequence, parameters, and the hash of the `atlas_kitti`
binary. Submitting the same sweep twice adds nothing. Rebuilding the engine
changes the hash, so old results can never be mistaken for new code's.

**The queue is bounded.** At most `capacity` jobs are queued or running.
Beyond that, `POST /jobs` answers 429 and the submitter waits with
exponential backoff. (Backpressure: a producer cannot bury the system.)

**A lease is a time-limited claim.** A worker that leases a job gets a random
token and must heartbeat every third of the lease. If the heartbeats stop
(crash, pause, network cut), the lease expires and the job is queued again.

**Old leases are fenced off.** Every heartbeat, completion and failure must
carry the current token. A worker that wakes up after its lease expired is
refused (HTTP 409) even if it finished the work. So a job may be *executed*
more than once, but only one result is ever *accepted*: at-least-once
execution, one accepted result per job. This is not exactly-once execution,
and the system does not claim it.

**Retries back off.** A failed or expired attempt waits
min(60 s, 2 s x 2^(attempt-1)) times a random factor in [0.5, 1] before it
can run again; after 3 attempts the job is marked failed (kept, not lost).
Workers retry an unreachable coordinator the same way (0.2 s doubling to 5 s,
for up to 2 minutes), which is what lets the coordinator restart under them.

**Results are durable and content-addressed.** A result is written to a
temporary file, fsynced, renamed to its SHA-256, and its directory fsynced,
before the database transaction that marks the job done. A crash in between
leaves an unreferenced file and a job that runs again: wasted work, never a
done job without its result. Reading a result re-checks its hash. The poses
file is stored the same way and referenced from the result.

**A restarted coordinator honours old leases.** While it was down, workers
could not heartbeat. On start it gives every live lease one full period
(`grant_restart_grace`), so an outage does not turn into a wave of repeated
jobs. A worker that really died meanwhile costs one extra lease period.

**Repeats are harmless.** If a worker's completion reply is lost and it sends
the same completion again, the coordinator returns the same answer.

## Run it

```bash
cmake --build build --parallel                     # the engine the workers run
python3 eval/cli.py coordinator --store results/eval/run1 &
python3 eval/cli.py worker --name w1 --engine build/atlas_kitti \
    --odometry ~/Desktop/Datasets/kitti/odometry/dataset \
    --velodyne 04=<drive_0016>/velodyne_points/data 07=<drive_0027>/velodyne_points/data &
python3 eval/cli.py sweep --engine build/atlas_kitti --sequences 04 07 \
    --grid voxel=0.75,1.0,1.5 deskew=0,1 correct=0,1
python3 eval/cli.py status
python3 eval/cli.py report --store results/eval/run1
```

The whole thing, with the failures injected (see below):

```bash
python3 eval/cli.py demo --kitti --store results/eval/kitti_sweep --engine build/atlas_kitti \
    --odometry <odometry_dir> --velodyne 04=<dir> 07=<dir> --workers 3 --lease 6 --capacity 8
python3 eval/cli.py demo --store /tmp/demo     # same, with test jobs: no KITTI data needed
```

## What the demo does to it

1. Starts a coordinator (lease 6 s, queue capacity 8), the submitter, and
   workers w1-w3.
2. **Worker crash:** `kill -9 w1` while it runs a job. Its lease expires; the
   job is requeued with backoff and another worker runs it. w4 joins.
3. **Coordinator crash:** `kill -9` the coordinator, wait 4 s, start it again
   on the same directory. Workers and the submitter retry and carry on.
4. **Long pause:** `SIGSTOP w2` for 14 s (more than twice its lease), then
   `SIGCONT`. Its job was re-leased meanwhile; when w2 wakes, its heartbeat
   (and, if it got that far, its result) is refused.
5. Waits for everything to finish and writes `report.md` from the store.

## Result: the KITTI sweep under failures (2026-11-24, after the fixes)

24 jobs (sequences 04 and 07 x voxel 0.75/1.0/1.5 m x deskew on/off x
KITTI correction on/off), 3 workers plus 1 joining, one Apple M4, 227 s:

```
[  1.2 s] CRASH: kill -9 w1 while it runs job 6f9ea4328ea5f879
[  3.2 s] w4 joins
[  3.2 s] CRASH: kill -9 the coordinator (state before: queued 5, leased 3, done 0)
[  7.3 s] coordinator restarted on the same store: queued 5, leased 3, done 0
[  7.8 s] PAUSE: SIGSTOP w2 for 14 s (longer than its 6 s lease) while it runs job 8526837290299806
[ 21.9 s] w2 resumes; its old lease is gone
[226.9 s] finished: queued 0, leased 0, done 24, failed 0
          every job done with exactly one accepted result: True
```

| What happened | Count |
| --- | --- |
| Jobs | 24, all done |
| Attempts started | 26 (24 + the 2 interrupted jobs, run again) |
| Leases expired | 2: crashed w1's and paused w2's, nothing else |
| Late heartbeat / late result refused | 1 / 1 (w2 waking up: its finished run was discarded) |
| Results accepted | 24, one per job |

w1's lease expired at 13 s rather than 7 s because the coordinator restart
gave live leases a fresh 6 s; the queue was full (capacity 8) from the
start, so the submitter was held back by 429s throughout. The accuracy
numbers were identical to the first run's, as they should be: the engine is
deterministic. (Result hashes differ between runs because results include
timings.)

What the sweep says about the method, KITTI t_err (default in bold):

| Sequence | **Default** (voxel 1.0, deskew, correction) | No deskew | No correction | Neither | voxel 0.75 / 1.5 |
| --- | --- | --- | --- | --- | --- |
| 04 (country road) | **0.48 %** | 0.58 % | 1.07 % | 1.11 % | 0.48 % / 0.52 % |
| 07 (city loop, held out) | **0.43 %** | 0.76 % | 0.54 % | 0.88 % | 0.44 % / 0.49 % |

- Both fixes matter on both drives, but in a different order: the KITTI
  scan correction matters most on the straight country road (0.48 -> 1.07 %),
  deskewing on the turning city loop (0.43 -> 0.76 %). A plausible reading
  (not tested separately): the correction fixes a small elevation-angle
  error that mostly distorts distance along a straight road, while
  deskewing undoes the smear of a sweep taken while turning.
- Voxel size is flat between 0.75 and 1.0 m (0.75 m is about 1.7x slower)
  and a little worse at 1.5 m (about 2x faster).
- The default, chosen on sequence 04 before 07 was downloaded, is best or
  tied-best on both. The sweep is a sensitivity study; no parameter was
  re-chosen from it.

The full table is in the generated `report.md` (results/, not in Git).

## Two problems the first KITTI run found

The first full run (24 jobs) ended correctly: every job done, one accepted
result each. Its logs still showed two things wrong, both now fixed and
tested.

1. **A finished job's heartbeat cancelled the next job (worker bug).** The
   heartbeat thread read the job id, token and "lost" flag from the worker
   loop's variables when it used them, not when it started (Python closures
   bind late). One heartbeat was still retrying through the coordinator
   restart when its job finished; its reply was 409 ("that job is done"), and
   it set the NEW job's "lost" flag. The worker abandoned a job whose lease
   was fine, and the job waited for its lease to expire. Fix: the thread gets
   its values as arguments, and the worker joins it before leasing again.
   Regression test: heartbeats that take 0.4 s to answer, ten 0.6 s jobs;
   with the old code 6 of 10 jobs were abandoned, with the fix none.
2. **A coordinator outage outlasted the leases (design gap).** The
   coordinator was down about 4 s plus the workers' retry delay; a 6 s lease
   last renewed before the crash expired right after the restart, and a
   healthy job ran again. Fix: restart grace (above). Test: a lease taken a
   minute before the restart can still heartbeat after it.

## Tests

`python3 -m unittest discover -s eval/tests -v` (19 tests, about a minute;
runs in CI on Linux):

- Store, with a fake clock: idempotent submission (key order does not
  matter), bounded queue, oldest-first leasing, heartbeats keep a lease alive,
  expiry requeues with backoff and fences the old token (heartbeat and
  result), a late result after another worker finished is rejected, repeated
  completion is harmless but a different result on the same lease is not,
  retries grow then give up, backoff is capped, results and artifacts survive
  a reopen and are content-addressed, a corrupted result file is detected,
  leases survive a coordinator restart and still expire.
- HTTP: backpressure (429), a "zombie" worker whose lease expired cannot
  publish before or after the real result, malformed requests are refused.
- Worker: a finished job's late heartbeat cannot cancel the next job.
- Restart: a coordinator restarted after a long outage accepts heartbeats on
  leases from before it.
- Processes: the test-job demo (40 jobs): worker kill -9, coordinator kill -9
  and restart, a worker paused past its lease; every job done, exactly one
  accepted result each, at least two expired leases, the paused worker
  fenced off, and every stored answer correct.

Deliberate breaks, each caught: accepting a completion without the current
token (3 tests fail), no capacity check, never reaping expired leases, no
backoff, job ids from unsorted JSON, no attempt limit, the coordinator not
granting restart grace, and the original late-binding heartbeat.

## Limits

- One coordinator, one SQLite file: the coordinator is a single point of
  failure. It restarts cleanly but is not replicated. No authentication:
  it listens on 127.0.0.1 by default.
- Workers here are processes on one Mac, not separate machines. The protocol
  is plain HTTP and would work across machines, but that was not tested.
- At-least-once execution: a job can run twice (the demo shows it). The
  guarantee is one accepted result per job, not exactly-once execution.
- Lease expiry uses the coordinator's wall clock. A large clock jump on the
  coordinator would expire or extend leases wrongly.
- A worker killed with `kill -9` leaves its `atlas_kitti` child running as an
  orphan until it finishes (it cannot publish: no lease). Linux could tie the
  child to the parent (PR_SET_PDEATHSIG); macOS has no direct equivalent.
- Result files from rejected completions are kept (unreferenced); there is no
  garbage collection.
- Timings from a sweep come from several runs sharing the machine and are
  not comparable to single-run timings.
