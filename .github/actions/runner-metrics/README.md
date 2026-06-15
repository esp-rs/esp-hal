# runner-metrics

Samples CPU load / memory / swap on the **`macos-m1-self-hosted`** machine while
a job runs, so we can decide whether registering **more runners on the same
machine** would actually improve CI throughput.

## Why

Adding runners to one physical machine only helps if **both** are true:

1. **There is spare capacity** — a single job does not already saturate the
   CPU/RAM. → measured here (this action).
2. **Jobs actually queue** — work is waiting for a free runner. → measured by
   `queue-stats.py` (see below), not by this action.

`cargo-batch` parallelizes across all cores, so a single build job may already
peg the CPU; if so, a second concurrent runner just splits the same cores and
throughput won't improve (and concurrent builds risk swapping). This action
makes that visible per run.

## Usage

```yaml
steps:
  - uses: actions/checkout@v6          # required: local action needs the repo
  - uses: ./.github/actions/runner-metrics
    with: { mode: start }
  # ... the real job steps ...
  - if: always()
    uses: ./.github/actions/runner-metrics
    with:
      mode: report
      name: ${{ matrix.group.toolchain }}   # unique per matrix leg
```

`mode: start` launches a background sampler (`sampler.sh`, every `interval`s,
default 5). `mode: report` stops it, writes a verdict table to the job summary,
and uploads an artifact.

## Output

- **Job summary**: machine facts + avg/peak load (absolute and as % of cores),
  memory, swap, and an auto-verdict (memory-bound / CPU-saturated / headroom /
  moderate).
- **Artifact** `runner-metrics-<name>-<run_id>` containing:
  - `samples.csv` — `ts,load1,mem_used_pct,swap_used_mb`
  - `summary.json` — one machine-readable record per job (run_id, label, core
    count, load/mem/swap aggregates, duration).
  - `info.env` — static machine facts.

## The queue/demand side: `queue-stats.py`

The capacity signal above is only half the decision. To answer *"do jobs wait
for a free runner?"* run `queue-stats.py`, which pulls the GitHub Actions API
over recent runs and, for the `macos-m1-self-hosted` jobs, reports:

- queue wait (`started_at - created_at`) vs run duration (`completed_at -
  started_at`) distributions, and
- whether the two `esp-hal` matrix legs **serialized** (one started only after
  the other finished → only one runner today) or overlapped.

```sh
# requires the `gh` CLI, authenticated against the repo
.github/actions/runner-metrics/queue-stats.py            # last 30 ci.yml runs
.github/actions/runner-metrics/queue-stats.py --runs 50  # scan more history
```

Combine: **headroom (this action) + jobs queuing (`queue-stats.py`) ⇒ add a
runner.** No queuing, or already saturated ⇒ don't.
