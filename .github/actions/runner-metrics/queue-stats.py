#!/usr/bin/env python3
"""Queue/demand analysis for the shared Mac self-hosted runner.

The `runner-metrics` action measures *capacity* (is one job already saturating
the machine?). This script measures the other half of the decision — *demand*:
do jobs actually wait for a free runner, and do the two `esp-hal` matrix legs
run concurrently or one-after-another?

It reads the GitHub Actions API (via the `gh` CLI, so auth is inherited) for the
most recent `ci.yml` runs and reports, for the `macos-m1-self-hosted` jobs:

  * queue wait  (started_at - created_at)  distribution
  * run duration (completed_at - started_at) distribution
  * how many multi-job runs serialized (one runner) vs overlapped (>1 runner)

Combine with the per-job utilization reports:
  headroom in the job summaries + jobs queuing here  =>  add a runner.
  no queuing, or the machine is already saturated     =>  don't.

Usage:
  ./queue-stats.py [--repo OWNER/REPO] [--runs N] [--workflow FILE] [--label L]

Requires the `gh` CLI, authenticated against the repo.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime


def gh_api(path: str) -> dict:
    """Call `gh api <path>` and return parsed JSON."""
    try:
        out = subprocess.run(
            ["gh", "api", path],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except FileNotFoundError:
        sys.exit("error: the `gh` CLI is required and was not found on PATH")
    except subprocess.CalledProcessError as e:
        sys.exit(f"error: `gh api {path}` failed:\n{e.stderr.strip()}")
    return json.loads(out)


def parse_ts(value: str | None) -> float | None:
    if not value:
        return None
    return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


def percentile(values: list[float], pct: float) -> float:
    """Nearest-rank percentile of an already-collected list."""
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = max(0, min(len(ordered) - 1, round(pct / 100 * (len(ordered) - 1))))
    return ordered[rank]


def fmt_dist(label: str, values: list[float]) -> str:
    if not values:
        return f"  {label}: (no data)"
    return (
        f"  {label}: min {percentile(values, 0):.0f}s  "
        f"median {percentile(values, 50):.0f}s  "
        f"avg {sum(values) / len(values):.0f}s  "
        f"p90 {percentile(values, 90):.0f}s  "
        f"max {percentile(values, 100):.0f}s"
    )


def overlaps(jobs: list[dict]) -> bool:
    """True if any two of these jobs' [started, completed] intervals overlap."""
    intervals = sorted(
        (j["started"], j["completed"])
        for j in jobs
        if j["started"] is not None and j["completed"] is not None
    )
    for (s_prev, e_prev), (s_next, _e_next) in zip(intervals, intervals[1:]):
        if s_next < e_prev:
            return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default="esp-rs/esp-hal")
    ap.add_argument("--runs", type=int, default=30, help="how many recent runs to scan")
    ap.add_argument("--workflow", default="ci.yml")
    ap.add_argument("--label", default="macos-m1-self-hosted")
    args = ap.parse_args()

    runs = gh_api(
        f"repos/{args.repo}/actions/workflows/{args.workflow}/runs?per_page={args.runs}"
    )["workflow_runs"]

    jobs: list[dict] = []
    runs_with_match = 0
    serialized = overlapped = 0

    for run in runs:
        run_jobs = gh_api(f"repos/{args.repo}/actions/runs/{run['id']}/jobs")["jobs"]
        matched = [
            {
                "name": j["name"],
                "created": parse_ts(j.get("created_at")),
                "started": parse_ts(j.get("started_at")),
                "completed": parse_ts(j.get("completed_at")),
            }
            for j in run_jobs
            if args.label in (j.get("labels") or [])
        ]
        if not matched:
            continue
        runs_with_match += 1
        jobs.extend(matched)

        timed = [
            j
            for j in matched
            if j["started"] is not None and j["completed"] is not None
        ]
        if len(timed) >= 2:
            if overlaps(timed):
                overlapped += 1
            else:
                serialized += 1

    queue_waits = [
        j["started"] - j["created"]
        for j in jobs
        if j["started"] is not None and j["created"] is not None
    ]
    run_durations = [
        j["completed"] - j["started"]
        for j in jobs
        if j["completed"] is not None and j["started"] is not None
    ]

    print(
        f"Mac runner queue analysis — {args.repo} · {args.workflow} · "
        f"last {len(runs)} runs"
    )
    print(f"Label: {args.label}\n")

    if not jobs:
        print(f"No `{args.label}` jobs found in the scanned runs.")
        return

    print(f"Jobs analyzed: {len(jobs)} across {runs_with_match} runs\n")
    print("Queue wait (started - created):")
    print(fmt_dist("wait", queue_waits) + "\n")
    print("Run duration (completed - started):")
    print(fmt_dist("dur ", run_durations) + "\n")

    multi = serialized + overlapped
    print(f"Runs with >=2 Mac jobs: {multi}")
    print(f"  serialized (one-after-another): {serialized}   <- suggests a single runner")
    print(f"  overlapped (ran concurrently):  {overlapped}")
    print()
    print(
        "Reading this: high queue waits and/or serialized matrix legs mean jobs are\n"
        "waiting for a free runner. If the per-job utilization reports also show the\n"
        "machine has spare capacity, registering another runner should help."
    )


if __name__ == "__main__":
    main()
