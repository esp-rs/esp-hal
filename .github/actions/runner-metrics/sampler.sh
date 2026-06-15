#!/usr/bin/env bash
# Background resource sampler for self-hosted runner capacity analysis.
#
# Appends a CSV row every <interval> seconds until it is killed or until
# <max_seconds> elapses (a safety net so a missed "report" step can never leave
# an orphan loop running on the long-lived self-hosted machine).
#
# Columns: ts (epoch s), load1 (1-min load avg), mem_used_pct, swap_used_mb
#
# Usage: sampler.sh <out_dir> [interval_s] [max_seconds]
#
# macOS / Apple Silicon only (uses sysctl + vm_stat). The Mac runner is the
# single shared machine we want to size, so portability is intentionally not a
# goal here.
set +e

DIR="${1:?out dir required}"
INTERVAL="${2:-5}"
MAX_SECONDS="${3:-7200}"

# The elapsed-time cutoff and `sleep` both need positive integers. A garbled or
# zero value must not disable the safety net (which would orphan this loop on
# the long-lived runner) or wedge `sleep`, so fall back to sane defaults.
case "$INTERVAL" in ''|*[!0-9]*) INTERVAL=5 ;; esac
[ "$INTERVAL" -lt 1 ] && INTERVAL=5
case "$MAX_SECONDS" in ''|*[!0-9]*) MAX_SECONDS=7200 ;; esac
[ "$MAX_SECONDS" -lt 1 ] && MAX_SECONDS=7200

CSV="$DIR/samples.csv"

PAGESIZE="$(sysctl -n hw.pagesize 2>/dev/null || echo 16384)"
TOTAL_MEM="$(sysctl -n hw.memsize 2>/dev/null || echo 0)"

echo "ts,load1,mem_used_pct,swap_used_mb" > "$CSV"

start="$(date +%s)"
while :; do
  now="$(date +%s)"
  [ $(( now - start )) -gt "$MAX_SECONDS" ] && break

  # 1-minute load average. `vm.loadavg` prints "{ 2.34 2.10 1.95 }".
  load1="$(sysctl -n vm.loadavg 2>/dev/null | awk '{print $2}')"

  # "Used" memory ~= active + wired + compressed (Activity Monitor's notion),
  # as a percentage of physical RAM. Inactive/speculative are reclaimable so we
  # exclude them; what really matters for the decision is swap (below).
  mem_pct="$(vm_stat 2>/dev/null | awk -v ps="$PAGESIZE" -v total="$TOTAL_MEM" '
    /Pages active/           { active=$3 }
    /Pages wired down/       { wired=$4 }
    /occupied by compressor/ { comp=$5 }
    END {
      gsub(/\./,"",active); gsub(/\./,"",wired); gsub(/\./,"",comp);
      used=(active+wired+comp)*ps;
      if (total>0) printf "%.1f", used/total*100; else printf "0";
    }')"

  # Swap "used" value, e.g. "vm.swapusage: total = 3072.00M used = 256.00M ...".
  # Reported in megabytes; strip the unit letter.
  swap_mb="$(sysctl -n vm.swapusage 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="used"){v=$(i+2); gsub(/[A-Za-z]/,"",v); print v; exit}}')"

  printf '%s,%s,%s,%s\n' "$now" "${load1:-0}" "${mem_pct:-0}" "${swap_mb:-0}" >> "$CSV"
  sleep "$INTERVAL"
done
