#!/usr/bin/env bash
# lab_queue.sh — run a LIST of cells back-to-back, unattended.
#
# WHY THIS EXISTS
#   A cell is ~25 minutes and every one so far was started by hand, so the
#   campaign was serial on human attention rather than on the Dell. Worse, two
#   cells were lost to the same failure: lab_cell.sh reuses a stack that is
#   still alive, so everything delivered through EXTRA_LAUNCH (oracle_params,
#   evasion_params) lands on nothing and the battery re-flies the PREVIOUS
#   configuration under the new tag. RANGE and ORACLE_RATE appear to work
#   because they are launch arguments the new banner echoes, so nothing looks
#   wrong until the numbers are compared.
#
#   This script owns the two things that kept being forgotten:
#     * a HARD teardown between cells. lab_up.sh's own teardown kills by
#       process GROUP, which misses node processes that outlived the
#       `ros2 launch` that started them; and
#     * per-cell proof that the oracle came up with the configuration the cell
#       asked for, BEFORE the 25 minutes are spent (EXPECT_BANNER, below).
#
# QUEUE FILE
#   One cell per line, `KEY=VAL KEY=VAL ...`. Blank lines and # comments are
#   skipped. Keys are exactly lab_cell.sh's environment, so a queue line is a
#   literal transcription of the interactive command it replaces:
#
#     TAG=st23 RANGE=23.0 ORACLE_RATE=0.0 DODGE_SPEED=3.0 RESP_OUT=1 \
#       CONFIG=$LAB/config/hz20_r23.0.yaml BATTERY_ARGS="-p hover_mode:=true" \
#       EXTRA_LAUNCH="oracle_params:=$LAB/params/oracle_stereo.yaml" \
#       EXPECT_BANNER="0.3, 0.02, 0.02"
#
#   The line is eval'd, so $LAB and friends expand. That is deliberate: a queue
#   is lab input written by the operator, not untrusted data.
#
# USAGE
#   nohup setsid bash ~/hz_lab/bin/lab_queue.sh ~/hz_lab/config/q1.txt \
#     > /tmp/queue.log 2>&1 < /dev/null &
#
#   Results land in $LAB/results as usual, one set per TAG. Per-cell stdout
#   goes to $LOGDIR so a failed cell can be read without disturbing the queue.

source "$HOME/hz_tools/hz_env.sh"          # before `set -u` (CLAUDE.md)
set -u

LAB="${LAB:-$HOME/hz_lab}"
BIN="$LAB/bin"
QUEUE="${1:?usage: lab_queue.sh <queue-file>}"
LOGDIR="${LOGDIR:-$LAB/logs/queue_$(date +%Y%m%d_%H%M%S)}"

[ -r "$QUEUE" ] || { echo "!! no such queue file: $QUEUE" >&2; exit 2; }
mkdir -p "$LOGDIR"

# Every pattern is bracketed. Inside a script file the brackets are belt and
# braces (pkill's argv holds the pattern, not the loop body), but these get
# copy-pasted into ssh one-liners where an unbracketed pattern matches the
# invoking command line and kills the controlling session — exit 255, twice.
#
# NODES ARE KILLED BY INSTALLED PATH, never by guessed executable name. The
# first version of this list named `evasion_nod[e]` and `patrol_nod[e]`, but a
# colcon console_script installs as `lib/huitzilin_perception/evasion` and
# `lib/huitzilin_sim/patrol` — no `_node` suffix — so those two patterns
# matched NOTHING while `mav_bridg[e]`, `oracle_detecto[r]` and the rest
# matched fine. `gz_pose_bridge` and `telemetry_logger` were not listed at all.
# Six cells then ran with up to SEVEN complete stacks alive: duplicate
# /evasion nodes made `ros2 param set` and `ros2 param get` disagree,
# duplicate pose bridges fed hz_counterfactual an interleaved ball track that
# scored NO_FIT, and the derived ball speed read anywhere from 8 to 32 m/s for
# one 20 m/s throw — which the battery then blamed on a dropped impulse
# wrench. Every number was void and nothing in the output said so.
# `[h]uitzilin_ws/install/` matches every node in both packages regardless of
# what its entry point is called, which is the property the old list lacked.
PATTERNS='hz_counterfactua[l]
hz_dodge_respons[e]
dodge_batter[y]
[h]uitzilin_ws/install/
[r]os2 launch huitzilin
[p]arameter_bridge
gz-transport-topi[c]
[m]avproxy.py
[s]im_vehicle.py
[a]rducopter
[g]z sim'

hard_teardown() {
  local sig pat left
  for sig in TERM KILL; do
    # IFS=newline so "gz sim" and "ros2 launch huitzilin" survive as one
    # pattern each rather than word-splitting into "gz" and "sim".
    local IFS=$'\n'
    for pat in $PATTERNS; do
      pkill -"$sig" -f "$pat" 2>/dev/null
    done
    unset IFS
    sleep 6
  done
  # Truncated, not deleted: lab_up.sh redirects into it, and a STALE log is
  # exactly what lets a reused stack pass the range check on the PREVIOUS
  # cell's banner.
  : > /tmp/lab_stack.log
  # Survivors are counted against the SAME list that was just killed. The old
  # check counted only oracle/ArduPilot/Gazebo — which happen to be three
  # patterns that worked — so it printed `survivors=0` while seven complete
  # ROS stacks were still running, and the queue spent 40 minutes producing
  # data that looked entirely normal and meant nothing.
  local alt
  alt="$(printf '%s' "$PATTERNS" | tr '\n' '|')"
  left="$(pgrep -fc "${alt%|}" 2>/dev/null || true)"
  echo "[teardown] survivors=${left:-0}"
  # FAIL CLOSED. Continuing past a failed teardown is strictly worse than
  # stopping: a dirty stack does not error, it silently returns void numbers.
  if [ "${left:-0}" != "0" ]; then
    echo "!! teardown FAILED — these are still alive:"
    pgrep -af "${alt%|}"
    return 1
  fi
  return 0
}

echo "############ lab_queue $(date -Is)"
echo "queue:  $QUEUE"
echo "logs:   $LOGDIR"
echo "############"
grep -vE '^[[:space:]]*(#|$)' "$QUEUE" | nl -ba

n=0
SUMMARY=""
while IFS= read -r line || [ -n "$line" ]; do
  line="${line#"${line%%[![:space:]]*}"}"          # strip leading blanks
  [ -z "$line" ] && continue
  case "$line" in \#*) continue ;; esac

  n=$((n + 1))
  tag="$(printf '%s' "$line" | grep -oE '(^| )TAG=[^ ]+' | head -1 | cut -d= -f2)"
  tag="${tag:-cell$n}"

  echo ""
  echo "=========== [$n] $tag  start $(date -Is) ==========="
  echo "  $line"
  if ! hard_teardown; then
    echo "!! ABORTING QUEUE before [$n] $tag — every cell run on a dirty stack is void"
    SUMMARY="$SUMMARY$(printf '%-14s %s' "$tag" "NOT RUN (teardown failed)")
"
    break
  fi

  # stdin from /dev/null: the queue file is this loop's stdin, and a child that
  # reads stdin would swallow the remaining cells.
  eval "env $line bash '$BIN/lab_cell.sh'" > "$LOGDIR/$tag.log" 2>&1 < /dev/null
  rc=$?

  echo "----------- [$n] $tag  rc=$rc  end $(date -Is) -----------"
  tail -40 "$LOGDIR/$tag.log"
  SUMMARY="$SUMMARY$(printf '%-14s rc=%-3s %s' "$tag" "$rc" "$(date -Is)")
"
done < "$QUEUE"

hard_teardown

echo ""
echo "############ queue done $(date -Is)"
printf '%s' "$SUMMARY"
echo "############ logs in $LOGDIR"
