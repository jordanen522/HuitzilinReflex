#!/usr/bin/env bash
# Copy the probe harness to the Dell and strip the CRLF the Windows side adds.
set -eu
DELL="${DELL:-jordanen@192.168.0.123}"
KEY="${KEY:-$HOME/.ssh/id_ed25519_dell}"
DEST=/home/jordanen/huitzilin_ws/scripts/optics_probe
HERE="$(cd "$(dirname "$0")" && pwd)"
ssh -i "$KEY" -o BatchMode=yes "$DELL" "mkdir -p $DEST"
scp -i "$KEY" -o BatchMode=yes "$HERE"/run_probe.sh "$HERE"/count_ball_points.py \
  "$HERE"/probe_world.sdf.in "$DELL:$DEST/"
ssh -i "$KEY" -o BatchMode=yes "$DELL" \
  "cd $DEST && sed -i 's/\r\$//' * && chmod +x *.sh *.py && file run_probe.sh"
