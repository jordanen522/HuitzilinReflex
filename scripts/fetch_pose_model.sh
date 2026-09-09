#!/usr/bin/env bash
# fetch_pose_model.sh -- download the pose model the detector needs.
#
# The model is a 13 MB binary and is NOT tracked in this repository. It lives
# outside the tree so a clone stays small and so the weights are never
# something this project appears to redistribute.
#
# USAGE:
#   ./scripts/fetch_pose_model.sh            # into ~/models
#   ./scripts/fetch_pose_model.sh /opt/models
#
# YOLOv8n-pose, ONNX export, 17 COCO keypoints. Five of those are the face and
# are discarded by pose_detector.decode before anything is published; the model
# computing them is not the same as this system carrying them.

set -euo pipefail

DEST="${1:-$HOME/models}"
NAME="yolov8n-pose.onnx"
URL="https://huggingface.co/Xenova/yolov8n-pose/resolve/main/onnx/model.onnx"
SHA="04f6d2416266f2aba6c5ba8b26de33ed9eba3279f972ac02e33f9f1366547586"

mkdir -p "$DEST"
OUT="$DEST/$NAME"

if [ -f "$OUT" ] && echo "$SHA  $OUT" | sha256sum -c --status; then
  echo "already present and verified: $OUT"
  exit 0
fi

echo "fetching $NAME into $DEST"
curl -fL --retry 3 -o "$OUT" "$URL"

# Verify before use, not after a confusing failure downstream. A truncated or
# substituted model loads far enough to produce plausible-looking garbage.
if ! echo "$SHA  $OUT" | sha256sum -c --status; then
  echo "ERROR: checksum mismatch for $OUT" >&2
  echo "expected $SHA" >&2
  echo "got      $(sha256sum "$OUT" | cut -d" " -f1)" >&2
  exit 1
fi

echo "verified: $OUT"
echo
echo "onnxruntime is required and is not a ROS package. The system interpreter"
echo "is externally managed (PEP 668), so install it beside the workspace:"
echo "  python3 -m pip install --target ~/.local/ros-deps onnxruntime"
echo "  rm -rf ~/.local/ros-deps/numpy*   # keep the system numpy/scipy pair"
echo "then run nodes with PYTHONPATH=~/.local/ros-deps"
