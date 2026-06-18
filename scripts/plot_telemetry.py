#!/usr/bin/env python3
"""Quick plot of a Week 2 telemetry CSV. Run after a patrol session.

Usage: python3 scripts/plot_telemetry.py week2_telemetry_XXXX.csv

CSV schema (from telemetry_logger.py): t,x,y,z,vx,vy,vz,cmd_vx,cmd_vy,cmd_vz
Frames: odom topic is ENU (x=East, y=North, z=Up).

Outputs <csv>_proof.png with three panels:
  1. Patrol ground track (ENU x/y) with the 4 waypoints overlaid
  2. Altitude hold (z vs time)
  3. Commanded vs measured forward velocity (vx)
"""
import sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")          # headless-safe (WSL has no display); savefig still works
import matplotlib.pyplot as plt

# Patrol waypoints, converted NED [n,e,d] -> ENU (x=e, y=n) for the odom frame.
# patrol.yaml square: (5,0,-2) (5,5,-2) (0,5,-2) (0,0,-2)  ->  ENU x/y below.
WAYPOINTS_ENU = [(0, 5), (5, 5), (5, 0), (0, 0)]
TARGET_ALT_M = 2.0

csv_path = sys.argv[1] if len(sys.argv) > 1 else "week2_telemetry.csv"
df = pd.read_csv(csv_path)
t = df["t"] - df["t"].iloc[0]

fig, (a, b, c) = plt.subplots(1, 3, figsize=(15, 4.2))

# --- 1. ground track + waypoints ---------------------------------------------
a.plot(df["x"], df["y"], lw=0.8, color="tab:blue", label="flown track")
wx = [p[0] for p in WAYPOINTS_ENU] + [WAYPOINTS_ENU[0][0]]
wy = [p[1] for p in WAYPOINTS_ENU] + [WAYPOINTS_ENU[0][1]]
a.plot(wx, wy, "--", color="tab:red", lw=1.0, alpha=0.7, label="planned loop")
a.scatter([p[0] for p in WAYPOINTS_ENU], [p[1] for p in WAYPOINTS_ENU],
          s=90, color="tab:red", zorder=5)
for i, (px, py) in enumerate(WAYPOINTS_ENU):
    a.annotate(f"WP{i}", (px, py), textcoords="offset points", xytext=(6, 6))
a.set_title("Patrol ground track (ENU)")
a.set_xlabel("x (m, East)"); a.set_ylabel("y (m, North)")
a.axis("equal"); a.grid(True); a.legend(loc="upper right", fontsize=8)

# --- 2. altitude hold --------------------------------------------------------
c2 = b
c2.plot(t, df["z"], color="tab:green", label="altitude (z, up)")
c2.axhline(TARGET_ALT_M, ls="--", color="gray", lw=1, label=f"target {TARGET_ALT_M:g} m")
c2.set_title("Altitude hold")
c2.set_xlabel("time (s)"); c2.set_ylabel("z (m, Up)")
c2.grid(True); c2.legend(loc="lower right", fontsize=8)

# --- 3. commanded vs measured forward velocity -------------------------------
c.plot(t, df["cmd_vx"], "--", label="cmd vx")
c.plot(t, df["vx"], label="meas vx")
c.set_title("Commanded vs measured vx")
c.set_xlabel("time (s)"); c.set_ylabel("m/s")
c.grid(True); c.legend(loc="upper right", fontsize=8)

plt.tight_layout()
out = csv_path.replace(".csv", "_proof.png")
plt.savefig(out, dpi=120)
print(f"wrote {out}")
