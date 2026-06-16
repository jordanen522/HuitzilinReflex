#!/usr/bin/env python3
"""Quick plot of a Week 2 telemetry CSV. Run after a patrol session.

Usage: python3 scripts/plot_telemetry.py week2_telemetry_XXXX.csv
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt

csv_path = sys.argv[1] if len(sys.argv) > 1 else "week2_telemetry.csv"
df = pd.read_csv(csv_path)

fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))

a.plot(df["x"], df["y"])
a.set_title("Patrol ground track (ENU x/y)")
a.set_xlabel("x (m, East)")
a.set_ylabel("y (m, North)")
a.axis("equal")
a.grid(True)

t = df["t"] - df["t"].iloc[0]
b.plot(t, df["cmd_vx"], label="cmd vx", linestyle="--")
b.plot(t, df["vx"], label="meas vx")
b.legend()
b.set_title("Commanded vs measured vx")
b.set_xlabel("time (s)")
b.set_ylabel("m/s")
b.grid(True)

plt.tight_layout()
out = csv_path.replace(".csv", "_proof.png")
plt.savefig(out, dpi=120)
print(f"wrote {out}")
plt.show()
