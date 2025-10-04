import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

on = pd.read_csv("data/out/metrics_on.csv")
off = pd.read_csv("data/out/metrics_off.csv")

def describe(name, df):
    print(f"\n=== {name} ===")
    print("n =", len(df))
    print("avg stop_time_sec   =", df["stop_time_sec"].mean().round(2))
    print("avg clear_time_sec  =", df["clear_time_sec"].mean().round(2))
    print("avg stops           =", df["stops"].mean().round(2))

describe("BASELINE (OFF)", off)
describe("PREEMPTION (ON)", on)

# Simple bar chart for report/poster
m = pd.DataFrame({
    "baseline_off": [off["stop_time_sec"].mean(), off["clear_time_sec"].mean()],
    "preemption_on": [on["stop_time_sec"].mean(), on["clear_time_sec"].mean()],
}, index=["avg_stop_time_s", "avg_clear_time_s"])

Path("data/out").mkdir(parents=True, exist_ok=True)
ax = m.plot(kind="bar")
ax.set_title("EMS Delay Metrics (lower is better)")
ax.set_ylabel("seconds")
plt.tight_layout()
plt.savefig("data/out/ems_metrics_compare.png", dpi=160)
print("\n[OK] Saved plot -> data/out/ems_metrics_compare.png")
