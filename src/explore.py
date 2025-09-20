# Import necessary libraries
from pathlib import Path # Use pathlib to build OS-safe paths
import argparse, pandas as pd, matplotlib.pyplot as plt, numpy as np
REPO_ROOT = Path(__file__).resolve().parents[1] # repo root = parent of this script's folder (src/)
FIG_DIR = REPO_ROOT / "reports" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Normalize dataframe columns to a standard format
def normalize_columns(df):
    df = df.copy()
    df.columns = [c.strip().lower().replace(".", "").replace(" ", "_") for c in df.columns]
    return df

def pick(df, names):
    for n in names:
        if n in df.columns:
            return n
    raise KeyError(...)

# Parse command line arguments
ap = argparse.ArgumentParser()
default_data = REPO_ROOT / "data" / "solomon_dataset" / "C1" / "C108.csv" # default dataset if user doesn't pass --data
ap.add_argument("--data", default=str(default_data)) 
ap.add_argument("--capacity", type=int, default=200)
args = ap.parse_args()

data_path = Path(args.data)
if not data_path.is_absolute(): # if user gave a relative path, make it absolute based on the repo root
    data_path = (REPO_ROOT / data_path).resolve()
assert data_path.exists(), "Dataset not found" # fail fast if file doesn't exist

# Load and preprocess column names
df = pd.read_csv(data_path)
df = normalize_columns(df)
xcol = pick(df, ["xcoord","x"])
ycol = pick(df, ["ycoord","y"])
dcol = pick(df, ["demand"])
rtcol = pick(df, ["ready_time","readytime"])
ducol = pick(df, ["due_date","duedate"])
stcol = pick(df, ["service_time","servicetime"])

# Detect depot row: demand==0 & service_time==0 & first such row
depot_idx = df.query("demand == 0 and service_time == 0").index[0]

# Compute total demand, capacity, and lower bound on number of vehicles
total_demand = int(df[dcol].drop(index=depot_idx).sum())
Q = int(args.capacity)
lb = (total_demand + Q - 1) // Q
print("File:", data_path.name)
print("Depot row index:", depot_idx)
print("Total customers (excl. depot):", len(df) - 1)
print("Total demand:", total_demand)
print("Capacity Q:", Q, " -> Lower bound on vehicles:", lb)

# Scatter plot of locations, colored by ready time
depot_x, depot_y = df.loc[depot_idx, [xcol, ycol]]
is_customer = np.ones(len(df), dtype=bool); is_customer[depot_idx] = False

plt.figure(figsize=(7,6))
sc = plt.scatter(df.loc[is_customer, xcol], df.loc[is_customer, ycol],
                 s=18, c=df.loc[is_customer, rtcol], cmap="viridis")
plt.scatter([depot_x], [depot_y], s=120, marker="*", edgecolors="k", linewidths=0.7, label="Depot")
plt.colorbar(sc).set_label("Ready time")
plt.title(f"{data_path.stem} — Locations (colored by Ready Time)")
plt.xlabel("X"); plt.ylabel("Y"); plt.legend()
plt.tight_layout()
out1 = FIG_DIR / f"{data_path.stem}_scatter.png"
plt.savefig(out1, dpi=180)


# Histogram of customer demands
plt.figure(figsize=(7,4))
df.loc[is_customer, dcol].plot(kind="hist", bins=15, edgecolor="black")
plt.title(f"{data_path.stem} — Demand distribution")
plt.xlabel("Demand"); plt.ylabel("Count of customers")
plt.tight_layout()
out2 = FIG_DIR / f"{data_path.stem}_demand_hist.png"
plt.savefig(out2, dpi=180)


# Histogram of time-window widths (due - ready)
tw_width = (df.loc[is_customer, ducol] - df.loc[is_customer, rtcol]).clip(lower=0)
plt.figure(figsize=(7,4))
tw_width.plot(kind="hist", bins=15, edgecolor="black")
plt.title(f"{data_path.stem} — Time-window width (Due − Ready)")
plt.xlabel("Window width (minutes)"); plt.ylabel("Count of customers")
plt.tight_layout()
out3 = FIG_DIR / f"{data_path.stem}_tw_width_hist.png"
plt.savefig(out3, dpi=180)


# 
print("Saved:\n •", out1, "\n •", out2, "\n •", out3)
