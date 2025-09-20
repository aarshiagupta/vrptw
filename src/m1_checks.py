from pathlib import Path
import pandas as pd
p = Path("data/solomon_dataset/C1/C108.csv")  # change if needed
df = pd.read_csv(p)
# normalize headers once
df.columns = [c.strip().lower().replace('.', '').replace(' ', '_') for c in df.columns]

# find the depot robustly: demand==0 & service_time==0 & the first such row
depot_idx = df.query("demand == 0 and service_time == 0").index[0]
print("Depot row index:", depot_idx)
print(df.iloc[depot_idx])

total_demand = int(df["demand"].iloc[1:].sum())  # exclude depot
Q = 200  # Solomon C1 standard
lb_vehicles = (total_demand + Q - 1) // Q
print("Total customer demand:", total_demand)
print("Lower bound on vehicles (ceil(total/Q)):", lb_vehicles)
