from pathlib import Path
import argparse
import math
import pandas as pd
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

ROOT = Path(__file__).resolve().parents[1]  # repo root = parent of this script's folder (src/)

# Normalize dataframe columns to a standard format
def _normalize_headers(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [c.strip().lower().replace(".", "").replace(" ", "_") for c in df.columns]
    return df

# Compute Euclidean distance matrix
def _euclid_matrix(x, y):
    n = len(x)
    dist = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            dist[i][j] = int(round(math.hypot(x[i]-x[j], y[i]-y[j])))
    return dist

# Compute lower bound on number of vehicles needed
def _lower_bound_vehicles(total_demand: int, Q: int) -> int:
    return (int(total_demand) + int(Q) - 1) // int(Q)

# Load Solomon CSV dataset and preprocess
def load_solomon_csv(path: Path):
    df = pd.read_csv(path)
    df = _normalize_headers(df)
    # depot = the row with demand=0 and service_time=0
    depot_idx = df.query("demand == 0 and service_time == 0").index[0]
    # move depot to the top so it is node 0
    if depot_idx != 0:
        df = pd.concat([df.iloc[[depot_idx]], df.drop(index=depot_idx)], ignore_index=True)

    data = {
        "x": df["xcoord"].to_numpy(),
        "y": df["ycoord"].to_numpy(),
        "demand": df["demand"].astype(int).to_numpy(),
        "ready": df["ready_time"].astype(int).to_numpy(),
        "due": df["due_date"].astype(int).to_numpy(),
        "service": df["service_time"].astype(int).to_numpy(),
        "depot": 0,
    }
    data["dist"] = _euclid_matrix(data["x"], data["y"])
    return data


def save_routes_csv(manager, routing, solution, num_vehicles, out_path):
    rows = []
    for v in range(num_vehicles):
        idx = routing.Start(v)
        nxt = solution.Value(routing.NextVar(idx))
        if routing.IsEnd(nxt):
            continue
        seq = 0
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            rows.append({"vehicle": v+1, "seq": seq, "node_idx": node})
            idx = solution.Value(routing.NextVar(idx))
            seq += 1
        rows.append({"vehicle": v+1, "seq": seq, "node_idx": manager.IndexToNode(idx)})
    import pandas as pd
    pd.DataFrame(rows).to_csv(out_path, index=False)



# Solve VRPTW using OR-Tools
def solve_vrptw(csv_path: Path, Q: int, vehicles_arg, time_limit: int):
    d = load_solomon_csv(csv_path)
    n = len(d["x"])
    depot = d["depot"]

    # how many vehicles are allowed?
    if vehicles_arg in (None, "auto"):
        lb = _lower_bound_vehicles(int(d["demand"][1:].sum()), Q)
        num_vehicles = lb + 5          # small buffer above the lower bound
    else:
        num_vehicles = int(vehicles_arg)

    # set up the routing model using OR-Tools
    manager = pywrapcp.RoutingIndexManager(n, num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    # distance callback - cost to minimize
    def dist_cb(from_index, to_index):
        i, j = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        return d["dist"][i][j]
    dist_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)

    # capacity constraint
    def demand_cb(index):
        node = manager.IndexToNode(index)
        return int(d["demand"][node])   # depot is 0
    dem_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        dem_idx,
        0,                              # no extra slack for load
        [int(Q)] * num_vehicles,        # each truck holds Q
        True,                           # start at 0 load
        "Capacity",
    )


    # Time dimension (Travel + Service time) 
    # time from i -> j = service at i + travel(i,j)
    def time_cb(from_index, to_index):
        i, j = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        travel = d["dist"][i][j]
        return int(travel + d["service"][i])  # depot service is 0
    time_idx = routing.RegisterTransitCallback(time_cb)

    # safe horizon and plenty of waiting (slack) so trucks can wait if early
    horizon = int(max(d["due"])) + int(max(d["service"])) + 100  # small cushion
    routing.AddDimension(
        time_idx,
        10_000,     # slack (waiting) allowed
        horizon,    # max route time
        True,       # FIX START CUMUL TO ZERO (start time = 0 at depot)
        "Time",
)
    time_dim = routing.GetDimensionOrDie("Time")

    # 1) Apply time windows to all CUSTOMER nodes (skip depot here)
    for node in range(1, n):
        idx = manager.NodeToIndex(node)
        time_dim.CumulVar(idx).SetRange(int(d["ready"][node]), int(d["due"][node]))

    # 2) Apply depot window to EACH vehicle's start and end cumul variables
    depot_ready, depot_due = int(d["ready"][depot]), int(d["due"][depot])
    for v in range(num_vehicles):
        s = routing.Start(v)
        e = routing.End(v)
        time_dim.CumulVar(s).SetRange(depot_ready, depot_due)
        time_dim.CumulVar(e).SetRange(depot_ready, depot_due)
        # Help the solver pick tight, feasible start/end times
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(s))
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(e))

    # Search parameters
    sp = pywrapcp.DefaultRoutingSearchParameters()
    sp.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    sp.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    sp.time_limit.FromSeconds(int(time_limit))

    # solve ; set time limit
    solution = routing.SolveWithParameters(sp)
    if not solution:
        raise RuntimeError("No solution found (try raising --time_limit or vehicles).")

    total_distance = 0
    vehicles_used = 0

    print("\n==== Baseline VRPTW (OR-Tools) ====")
    print(f"File: {csv_path.name} | Capacity Q: {Q} | Vehicles (limit): {num_vehicles}")

    for v in range(num_vehicles):
        index = routing.Start(v)
        next_index = solution.Value(routing.NextVar(index))
        if routing.IsEnd(next_index):
            continue  # this truck wasn't used

        vehicles_used += 1
        route_nodes, route_load, route_dist = [], 0, 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route_nodes.append(node)
            if node != depot:
                route_load += int(d["demand"][node])

            prev = index
            index = solution.Value(routing.NextVar(index))
            route_dist += routing.GetArcCostForVehicle(prev, index, v)

        route_nodes.append(manager.IndexToNode(index))  # add depot at end
        total_distance += route_dist
        print(f"\nVehicle {v+1}: {route_nodes} | load {route_load}/{Q} | distance {route_dist}")

    on_time_pct = 100.0  # time windows are hard; feasible => on time
    lb = _lower_bound_vehicles(int(d["demand"][1:].sum()), Q)

    print("\n==== KPIs ====")
    print(f"Vehicles used: {vehicles_used}  (capacity lower bound = {lb})")
    print(f"Total distance: {total_distance}")
    print(f"% on-time deliveries: {on_time_pct}")
    print("All constraints enforced: Capacity, Time windows, Service times, Depot start/end\n")

    from pathlib import Path
    # make sure the folder exists
    Path("reports").mkdir(parents=True, exist_ok=True)
    out_csv = Path("reports") / f"{Path(csv_path).stem}_baseline_routes.csv"
    save_routes_csv(manager, routing, solution, num_vehicles, out_csv)
    print("Saved routes to:", out_csv)




def main():
    ROOT = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(ROOT / "data/solomon_dataset/C1/C108.csv"),
                    help="Path to Solomon CSV")
    ap.add_argument("--capacity", type=int, default=200, help="Vehicle capacity Q")
    ap.add_argument("--vehicles", default="auto",
                    help='"auto" (LB + buffer) or an integer')
    ap.add_argument("--time_limit", type=int, default=20,
                    help="Search time in seconds")
    args = ap.parse_args()

    csv_path = Path(args.data)
    if not csv_path.is_absolute():
        csv_path = (ROOT / csv_path).resolve()
    assert csv_path.exists(), f"Dataset not found: {csv_path}"

    solve_vrptw(csv_path, args.capacity, args.vehicles, args.time_limit)
    

if __name__ == "__main__":
    main()
