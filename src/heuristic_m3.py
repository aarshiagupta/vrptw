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

# helpers to report true (unweighted) distance, even if cost is weighted by time/emissions
def _arc_distance(i: int, j: int, dist_mat) -> int:
    """Return true distance between nodes i and j from the distance matrix."""
    return int(dist_mat[i][j])

def _route_true_distance(manager, routing, solution, vehicle_id: int, dist_mat) -> int:
    """Sum true distances along the actual route of vehicle_id."""
    idx = routing.Start(vehicle_id)
    total = 0
    while not routing.IsEnd(idx):
        i = manager.IndexToNode(idx)
        nxt = solution.Value(routing.NextVar(idx))
        j = manager.IndexToNode(nxt)
        total += _arc_distance(i, j, dist_mat)
        idx = nxt
    return total

def append_experiment_log(path_csv, row: dict):
    """Append a row of KPIs/configs to a CSV (create with header if missing)."""
    import pandas as pd
    path_csv.parent.mkdir(parents=True, exist_ok=True)
    df_row = pd.DataFrame([row])
    if not path_csv.exists():
        df_row.to_csv(path_csv, index=False)
    else:
        df_row.to_csv(path_csv, mode="a", header=False, index=False)




# Solve VRPTW using OR-Tools
def solve_vrptw(csv_path: Path, Q: int, vehicles_arg, time_limit: int, args):
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

    def cost_cb(from_index, to_index):
        i, j = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        # travel distance scaled by traffic
        dist = int(round(args.traffic * d["dist"][i][j]))
        # time proxy = service at FROM + travel
        travel_time = dist + d["service"][i]
        # emissions proxy
        co2 = args.co2_ef * dist
        cost = args.w_dist*dist + args.w_time*travel_time + args.w_co2*co2
        return int(round(cost))

    cost_idx = routing.RegisterTransitCallback(cost_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(cost_idx)


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


    # time callback with traffic multiplier
    def time_cb(from_index, to_index): 
        i, j = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        travel = int(round(args.traffic * d["dist"][i][j]))
        return int(travel + d["service"][i])  # depot service is 0 so it’s fine
    time_idx = routing.RegisterTransitCallback(time_cb)

    # Give a safe horizon and plenty of waiting (slack) so trucks can wait if early
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
    sp.first_solution_strategy = getattr(routing_enums_pb2.FirstSolutionStrategy, args.first)
    sp.local_search_metaheuristic = getattr(routing_enums_pb2.LocalSearchMetaheuristic, args.meta)
    sp.time_limit.FromSeconds(int(time_limit))
    # sp.log_search = True  # optional


    # solve ; set time limit
    solution = routing.SolveWithParameters(sp)
    if not solution:
        raise RuntimeError("No solution found (try raising --time_limit or vehicles).")


    total_distance = 0
    vehicles_used = 0
    total_true_distance = 0  # sum of real distances for all used vehicles


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
        route_true_dist = _route_true_distance(manager, routing, solution, v, d["dist"])

        total_distance += route_dist 
        total_true_distance += route_true_dist # sum actual distances

        print(f"\nVehicle {v+1}: {route_nodes} | load {route_load}/{Q} | "
      f"true_distance {route_true_dist} | weighted_cost {route_dist}")

    on_time_pct = 100.0  # time windows are hard; feasible => on time
    lb = _lower_bound_vehicles(int(d["demand"][1:].sum()), Q)

    print("\n==== KPIs ====")
    print(f"Vehicles used: {vehicles_used}  (capacity lower bound = {lb})")
    print(f"Total distance (true): {total_true_distance}")
    print(f"Total cost (weighted): {total_distance}")  # optional; only if you kept it
    print(f"% on-time deliveries: {on_time_pct}")
    print("All constraints enforced: Capacity, Time windows, Service times, Depot start/end\n")



    from pathlib import Path

    reports_dir = Path("reports") / "experiments"
    reports_dir.mkdir(parents=True, exist_ok=True)

    stem = Path(csv_path).stem  # e.g., C108
    tag = f"{stem}_{args.first}-{args.meta}_wd{args.w_dist}_wt{args.w_time}_t{args.traffic}_tl{time_limit}_veh{num_vehicles}"
    # make the tag filesystem-friendly
    tag = tag.replace(" ", "").replace("/", "-")

    routes_csv = reports_dir / f"{tag}_routes.csv"
    save_routes_csv(manager, routing, solution, num_vehicles, routes_csv)
    print("Saved routes to:", routes_csv)

    # log KPIs for comparison 
    log_csv = Path("reports") / "experiments" / "experiments_log.csv"
    append_experiment_log(
        log_csv,
    {
        "dataset": Path(csv_path).stem,
        "first": args.first,
        "meta": args.meta,
        "weights_dist": args.w_dist,
        "weights_time": args.w_time,
        "weights_co2": args.w_co2,
        "traffic": args.traffic,
        "time_limit_s": time_limit,
        "vehicles_cap": num_vehicles,
        "vehicles_used": vehicles_used,
        "total_distance_true": total_true_distance,
        "total_cost_weighted": total_distance,  # optional
        "on_time_pct": on_time_pct,
        }
    )
    print("Logged KPIs to:", log_csv)



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
    ap.add_argument("--first", default="PATH_CHEAPEST_ARC",
                help="PATH_CHEAPEST_ARC | SAVINGS | PARALLEL_CHEAPEST_INSERTION")
    ap.add_argument("--meta", default="GUIDED_LOCAL_SEARCH",
                    help="GUIDED_LOCAL_SEARCH | TABU_SEARCH | SIMULATED_ANNEALING | NONE")
    ap.add_argument("--seed", type=int, default=0, help="Random seed (0=off)")

    # weights & simple scenario knobs
    ap.add_argument("--w_dist", type=float, default=1.0, help="Weight for distance")
    ap.add_argument("--w_time", type=float, default=0.0, help="Weight for time (service+travel)")
    ap.add_argument("--w_co2",  type=float, default=0.0, help="Weight for emissions proxy")
    ap.add_argument("--co2_ef", type=float, default=1.0, help="CO2 per distance unit (proxy)")
    ap.add_argument("--traffic", type=float, default=1.0, help="Multiply travel by this (e.g., 1.25)")

    args = ap.parse_args()

    csv_path = Path(args.data)
    if not csv_path.is_absolute():
        csv_path = (ROOT / csv_path).resolve()
    assert csv_path.exists(), f"Dataset not found: {csv_path}"

    solve_vrptw(csv_path, args.capacity, args.vehicles, args.time_limit, args)

    

if __name__ == "__main__":
    main()
