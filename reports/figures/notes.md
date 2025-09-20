# Milestone 1 — Understanding the Problem & Data

## Problem statement (VRPTW)
Given delivery orders with locations, demand, service time and time windows, plan routes that:
- minimize total travel distance (primary objective),
- satisfy **vehicle capacity**, **time windows**, and **service times**,
- start and end at the **depot**,
- and use as few vehicles as possible (secondary objective).

## Dataset used (Solomon benchmark)
- File: `data/solomon_dataset/C1/C108.csv` (e.g., `C108.csv`)
- Format: row 0 = **depot**, rows 1..N = **customers**  
  Columns:
  - `XCOORD`, `YCOORD` → coordinates for distance
  - `DEMAND` → quantity delivered at that stop
  - `READY TIME`, `DUE DATE` → allowed start-of-service window
  - `SERVICE TIME` → minutes spent at stop

**Depot details**: demand=0, service=0, time window typically `[0, 1236]` (latest return).

## Key assumptions (documented for baseline)
1. Vehicle **capacity Q** for this instance family = **200** (Solomon C1 standard).
2. Travel time = Euclidean distance (benchmark convention).
3. If a vehicle arrives before `READY TIME`, it waits; it must start service by `DUE DATE`.
4. All routes must return to the depot before the depot `DUE DATE`.
5. Number of vehicles is not hard-limited; we penalize each used vehicle to prefer fewer.
6. Single trip per vehicle: each truck leaves the depot once and returns once (no mid-day reloads / multi-trip).

## Constraints (what “feasible” means)
- **Capacity**: for each vehicle, `Σ DEMAND on its route ≤ Q`.
- **Time windows**: for each stop `i`, `start_service_i ∈ [READY_i, DUE_i]`.
- **Service time**: after starting service, you occupy `SERVICE_i` minutes before leaving.
- **Depot**: routes start at depot (node 0) and end at depot.

## KPIs we will report
- **Total distance** (objective)
- **Vehicles used** (secondary objective; minimized via fixed cost)
- **% on-time deliveries** (windows are hard → 100% if solver returns a solution)
- (Optional later) route length distribution, average load utilization

## Sanity checks on the dataset
- Depot is row 0 and has demand=0, service=0.
- All columns exist: XCOORD, YCOORD, DEMAND, READY TIME, DUE DATE, SERVICE TIME.
- DEMAND are non-negative; time windows make sense (`READY ≤ DUE`).
- Total demand ≈ `<FILL AFTER STEP 4>`; lower bound on vehicles = `ceil(total_demand / Q)`.

## Evidence (attach once generated)
- Location scatter + demand histogram (two images).
- OR-Tools baseline outputs (routes, KPIs) — will be added in Milestone 2.

# Problem Statement — VRPTW (Vehicle Routing Problem with Time Windows)

Plan delivery routes for a fleet of identical trucks that **start and end at the depot** (the warehouse).  
Each customer has a location, a **demand**, a **service time**, and a **time window** during which service may begin.

**Goal:** **Minimize total travel distance** while:
- respecting **vehicle capacity**,
- serving each customer **within its time window**,
- including the required **service time** at each stop, and
- returning every vehicle to the depot before the depot’s cutoff time.

We also prefer to use **as few vehicles as possible** (secondary objective).

> **Dataset note (Solomon):** The first data row is the **depot** (DEMAND=0, SERVICE TIME=0, wide time window like 0–1236).  
> For C1 instances (e.g., `C108.csv`) the standard vehicle capacity is **Q = 200**.

---

## Assumptions (Baseline)

- **Single trip per vehicle**: each truck leaves the depot once and returns once (no mid-day reloads / multi-trip).
- **Identical vehicles** with the same capacity **Q** (e.g., Q=200 for Solomon C1).
- **Travel time = Euclidean distance** between coordinates (benchmark convention).
- **Hard time windows**: starting service after the due time is not allowed.
- **Early arrival → waiting** until the customer’s ready time is allowed.
- **Fewer vehicles preferred** via a fixed cost per used vehicle in the model.

---

## Constraints (Feasibility)

1. **Depot start & end**  
   Every route starts at the depot and ends at the depot; must finish before the depot’s due time.

2. **Capacity**  
   For each vehicle/route,  
   \[
   \sum_{\text{customers on route}} \text{DEMAND} \;\le\; Q
   \]  
   Intuition: a truck can’t carry more than Q; excess customers go to another route.

3. **Time windows**  
   For each customer \(i\), the **start of service** must satisfy  
   \[
   \text{READY\_TIME}_i \;\le\; \text{start\_service}_i \;\le\; \text{DUE\_DATE}_i
   \]  
   If the truck arrives early, it waits until READY\_TIME. Late start is infeasible.

4. **Service time**  
   After service begins at customer \(i\), the truck remains for **SERVICE\_TIME\(_i\)** minutes before departing.

5. **Objective (Cost)**  
   - **Primary:** minimize **total distance** traveled across all vehicles.  
   - **Secondary:** minimize **vehicles used** (handled via a fixed cost per used vehicle).

---

## KPIs to Report

- **Total distance** traveled (primary objective).  
- **Vehicles used** (with context from the capacity lower bound below).  
- **% on-time deliveries** (should be 100% under hard time windows if a solution is found).  
- *(Optional later)* Average load utilization, total waiting time, route length distribution.

---

## Capacity Lower Bound (LB) — For Context

The **capacity lower bound (LB)** is the minimum possible number of vehicles based **only on capacity** (since we assume single trip per vehicle):

\[
\textbf{LB} \;=\; \left\lceil \frac{\text{Total customer demand}}{Q} \right\rceil
\]

- This is a **sanity check**: any solution must use **≥ LB** vehicles.  
- Actual solutions may need **more than LB** because of **time windows** and **service times** (even when capacity would allow fewer).


