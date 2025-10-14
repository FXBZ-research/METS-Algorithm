# hexaly model for GVRP-PCAFS
 # Call GrVRP_PCAFS_MILP.py:
 # cmd = [
 # sys.executable, # Python executable
 # "GrVRP_PCAFS_MILP.py", # script file
 # "--mat_file", mat_path, # input .mat instance
 # "--time_limit", str(time_limit) # time limit ]



import os
import math
import hexaly.optimizer
import sys
import time
import scipy.io
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--mat_file", type=str, required=True)
parser.add_argument("--time_limit", type=int, default=300)
args = parser.parse_args()

## load .mat 
data = scipy.io.loadmat(args.mat_file)
vrp_struct = data['vrp'][0, 0]
S = int(vrp_struct['nb_station'][0][0]) if 'nb_station' in vrp_struct.dtype.names else 1
m_vehicles = int(vrp_struct['nb_customer'][0][0])                                                
Q = 32
r = float(vrp_struct['V_fuel_rate'][0][0])                      
T_max = float(vrp_struct['T_max_V'][0][0])                         
p_start = 0           
C_Afs = int(vrp_struct['C_Afs'][0][0])
vehicle_speed = float(vrp_struct['V_speed'][0][0])
service_time = float(vrp_struct['T_Customer'][0][0])

# Retrieve refueling time at an AFS (time required to refuel the vehicle)
refuel_time = float(vrp_struct['T_Afs'][0][0])

## Generate depot, customers, and stations
# Depot with id = 0
depot = {"id": 0, 
         "x": float(vrp_struct['longitude'][0][0]), 
         "y": float(vrp_struct['latitude'][0][0]), 
         "service_time": 0.0}

# Customers with ids 1 to n
customers = []
n_customers = int(vrp_struct['nb_customer'][0][0])
for i in range(n_customers):
    customer = {
        "id": i + 1,
        "x": float(vrp_struct['longitude'][i+1][0]),
        "y": float(vrp_struct['latitude'][i+1][0]),
        "service_time": service_time
    }
    customers.append(customer)

H = C_Afs 
base_station = {
    "id": n_customers + 1,
    "x": float(vrp_struct['longitude'][n_customers + 1][0]),
    "y": float(vrp_struct['latitude'][n_customers + 1][0]),
    "refuel_time": refuel_time,
    "capacity": C_Afs
}
stations = []
for s in range(S):
    stations.append({
        "id": n_customers + 1 + s,    
        "x": base_station["x"],
        "y": base_station["y"],
        "refuel_time": base_station["refuel_time"],
        "capacity": H           
    })


# Combine all into one list
depot_customers_stations = [depot] + customers + [stations]



coords = {}
coords["D0"] = (depot["x"], depot["y"])
for cust in customers:
    coords[f"C{cust['id']}"] = (cust["x"], cust["y"])
for st in stations: 
    coords[f"S{st['id']}"] = (st["x"], st["y"])
D = {}  # D[node_i][node_j] = distance
T = {}  # T[node_i][node_j] = time
all_nodes = list(coords.keys())
for i in all_nodes:
    xi, yi = coords[i]
    D[i] = {}
    T[i] = {}
    for j in all_nodes:
        xj, yj = coords[j]
        dist = math.hypot(xj - xi, yj - yi)
        D[i][j] = dist
        T[i][j] = dist/vehicle_speed 
        


clones = {}       # clones[station_index][pump_index] = list of clone node names
clone_nodes = []  
for s_idx, st in enumerate(stations):
    station_id = st["id"]
    pump_count = st["capacity"]           
    clones[s_idx] = {}
    for pump in range(1, pump_count + 1):
        clones[s_idx][pump] = []
        for k in range(1, m_vehicles + 1):
            clone_name = f"F{station_id}_{pump}_{k}"  # named
            clones[s_idx][pump].append(clone_name)
            clone_nodes.append(clone_name)
            coords[clone_name] = coords[f"S{station_id}"]  
            D[clone_name] = {}
            T[clone_name] = {}
            for j in D[f"S{station_id}"]:
                D[clone_name][j] = D[f"S{station_id}"][j]
                T[clone_name][j] = T[f"S{station_id}"][j]
            for i in D:
                if f"S{station_id}" in D[i]:
                    D[i][clone_name] = D[i][f"S{station_id}"]
                    T[i][clone_name] = T[i][f"S{station_id}"]

def parse_clone(clone_name: str):
    sid = int(clone_name.split("_")[0][1:])
    pump = int(clone_name.split("_")[1])
    kidx = int(clone_name.split("_")[2])
    return sid, pump, kidx

# node_list
node_list = ["D0"] \
            + [f"C{c['id']}" for c in customers] \
            + clone_nodes


with hexaly.optimizer.HexalyOptimizer() as optimizer:
    m = optimizer.model  
    # Decision variables
    # x[i,j]: binary, 1 if a vehicle travels from i to j
    # tau[i]: arrival time at node i
    # y[i]: fuel level at node i (full if depot/AFS clone)

    # Create x variables (only for feasible arcs)
    x = {}
    for i in node_list:
        for j in node_list:
            if i == j:
                continue  # no self-loops
            if i in clone_nodes and j in clone_nodes:
                if i.split("_")[0] == j.split("_")[0]:
                    continue     # no arcs between clones of same station
            if i == "D0" and j == "D0":
                continue    # skip depot-depot
            if D[i][j] * r> Q:
                continue   # arc not feasible due to fuel
            x[(i, j)] = m.bool()  

    # tau and y variables
    tau = {}
    y = {}
    tau["D0"] = m.float(0, T_max)
    y["D0"] = m.float(0, Q)
    for cust in customers:
        node = f"C{cust['id']}"
        tau[node] = m.float(0, T_max)
        y[node] = m.float(0, Q)
    for clone in clone_nodes:
        tau[clone] = m.float(0, T_max)
        y[clone] = m.float(0, Q)

    # (2) Each customer visited exactly once
    for cust in customers:
        node = f"C{cust['id']}"
        m.constraint(sum(x[(node, j)] for j in node_list if (node, j) in x) == 1)
    for cust in customers:
        node = f"C{cust['id']}"
        m.constraint(sum(x[(i, node)] for i in node_list if (i, node) in x) == 1)

    # (3) Each AFS clone visited at most once
    for clone in clone_nodes:
        m.constraint(sum(x[(clone, j)] for j in node_list if (clone, j) in x) <= 1)
        m.constraint(sum(x[(i, clone)] for i in node_list if (i, clone) in x) <= 1) 

    # (4) Flow conservation (in = out for all nodes)
    for j in node_list:
        m.constraint(sum(x[(i, j)] for i in node_list if (i, j) in x) ==
                     sum(x[(j, k)] for k in node_list if (j, k) in x))

    # (5) Number of routes ≤ fleet size
    m.constraint(sum(x[("D0", j)] for j in node_list if ("D0", j) in x) <= m_vehicles)

    # (6) Time continuity with big-M
    for (i, j), x_var in x.items():
        if j == "D0":
            continue  
        if i == "D0":
            p_i = 0.0 
        elif i.startswith("C"):
            cid = int(i[1:])
            p_i = next(c["service_time"] for c in customers if c["id"] == cid)
        elif i in clone_nodes:
            st_id = int(i.split("_")[0][1:]) 
            p_i = next(s["refuel_time"] for s in stations if s["id"] == st_id)
        else:
            p_i = 0.0
        t_ij = T[i][j] 
        m.constraint(tau[j] >= tau[i] + (p_i + t_ij) * x_var - 9999 * (1 - x_var))

    # (7) Departure not earlier than refuel completion
    m.constraint(tau["D0"] >= p_start)

    
    # (8) Route duration ≤ Tmax when returning depot
    for (i, j), x_var in x.items():
        if j == "D0":
            if i.startswith("C"):
                cid = int(i[1:])
                p_i = next(c["service_time"] for c in customers if c["id"] == cid)
            elif i in clone_nodes:
                st_id = int(i.split("_")[0][1:])
                p_i = next(s["refuel_time"] for s in stations if s["id"] == st_id)
            else:
                p_i = 0.0
            t_i0 = T[i]["D0"]
            m.constraint(tau[i] <= T_max - (t_i0 + p_i))
            
    # (9) Fuel continuity at customers
    for (i, j), x_var in x.items():
        if j.startswith("C"):
            m.constraint(y[j] <= y[i] - r * D[i][j] * x_var + Q * (1 - x_var))
            
    # (10) Fuel reset to Q at depot and AFS clones
    m.constraint(y["D0"] == Q)
    for clone in clone_nodes:
        m.constraint(y[clone] == Q)

    # (11) Fuel feasibility before reaching depot/AFS
    for (i, j), x_var in x.items():
        if j == "D0" or j in clone_nodes:
            m.constraint(y[i] >= r * D[i][j] * x_var)
            
    # (12) Pump sequencing constraint
    for s_idx, st in enumerate(stations):
       p_s = st["refuel_time"]  
       for pump, clone_list in clones[s_idx].items():
           for idx in range(len(clone_list) - 1):
               curr_clone = clone_list[idx]
               next_clone = clone_list[idx + 1]
               use_next = sum(x[(i, next_clone)] for i in node_list if (i, next_clone) in x)
               m.constraint(tau[next_clone] >= tau[curr_clone] + p_s - (T_max + p_s) * (1 - use_next))

    # (13) Clone order constraint within same pump
    for s_idx, st in enumerate(stations):
        for pump, clone_list in clones[s_idx].items():
            for idx in range(len(clone_list) - 1):
                c1 = clone_list[idx]
                c2 = clone_list[idx + 1]
                in_c1 = sum(x[(i, c1)] for i in node_list if (i, c1) in x)
                in_c2 = sum(x[(i, c2)] for i in node_list if (i, c2) in x)
                m.constraint(in_c1 >= in_c2)

    # (14) Pump usage order within same station
    for s_idx, st in enumerate(stations):
        pump_list = sorted(clones[s_idx].keys())
        for pi in range(len(pump_list) - 1):
            pump_h = pump_list[pi]
            pump_h1 = pump_list[pi + 1]
            use_h = sum(x[(i, clone)] for clone in clones[s_idx][pump_h] for i in node_list if (i, clone) in x)
            use_h1 = sum(x[(i, clone)] for clone in clones[s_idx][pump_h1] for i in node_list if (i, clone) in x)
            m.constraint(use_h >= use_h1)

    
    # Objective: minimize total distance
    total_distance = 0
    for (i, j), x_var in x.items():
        total_distance += D[i][j] * x_var
    m.minimize(total_distance)
    m.close() 
    optimizer.param.time_limit = args.time_limit

    # Solve model
    start_time = time.time()
    optimizer.solve()
    end_time = time.time()

    # Results
    solve_time = end_time - start_time
    total_dist_value = total_distance.value

    # Rebuild routes
    routes = []
    used_vehicles = 0
    for (i, j), x_var in x.items():
        if i == "D0" and abs(x_var.value - 1) < 1e-6:
            used_vehicles += 1
            route = ["Depot"]
            curr_node = j
            while curr_node != "D0":
                if curr_node.startswith("C"):
                    cid = curr_node[1:]
                    route.append(f"Customer{cid}")
                elif curr_node.startswith("F"):
                    route.append(curr_node)
                # find successor of curr_node
                next_node = None
                for (u, v), x_var2 in x.items():
                    if u == curr_node and abs(x_var2.value - 1) < 1e-6:
                        next_node = v
                        break
                if next_node is None:
                    next_node = "D0"
                curr_node = next_node
            route.append("Depot")
            routes.append(route)

    # Print results
    print(f"Total distance: {total_dist_value:.2f}")
    print(f"Vehicles used: {used_vehicles}")
    print(f"Solve time: {solve_time:.2f} sec")
    for idx, route in enumerate(routes, start=1):
        print(f"Vehicle {idx} route: {' -> '.join(route)}")

    print("\n=== Arrival times at used AFS clones ===")
    for clone in clone_nodes:
        in_flow = sum(x[(i, clone)].value for i in node_list if (i, clone) in x)
        if in_flow > 0.5:
            print(f"{clone}: {tau[clone].value:.2f}")

