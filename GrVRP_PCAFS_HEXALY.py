# hexaly model for GVRP-PCAFS
 # Call GrVRP_PCAFS_HEXALY.py:
 # cmd = [
 # sys.executable, # Python executable
 # "GrVRP_PCAFS_HEXALY.py", # script file
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

from scipy.io import loadmat

parser = argparse.ArgumentParser()
parser.add_argument("--mat_file", required=True, help="path to .mat instance")
parser.add_argument("--time_limit", type=int, default=3600, help="solver time limit (sec)")
args = parser.parse_args()

data = scipy.io.loadmat(args.mat_file)
vrp_struct = data['vrp'][0, 0]
S = int(vrp_struct['nb_station'][0][0]) if 'nb_station' in vrp_struct.dtype.names else 1
m_vehicles  = int(vrp_struct['nb_customer'][0][0])         
Q           = 32                                           
r           = float(vrp_struct['V_fuel_rate'][0][0])       
T_max       = float(vrp_struct['T_max_V'][0][0])          
p_start     = 0                                           
C_Afs       = int(vrp_struct['C_Afs'][0][0])               
vehicle_speed = float(vrp_struct['V_speed'][0][0])       
service_time  = float(vrp_struct['T_Customer'][0][0])     
refuel_time   = float(vrp_struct['T_Afs'][0][0])      

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
    customers.append({
        "id": i + 1,
        "x": float(vrp_struct['longitude'][i+1][0]),
        "y": float(vrp_struct['latitude'][i+1][0]),
        "service_time": service_time
    })

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

coords = {}
coords["D0"] = (depot["x"], depot["y"])
for cust in customers:
    coords[f"C{cust['id']}"] = (cust["x"], cust["y"])
for st in stations:
    coords[f"S{st['id']}"] = (st["x"], st["y"])

D_name = {}
T_name = {}
MAX_COPIES_PER_STATION = m_vehicles
TOTAL_CLONES = S * H * MAX_COPIES_PER_STATION  

N = n_customers + TOTAL_CLONES + 1
clone_station = [None] * (N-1)
clone_pump    = [None] * (N-1)
pump_groups   = {}   # (s_idx, h_idx) -> [clone_j, clone_j, ...]
clone_base_coords = {} 

j_glob = n_customers
for s_idx, st in enumerate(stations):
    for h_idx in range(H):
        pump_groups[(s_idx, h_idx)] = []
        for c_idx in range(MAX_COPIES_PER_STATION):
            if j_glob >= (N-1):
                break
            clone_station[j_glob] = s_idx
            clone_pump[j_glob]    = h_idx
            pump_groups[(s_idx, h_idx)].append(j_glob)

            clone_base_coords[j_glob] = (st["x"], st["y"])
            j_glob += 1

dis_table_all = [[None for _ in range(N)] for _ in range(N)]
time_table_all = [[None for _ in range(N)] for _ in range(N)]

dis_table = [[None for _ in range(N-1)] for _ in range(N)]
time_table = [[None for _ in range(N-1)] for _ in range(N)]

dis_table_depot = [None]*(N-1)
time_table_depot = [None]*(N-1)

p_time = [None]*(N-1)
clone_station = [-1] * (N - 1)
clone_pump    = [-1] * (N - 1)
all_base_nodes = list(coords.keys())
for i in range(N):
    if i >= len(all_base_nodes):
        xi, yi = coords[all_base_nodes[-1]]
        p_time[i-1] = refuel_time
    else:
        if i > 0 :
            p_time[i-1] = service_time
        xi, yi = coords[all_base_nodes[i]]
    D_name[i] = {}
    T_name[i] = {}
    for j in range(N):
        if j >= len(all_base_nodes):
            xj, yj = coords[all_base_nodes[-1]]
        else:
            xj, yj = coords[all_base_nodes[j]]
        dist = math.hypot(xj - xi, yj - yi)
        D_name[i][j] = dist
        T_name[i][j] = dist / vehicle_speed
        dis_table_all[i][j] = dist
        time_table_all[i][j] = dist / vehicle_speed

dis_table = [row[1:] for row in dis_table_all[1:]]
time_table = [row[1:] for row in time_table_all[1:]]

dis_table_depot = [row[0] for row in dis_table_all[1:]]
time_table_depot = [row[0] for row in time_table_all[1:]]

with hexaly.optimizer.HexalyOptimizer() as optimizer:
    model = optimizer.model

    # Sequence of customers visited by each truck
    customers_sequences = [model.list(n_customers) for _ in range(m_vehicles)]

    # A customer might be visited by only one truck
    model.constraint(model.partition(customers_sequences))

    # Sequence of customers and afs visited by each truck
    routes = [model.list(N-1) for _ in range(m_vehicles)]

    for k in range(m_vehicles):
        for c in range(n_customers):
            model.constraint(model.contains(routes[k], c) == model.contains(customers_sequences[k], c))
    
    # A customer or a AFS might be visited by only one truck
    model.constraint(model.disjoint(routes))

    # Create Hexaly arrays to be able to access them with an "at" operator
    pause_time = model.array(p_time)

    dist_matrix = model.array(dis_table)
    time_matrix = model.array(time_table)

    dist_depot = model.array(dis_table_depot)
    time_depot = model.array(time_table_depot)

    dist_routes = [None] * m_vehicles
    time_routes = [None] * m_vehicles
    L_de_routes = [None] * m_vehicles
    l_arr_routes = [None] * m_vehicles
    time_dep_routes = [None] * m_vehicles
    time_arr_routes = [None] * m_vehicles

    tau = [model.float(0, T_max) for _ in range(N-1)]
    tau_arr = model.array(tau)

    for k in range(m_vehicles):
            sequence = routes[k]
            c = model.count(sequence)

            # Distance traveled by each truck
            dist_lambda = model.lambda_function(lambda i:
                                                model.at(dist_matrix,
                                                         sequence[i - 1],
                                                         sequence[i]))
            dist_routes[k] = model.sum(model.range(1, c), dist_lambda) \
                + model.iif(c > 0,
                            dist_depot[sequence[0]] + dist_depot[sequence[c - 1]],
                            0)
            
            L_de_lambda = model.lambda_function(   lambda i, prevL_de: 
                    model.iif(
                        model.at(sequence, i)>=n_customers,
                        Q,
                        model.iif(
                            i==0,
                            Q - r * dist_depot[sequence[0]],
                            prevL_de - r * model.at(dist_matrix,sequence[i - 1],sequence[i])),))
            
            L_de_routes[k] = model.array(model.range(0, c), L_de_lambda, Q)
            l_arr_lambda = model.lambda_function(
                lambda i, prevl_arr:
                    model.iif(
                        model.at(sequence, i)>=n_customers,
                            model.iif(
                                i==0,
                                Q - r * dist_depot[sequence[0]],
                                prevl_arr - r * model.at(dist_matrix,sequence[i - 1],sequence[i])),
                        L_de_routes[k][i]))

            l_arr_routes[k] = model.array(model.range(0, c), l_arr_lambda, L_de_routes[k][0])


            # 1) l_i ≥ 0
            ge0_lambda = model.lambda_function(lambda i: model.at(l_arr_routes[k], i) >= 0)
            model.constraint(model.and_(model.range(0, c), ge0_lambda))

            # 2) l_i ≤ L_i
            leL_lambda = model.lambda_function(lambda i: model.at(l_arr_routes[k], i) <= model.at(L_de_routes[k], i))
            model.constraint(model.and_(model.range(0, c), leL_lambda))

            # 3) L_i ≤ Q
            leQ_lambda = model.lambda_function(lambda i: model.at(L_de_routes[k], i) <= Q)
            model.constraint(model.and_(model.range(0, c), leQ_lambda))

            retdep_lambda = model.lambda_function(
                lambda i: model.iif(
                    i == c - 1,
                    model.at(L_de_routes[k], i) >= r * model.at(dist_depot, sequence[i]),
                    True
                )
            )
            model.constraint(model.and_(model.range(0, c), retdep_lambda))

            # i==0: depot -> sequence[0]
            # i>0 : dep[i-1] + travel(sequence[i-1], sequence[i])
            time_arr_lambda = model.lambda_function(
                lambda i, prev_time_arr:
                    model.iif(
                        i == 0,
                        time_depot[sequence[0]],
                        prev_time_arr + pause_time[sequence[i-1]] + model.at(time_matrix, sequence[i - 1], sequence[i])
                    )
            )

            # i==0: time_depot[sequence[0]] + service_or_refuel(sequence[0])
            # i>0 : dep[i-1] + travel + service_or_refuel(sequence[i])
            time_dep_lambda = model.lambda_function(
                lambda i, prev_time_dep:
                    model.iif(
                        i == 0,
                        time_depot[sequence[0]],
                        prev_time_dep + model.at(time_matrix, sequence[i - 1], sequence[i])
                    ) + pause_time[sequence[i]]
            )

            time_dep_routes[k] = model.array(model.range(0, c), time_dep_lambda, 0)
            time_arr_routes[k] = model.array(model.range(0, c), time_arr_lambda, 0)


            if k == 0:
                arr_pick = [ [None]*(N-1) for _ in range(m_vehicles) ]  # 仅初始化一次

            for j in range(n_customers, N-1):
                pick_lambda = model.lambda_function(
                    lambda i: model.iif(
                        model.at(sequence, i) == j,
                        model.at(time_arr_routes[k], i),
                        0.0
                    )
                )
                arr_pick[k][j] = model.sum(model.range(0, c), pick_lambda)

            # LB， τ[sequence[i]] ≥ time_arr_routes[k][i]
            tau_lb_lambda = model.lambda_function(
                lambda i: model.iif(
                    model.at(sequence, i) >= n_customers,
                    model.at(tau_arr, model.at(sequence, i)) >= model.at(time_arr_routes[k], i),
                    True
                )
            )
            model.constraint(model.and_(model.range(0, c), tau_lb_lambda))
    
    use = [model.sum([model.contains(routes[k], j) for k in range(m_vehicles)]) for j in range(N-1)]
    use_arr = model.array(use)

    
    arr_node = [model.float(0, T_max) for _ in range(N-1)]
    arr_arr  = model.array(arr_node)

    for j in range(n_customers, N-1):
        terms = [arr_pick[k][j] for k in range(m_vehicles)]
        S_j = model.sum(terms) if len(terms) > 1 else terms[0]
        model.constraint(model.at(arr_arr, j) == S_j)

    
    bigM = T_max
    for j in range(n_customers, N-1):
        model.constraint(
            model.at(tau_arr, j) >= model.at(arr_arr, j) - bigM * (1 - model.at(use_arr, j))
        )

    clone_station_arr = model.array(clone_station) 
    clone_pump_arr    = model.array(clone_pump)   
     
    pt = refuel_time
    for j in range(n_customers, N-1):               
        for k in range(m_vehicles):
            sequence = routes[k]
            c = model.count(sequence)
            tau_sequence_lambda = model.lambda_function(
                lambda i:
                    model.iif(
                        model.and_(
                            model.at(sequence, i) >= n_customers,     
                            model.at(sequence, i) != j,                
                            model.at(use_arr, j) >= 1,                 
                            model.at(use_arr, model.at(sequence, i)) >= 1,  
                            model.at(clone_station_arr, model.at(sequence, i)) 
                                == model.at(clone_station_arr, j),
                            model.at(clone_pump_arr,    model.at(sequence, i)) 
                                == model.at(clone_pump_arr,    j)
                        ),
                        model.iif(
                            model.at(time_arr_routes[k], i) >= model.at(arr_arr, j),
                            # τ_i ≥ τ_j + pt
                            model.at(tau_arr, model.at(sequence, i)) >= model.at(tau_arr, j) + pt,
                            # τ_j ≥ τ_i + pt
                            model.at(tau_arr, j) >= model.at(tau_arr, model.at(sequence, i)) + pt
                        ),
                        True
                    )
            )
            model.constraint(model.and_(model.range(0, c), tau_sequence_lambda))


    route_total_time = [None] * m_vehicles
    base_time_routes = [None] * m_vehicles
    total_wait_routes = [None] * m_vehicles
    back_time_routes  = [None] * m_vehicles

    for k in range(m_vehicles):
        seq = routes[k]
        c   = model.count(seq)
        wait_lambda = model.lambda_function(
            lambda i: model.iif(
                model.at(seq, i) >= n_customers,
                model.at(tau_arr, model.at(seq, i)) - model.at(time_arr_routes[k], i),
                0.0
            )
        )
        total_wait_routes[k] = model.sum(model.range(0, c), wait_lambda)
        base_time_routes[k] = model.iif(c > 0, model.at(time_dep_routes[k], c - 1), 0.0)
        last_node = model.iif(c > 0, model.at(seq, c - 1), 0)
        back_time_routes[k] = model.iif(c > 0, model.at(time_depot,last_node), 0.0)
        route_total_time[k] = base_time_routes[k] + total_wait_routes[k] + back_time_routes[k]
        model.constraint(route_total_time[k] <= T_max)





    total_distance = model.sum([dist_routes[k] for k in range(m_vehicles)])
    model.minimize(total_distance)

    model.close()
    optimizer.param.time_limit = args.time_limit
    optimizer.solve()


    print("=== Hexaly 求解完成 ===")
    print("Solution status:", optimizer.solution.status)  # INCONSISTENT/INFEASIBLE/FEASIBLE/OPTIMAL 
    print("Objective (total distance):", total_distance.value)

    for k in range(m_vehicles):
        route_vals = routes[k].value        
        dist_val   = dist_routes[k].value      
        time_val   = route_total_time[k].value  
        base_val   = base_time_routes[k].value
        wait_val   = total_wait_routes[k].value
        back_val   = back_time_routes[k].value
        total_val  = route_total_time[k].value
        dep_times  = list(time_dep_routes[k].value) 

        print(f"Route {k}: {list(route_vals)}")
        print(f"   Distance = {dist_val:.2f},   Total time = {time_val:.2f}")
        print(f"   Departure times = {dep_times}")
        print(f"   Base time    = {base_val:.2f}")
        print(f"   Wait time    = {wait_val:.2f}")
        print(f"   Return time  = {back_val:.2f}")
        print(f"   Total time   = {total_val:.2f}")

    afs_idx = list(range(n_customers, N-1))
    print("Tau (AFS only):", [(j, tau[j].value) for j in afs_idx])

    max_show = min(N-1, 10000)
    print("Tau[0..{}):".format(max_show), [tau[j].value for j in range(max_show)])
