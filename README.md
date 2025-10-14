# METS: Memetic Search for the Green Vehicle Routing Problem with Private Capacitated Refueling Stations (GrVRP-PCAFS)

**METS** is an open-source **memetic algorithm (MA)** designed to solve the *Green Vehicle Routing Problem with Private Capacitated Alternative Fuel Stations (GrVRP-PCAFS)*.  
It integrates **separate constraint-based tour segmentation**, **diversity–feasibility control**, and **efficient local search with constant-time evaluation mechanisms** to optimize large-scale sustainable routing problems.

---

## 🧩 Overview

The GrVRP-PCAFS extends the classical green vehicle routing problem by considering the **limited capacity** of refueling or charging stations, where only a limited number of vehicles can refuel simultaneously while others must wait.  
This creates additional challenges in managing **route duration limits**, **refueling schedules**, and **station congestion**.

**METS** addresses these challenges by balancing exploration and exploitation through three core components:

1. **Separate Constraint-based Tour Segmentation (SCTS)**  
   Splits a giant tour into multiple feasible routes using a *single constraint* (either route duration or driving range).  
   This separation promotes population diversity and exploration efficiency.

2. **Comprehensive Fitness Evaluation Function**  
   Integrates route cost, constraint violations (overtime, over-mileage, and over-capacity), and diversity contribution.  
   Adaptive penalty parameters dynamically control the trade-off between feasibility and diversity.

3. **Efficient Local Search with Conditional AFS Insertion (CAI)**  
   Introduces CAI-based move operators that automatically decide when to insert refueling stations.  
   Constant-time (\(O(1)\)) move evaluation significantly accelerates local search.

---

## 🚀 Features

- **Exploration:**  
  Diverse population generation via the SCTS strategy.

- **Exploitation:**  
  CAI-based local search with constant-time evaluation for efficient neighborhood exploration.

- **Adaptivity:**  
  Dynamic penalty control for constraint violations to maintain feasible yet diverse populations.

- **Performance Highlights:**  
  - Sets **31 new best-known solutions** across 40 public GrVRP-PCAFS benchmarks.  
  - Outperforms GRASP, CP-Proactive, and the general-purpose Hexaly solver.  
  - Scales to **1,000-customer real-world instances** from JD Logistics.

---

## ⚙️ Installation & Usage

1. **Prerequisites**  
   MATLAB R2022a or later is required.

2. **Clone the repository**
   ```bash
   git clone https://github.com/Fanxing1999/METS.git
   
3. **Add the project folder to MATLAB's path**
   ```bash
   addpath(genpath('METS'));
4. **Run the test program to verify the setup**
   ```bash
   test;

## Test Instances
The test instances are located in the `METS\Instances` folder. These include:

- **Instances with 15 to 100 customers**:
  These instances are based on the public instances from the following reference:  
  *Bruglieri, M., Ferone, D., Festa, P. & Pisacane, O. (2022). A GRASP with penalty objective function for the Green Vehicle Routing Problem with Private Capacitated Stations, to appear on Computers and Operations Research.*  
  We provide `.mat` files that include all necessary information from the referenced instances.

- **New larger instances**:
  A newly introduced large-scale benchmark set based on real-world data from Beijing. 
  It represents realistic urban delivery scenarios, with customer sizes ranging from 200 to 1,000.
  This set enables evaluation of algorithm scalability under real-world operational constraints.

You can adjust the parameters in the `METS\test.m` file to run the algorithm on different instances.

## ⚙️ Solver Implementations (Python)

Two independent Python models are provided for reproducing the mathematical formulations and solver integrations described in the paper:

File	Description
GrVRP_PCAFS_MILP.py — Implements the MILP formulation of the GrVRP-PCAFS for direct use with Hexaly (a general-purpose global optimization solver). This version closely follows the mathematical formulation presented in Appendix D of the manuscript.
GrVRP_PCAFS_HEXALY.py — Implements the Hexaly-preferred formulation, leveraging Hexaly’s native modeling features (sets, lists and arrays). This version follows the solver-recommended modeling practices for routing problems. 

## 🔧 Running the models
Both scripts run independently on .mat instance files.

## Run the Hexaly-preferred formulation
cmd = [sys.executable, "GrVRP_PCAFS_HEXALY.py", "--mat_file", mat_path, "--time_limit", str(time_limit)]

## Run the direct MILP formulation
cmd = [sys.executable, "GrVRP_PCAFS_MILP.py", "--mat_file", mat_path, "--time_limit", str(time_limit)]


## License
This project is licensed under the MIT License. For more details, see the LICENSE file.





