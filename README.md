# METS

METS is an open-source heuristic algorithm based on the Memetic framework, designed to solve the **Green Vehicle Routing Problem with Private Capacitated Refueling Stations (GVRP-PCRS)**. The algorithm is implemented in MATLAB and incorporates efficient segmentation and local search techniques to optimize complex vehicle routing problems.

## Project Structure
The project is organized as follows:
- **METS\Instances**: Contains test instances for the algorithm.
- **METS\Load**: Includes programs for reading input data.
- **METS\Two novel segmentation**: Contains two novel segmentation methods used in the solving process.
- **METS\Efficient local search**: Stores the implementation of local search procedures to refine solutions.
- **METS\Crossover select parents**: Includes code for crossover operations and parent selection.
- **METS\Population management**: Contains code for managing the population during the optimization process.
- **METS\test.m**: A small test program to verify the algorithm's functionality.

## Features
The METS algorithm incorporates three novel features to address the unique challenges of the GVRP-PCAFS:

1. **Multiple Tour Segmentation**:
   - Splits a giant tour into route-based solutions using different constraints as splitting criteria.
   - Generates diverse solutions for population initialization and crossover operations, enhancing exploration capabilities.

2. **Efficient Local Search with Novel Moves**:
   - Integrates an AFS conditional rule (CAI rule) and four newly designed move operators to explore large solution neighborhoods.
   - Utilizes a fast move evaluation technique with constant time complexity, significantly reducing computational overhead.

3. **Comprehensive Fitness Evaluation**:
   - Considers multiple factors, including constraint violations, AFS congestion levels, solution costs, and diversity contributions, to guide the search effectively.

### Key Achievements:
- **Performance**: METS sets new best-known solutions in 31 out of 40 benchmark instances, outperforming all state-of-the-art algorithms.
- **Problem Size Scalability**: Demonstrates superior results on small and large instances, including a new set of real-world instances with up to 1,000 customers.
- **Generality**: The novel components of METS have potential applicability to a broader range of evolutionary vehicle routing problems.

## Installation and Usage
1. Ensure MATLAB is installed on your system.
2. Clone or download the repository:
   ```bash
   git clone https://github.com/Fanxing1999/METS.git
3. Add the project folder to MATLAB's path：
   ```bash
   addpath(genpath('METS'));
4. Run the test program to verify the setup
   ```bash
   test;

## Test Instances
The test instances are located in the `METS\Instances` folder. These include:

- **Instances with 15 to 100 customers**: These instances are based on the public instances from the following reference:  
  *Bruglieri, M., Ferone, D., Festa, P. & Pisacane, O. (2022). A GRASP with penalty objective function for the Green Vehicle Routing Problem with Private Capacitated Stations, to appear on Computers and Operations Research.*  
  We provide `.mat` files that include all necessary information from the referenced instances.

- **New larger instances**: We also introduce a new set of larger instances, based on real-world data from JD Logistics, with up to 1,000 customers.

You can adjust the parameters in the `METS\test.m` file to run the algorithm on different instances.


## License
This project is licensed under the MIT License. For more details, see the LICENSE file.
