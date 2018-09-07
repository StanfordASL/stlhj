# STL-HJ
Code for “Signal Temporal Logic meets Hamilton-Jacobi Reachability: Connections and Applications” by Mo Chen, Qizhan Tam, Scott C. Livingston, and Marco Pavone:  
http://asl/wp-content/papercite-data/pdf/Chen.Tam.Livingston.Pavone.WAFR18.pdf.

## Installation
1. HJ Reachability Computations (C++, CUDA):
   - The computation uses the beacls toolbox, which can be found here: https://github.com/HJReachability/beacls/tree/master/sources.
   - In the Makefiles, make sure that the installation directories of the dependencies are correct (e.g. NVCC, INSTALL_DIR, etc.).
   - Currently, the STL functions are not compatible with the CUDA option.

2. Numerical Simulations (Matlab):
   - Some functions used are from the helperOC toolbox: https://github.com/HJReachability/helperOC.git
   - Using the helperOC toolbox requires: https://bitbucket.org/ian_mitchell/toolboxls

3. Hardware Experiments (ROS Kinetic, Python, Matlab):
   - Turtlebot 3 burger is used as the hardware platform.
   - ROS files derived from: https://github.com/StanfordASL/asl_turtlebot (as of 5 Sept 2018).
   - For switching controller, Matlab is required.

4. MPC Controller (IPOPT, Casadi-Python, Yalmip-Matlab):
   - An improved alternative to the switching controller typically used in HJ Reachability.
   - Implemented using Yalmip-Matlab and Casadi-Python as parsers, both use IPOPT as the solver.
   - For numerical simulations and visualization, install Yalmip: https://yalmip.github.io/tutorial/installation/.
   - For hardware experiments install Casadi: https://github.com/casadi/casadi/wiki/InstallationInstructions.
   - Note: LFRCS partially implemented for highway overtaking example.

## Folders
1. Turtlebot_Car4D_Car1D_Intersection:
   - Traffic light intersection example as described in the paper linked above.
   - Contains files for reachability computation, numerical simulations, and implementation on Turtlebots.

2. Turtlebot_Car4D_Car1D_Overtake:
   - Highway overtaking example as described in the paper linked above.
   - Contains files for reachability computation, numerical simulations, and implementation on Turtlebots.

3. STL Functions:
   - Contains files for temporal operators (i.e. always, eventually, until).

4. Test Sample:
   - A quick computation to test whether the installation was successful.

5. MPC Controller:
   - Contains Yalmip-Matlab implementation for numerical simulations, Casadi-Python implementation for (ROS) hardware experiments.

6. Old Files:
   - Contains work done leading up to the Traffic light intersection and Highway overtaking examples.
   - Varied approach to the problems posed by the examples.
