# CurrentLimit-PowerSystemStability
Companion code to Haley Ross's 2023 Master's Thesis "Inverter Current Limiting Impacts on Power System Stability"
https://scholar.colorado.edu/concern/graduate_thesis_or_dissertations/2227mr36d

The main file to run is CurrentLimitSimulation.jl. This file draws data from the .raw and .dyr files contained in 14BusNew. To run the main simulation be sure to update the pathname to wherever this folder is located for you.

After you have installed the relevant packages, the intention is you will only need to alter the code in the "Setup Simulation" section. You can run full sweeps of all generator and load locations, or set one or a subset of locations to test. You can also set whether you use a grid forming inverter (GFM), a grid following ivnerter (GFL) or both. You can also independently toggle the type of current limitng applied to the GFM and GFL inverter. 

Please note that as of 03/12/2024, the required changes to PowerSimulationsDynamics.jl for this script to work have not left PullRequest status. When that happens I will update the code to reflect the new API. 
