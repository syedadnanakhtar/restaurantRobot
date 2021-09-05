# Multi-Robot Planning for Restaurant Environments with Priority-Planning and Active Paths

This project implements an approach to automate order delivery to tables within a restaurant environment. The paths of two unicycle robots inside the restaurant are calculated by solving a path planning problem. The restaurant is considered a static environment and collision avoidance is done by determining the collision free path that satisfies the criteria for the shortest distance. The A*-algorithm has been implemented for path planning. Low level controllers control the robots by implementing motion primitives. The robots  navigate the environment using priority planning based on active paths.

The project was developed in collaboration with Saket Sarawgi and Arjan Vonk at TU Delft, Netherlands. 

## Prerequisites
- MATLAB

The repo only requires installation of MATLAB.

## How to use the code
1. Clone the repo locally on your computer

    `git clone https://github.com/syedadnanakhtar/restaurantRobot`

2. run the `main.m` script.

The code is reasonably well documented to understand. However, some important variables are highlighted below 

- `pointLocation` is a 2-D array, with each row depicting a coordinate sampled in a map, to generate the grid space. 
- `map` is the workspace in cell format
- `chk_pt` referes to checkpoints in the restaurant where algorithm evaluates the future step, as soon as the robot reaches there. 
- `Astar(args)` returns the shortest path where the distance to the goal is the heuristics. 

## Contact
Should you find any bug in the code, or have any questions, please feel free to send me an email at `syed.akhtar[at]tum[dot]de`.
