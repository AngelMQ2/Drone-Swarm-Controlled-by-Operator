# Drone-Swarm-Controlled-by-Operator

This repository contains the implementation of the algorithms developed for the paper "Drone Swarm Controlled by Operator." The project demonstrates controlling a swarm of drones through a leader-follower strategy based on the Consensus Theory, where one drone (the leader) is manually controlled by an operator, and the other drones (followers) adjust their behavior based on the leader's movements. The algorithms enable the swarm to exhibit three main behaviors: aggregation, shape formation, and agent queuing. 

## Repository Structure
* 2D Env: Includes the Python scripts for simulating drone swarm behaviors in a simplified 2D environment.
* 3D Env: Contains the MATLAB Simulink models used for simulating drone swarm behaviors with physical considerations.

## Implemented Behaviors

### Aggregation
In the aggregation behavior, all drones in the swarm converge to a single point in space. This behavior ensures that the drones group together, following the leader drone to a common location.

Key Features:
* Each drone calculates its new position based on the average position of all other drones.
* Collision avoidance is implemented to prevent drones from crashing into each other.

### Shape Formation
In shape formation, the drones form a predefined geometric shape around the leader. The shape and formation parameters are controlled by the operator.

Key Features:
* Drones position themselves at specific offsets relative to the leader, forming shapes where the leader occupies the center and the followers are the figure's vertices.
* The shape's size and orientation can be dynamically adjusted by the pilot.

### Agent Queuing
The queuing behavior arranges the drones in a linear sequence, with each drone following the one in front of it, creating a chain-like formation.

Key Features:
* Each drone follows its immediate predecessor, maintaining a set distance.
* This hierarchical structure allows the swarm to navigate complex paths, closely following the leader.


