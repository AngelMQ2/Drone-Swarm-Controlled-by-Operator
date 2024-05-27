% This script define a set of physical parameters for the quadrator drone
% in simulink.

% We are assuming a 1 kilo mass per each motor-
m = 1;
g = 9.8
% Inertial Moment matrix:
I = [2 0 0; 0 2 0; 0 0 4];
Ic = [2 0 0; 0 2 0; 0 0 4];

% Inertial Matrix:
M_RB = [eye(3), zeros(3); zeros(3), I];


%%%%%%%%%%%%%%%% SWARM PARAMS %%%%%%%%%%%%%
STEP_SIZE = 1;
SAVE_SPACE = 5;