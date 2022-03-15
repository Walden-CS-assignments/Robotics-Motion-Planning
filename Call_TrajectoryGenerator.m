%Tse_intial, Tsc_intial, Tsc_final,Tce_grasp, Tce_standoff, k
clc
clear

Tse_intial = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];

%the intial & desired final resting configuration of cube frame
Tsc_intial = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_final = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];

Tce_grasp = [-sqrt(2)/2 0 sqrt(2)/2 0; 0 1 0 0; -sqrt(2)/2 0 -sqrt(2)/2 0; 0 0 0 1];
Tce_standoff = [-sqrt(2)/2 0 sqrt(2)/2 0; 0 1 0 0; -sqrt(2)/2 0 -sqrt(2)/2 0.25; 0 0 0 1];
k = 1;
trajectory = TrajectoryGenerator(Tse_intial, Tsc_intial, Tsc_final,Tce_grasp, Tce_standoff, k);