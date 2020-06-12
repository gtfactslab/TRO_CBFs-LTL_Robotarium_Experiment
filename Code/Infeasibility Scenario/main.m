% Comparison between traditional finite time control barrier functions
% and Composite finite time control barrier functions
%
% Authors: Mohit Srinivasan, Samuel Coogan
% Date:    06/11/2020
%
% Description: This file simulates a simple example with two agents tasked
%              with reaching separate goal regions, while satisfying a
%              connectivity constraint. Two approaches can be used to model
%              this task. "ReachAB_comp.m" is the file that runs the
%              composite finite time control barrier function (Theorem 1)
%              in order to satisfy the task specification, whereas
%              "ReachAB_trad.m" runs the traditional finite time control
%              barrier functions which results in an infeasible solution
%

clear all;
clc;
close all;

%% Initialization of system parameters
X = [];
u = [];
u0 = [0; 0; 0; 0];
X0 = [-0.8; 0.5; -0.8; 0.4];
u = [u, u0];
X = [X, X0];
dt = 0.01;
hgAx = -1;
hgBx = -1;
PA = [1/(0.4)^2 0; 0 1/(0.2)^2];
PB = [1/(0.4)^2 0; 0 1/(0.2)^2];
cx_A = 0.9; cy_A = 0.7; cx_B = 0.6; cy_B = -0.6;
CA = [cx_A; cy_A];
CB = [cx_B; cy_B];
i = 1;
HAx = [];
HBx = [];
F = [];

%% Algorithm
while( hgAx <= 0 || hgBx <= 0)
    
    [hgAx, hgBx, flag, dx] = ReachAB_comp(X(:,i));
    F = [F, flag];

    if flag < 0
        break;
    end
    
    X(:,i+1) = X(:,i) + dt*dx;
    i = i+1;
end

%% Plot Trajectory of the agents
if min(F) < 0
   disp('Infeasibility occurred!')
else
    disp('Program was feasible!')
end

figure(1)
PlotEverything(X);