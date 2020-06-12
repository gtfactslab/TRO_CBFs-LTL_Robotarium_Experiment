%% Prioritization scheme for zeroing barrier functions
% Authors: Mohit Srinivasan & Sam Coogan
% Version: 1
% Date: 01/02/2019

%close all;
clear all;
clc;

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
    HAx = [HAx, hgAx];
    HBx = [HBx, hgBx];
    F = [F, flag];

    if flag < 0
        break;
    end
    X(:,i+1) = X(:,i) + dt*dx;
    i = i+1;
end

%% Plot Trajectory of system
if min(F) < 0
   disp('Infeasibility occurred!')
end