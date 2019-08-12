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
X0 = [-0.5; 1];
u = [u, u0];
X = [X, X0];
dt = 0.01;
hgCx = -1;
PA = [1/(0.6)^2 0; 0 1/(0.5)^2];
PB = [1/(0.6)^2 0; 0 1/(0.5)^2];
PC = [1/(0.35)^2 0; 0 1/(0.2)^2];
c1 = -0.3; c2 = 0; c3 = 0.3; c4 = 0; c5 = 0; c6 = 0;
CA = [c1; c2];
CB = [c3; c4];
CC = [c5; c6];
i = 1;
eps1 = [];
eps1 = [eps1, u0(3)];
eps2 = [];
eps2 = [eps2, u0(4)];
J = [];
J0 = norm(u0)^2;
J = [J, J0];
U = [];
U = [U, norm([u0(1:2,:)])^2];
Hax = [];
Hbx = [];
alphaA = 0.01;
alphaB = 30;

%% Algorithm

while( hgCx < 0 )
    
    [hAtx, hBtx, dx] = ReachC_ver2(X(:,i), u(3,i), u(4,i), PA, PB, PC, CA, CB, CC, alphaA, alphaB);
    u(3,i+1) = dx(3);
    u(4,i+1) = dx(4);
    u(1:2,i+1) = dx(1:2);
    X(:,i+1) = X(:,i) + dt*u(1:2,i+1);
    hgCx = 1 - (X(:,i+1) - CC)'*PC*(X(:,i+1) - CC);
    i = i+1;
    J = [J, norm(dx)^2];
    eps1 = [eps1, dx(3)];
    eps2 = [eps2, dx(4)];
    U = [U, norm(dx(1:2,:))^2];
    Hax = [Hax, hAtx];
    Hbx = [Hbx, hBtx];

end

%% Plot Trajectory of system
figure(1)
PlotEverything(X);

% figure(2)
% subplot(4,1,1)
% plot(eps1)
% title('Relaxation parameter $(\epsilon_1)$', 'interpreter', 'latex')
% subplot(4,1,2)
% plot(J)
% title('Cost function $(||v||_{2}^{2})$; $v = [u^T, \epsilon]^{T}$', 'interpreter', 'latex')
% subplot(4,1,3)
% plot(U)
% title('Control energy applied $(||u||_{2}^{2})$', 'interpreter', 'latex')
% subplot(4,1,4)
% plot(eps2)
% title('Relaxation parameter $(\epsilon_2)$', 'interpreter', 'latex')
% 
% figure(3)
% subplot(2,1,1)
% plot(Hax)
% title('Shrunk barrier $\tilde{h}_{A}(x)$', 'interpreter', 'latex')
% subplot(2,1,2)
% plot(Hbx)
% title('Shrunk barrier $\tilde{h}_{B}(x)$', 'interpreter', 'latex')
