% Control of Mobile Robots Using Barrier Functions Under Temporal Logic
% Specifications
%
% NOTE:         Please run the "init.m" file from the Robotarium package
%               prior to executing this code
%
% Description:  This file is a Robotarium simulation of two robots tasked
%               with reaching different goal regions. The entire task is
%               executed as a sequence of quadratic programs (QPs), with
%               the control barrier functions (CBFs) as constraints
%
% Authors:      Mohit Srinivasan, and Samuel Coogan
% Date:         06/01/2020

clear all;
close all;
clc;

N = 2;
iterations = 2000;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_pos_controller = create_si_position_controller();
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.15);
si_to_uni_dyn = create_si_to_uni_mapping();

hg1x1 = -1; hg2x2 = -1;

%% Plot the environment
c3 = 0.9; c4 = 0.7; c5 = 0.8; c6 = -0.4;

P2 = [1/(0.3)^2 0; 0 1/(0.1)^2];
P3 = [1/(0.3)^2 0; 0 1/(0.35)^2];
PlotGoalsObstacles(P2, P3, c3, c4, c5, c6);
hold on

%Set up video recorder
% vid = VideoWriter('TRO_Experiment_MATLAB.mp4', 'MPEG-4');
% vid.Quality = 100;
% vid.FrameRate = 72;
% open(vid);
% writeVideo(vid, getframe(gcf));

%% Initialization
for t = 0:500
   
   x = r.get_poses();

   dxi(:, 1) = [0 ; 0];
   dxi(:, 2) = [0 ; 0];

   dxi(:, 1) = si_pos_controller(x(1:2, 1), [-1.03 ; 0.4]);
   dxi(:, 2) = si_pos_controller(x(1:2, 2), [-1 ; 0.4]);
   
   dxu = si_to_uni_dyn(dxi, x);
    
   r.set_velocities(1:N, dxu);
   
   r.step();
   
%    writeVideo(vid, getframe(gcf)); 
    
end

%% Plotting variables
Plt_data1 = [];
Plt_data1 = [Plt_data1; x(1,1); x(2,1)];
p1 = plot(Plt_data1(1), Plt_data1(2), 'k-.', 'LineWidth', 3);
Plt_data2 = [];
Plt_data2 = [Plt_data2; x(1,2); x(2,2)];
p2 = plot(Plt_data2(1), Plt_data2(2), 'm-.', 'LineWidth', 3);
drawnow

% writeVideo(vid, getframe(gcf));

count = 0;
T = [];
T = [T, count];

U = [];

% writeVideo(vid, getframe(gcf));
F = [];
while(hg1x1 <= 0 || hg2x2 <= 0)
    
    x = r.get_poses();

    Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
    Plt_data2 = [Plt_data2, [x(1,2); x(2,2)]];

    p1.XData = Plt_data1(1,:);
    p1.YData = Plt_data1(2,:);
    p2.XData = Plt_data2(1,:);
    p2.YData = Plt_data2(2,:);

    X = [x(1:2,1);x(1:2,2)];
    
    [hg1x1, hg2x2, flag, dx] = GAB(X);
    H3 = [H3, hg3];
    F = [F, flag];
    dx = [dx(1:2), dx(3:4)];
    dx = si_to_uni_dyn(dx, x);

    r.set_velocities(1:N, dx);

%    writeVideo(vid, getframe(gcf)); 

    r.step();

end

display(min(F))
% close(vid);

r.debug();