% Control of Mobile Robots Using Barrier Functions Under Temporal Logic
% Specifications
%
% NOTE:         Please run the "init.m" file from the Robotarium package
%               prior to executing this code
%
% Description:  This file is a Robotarium simulation of three robots tasked
%               with reaching different goal regions, while avoiding an
%               obstacle and maintaining connectivity. The entire task is
%               executed as a sequence of quadratic programs (QPs), with
%               the control barrier functions (CBFs) as constraints
%
% Authors:      Mohit Srinivasan, and Samuel Coogan
% Date:         06/01/2020

clear all;
close all;
clc;

N = 3;
iterations = 2000;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_pos_controller = create_si_position_controller();
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 0.15);
si_to_uni_dyn = create_si_to_uni_mapping();

hg1x1 = -1; hg2x2 = -1; hg3x1 = -1; hg3x2 = -1; hg1x3 = -1; hg2x3 = -1; ...
hg3x3 = -1;

H_T1_1 = [];
H_T1_1 = [H_T1_1, hg1x1];
H_T2_2 = [];
H_T2_2 = [H_T2_2, hg2x2];
H_B_1 = [];
H_B_1 = [H_T2_2, hg3x1];
H_B_2 = [];
H_B_2 = [H_T2_2, hg3x2];
H_obs1_ab = [];
H_obs2_ab = [];
H_obs1_c = [];
H_obs2_c = [];

%% Plot the environment
c3 = 0.9; c4 = 0.7; c5 = 0.8; c6 = -0.7; c7 = -1; c8 = 0.4; c9 = 0.3; c10 = 0;

P2 = [1/(0.3)^2 0; 0 1/(0.1)^2];
P3 = [1/(0.3)^2 0; 0 1/(0.1)^2];
P4 = [1/(0.4)^2 0; 0 1/(0.2)^2];
P5 = [1/(0.3)^2 0; 0 1/(0.35)^2];
PlotGoalsObstacles(P2, P3, P4, P5, c3, c4, c5, c6, c7, c8, c9, c10);
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
   dxi(:, 3) = [0 ; 0];

   dxi(:, 1) = si_pos_controller(x(1:2, 1), [-1.10 ; 0.4]);
   dxi(:, 2) = si_pos_controller(x(1:2, 2), [-1.02 ; 0.4]);
   dxi(:, 3) = si_pos_controller(x(1:2, 3), [-0.83 ; 0.4]);
   
   dxi = si_barrier_cert(dxi, x);
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
Plt_data3 = [];
Plt_data3 = [Plt_data3; x(1,3); x(2,3)];
p3 = plot(Plt_data3(1), Plt_data3(2), 'b-.', 'LineWidth', 3);
drawnow

% writeVideo(vid, getframe(gcf));

%% Solve the first reachability objective
for t = 0:iterations  
    if (hg1x3 >= 0)
        
        break;
        
    else
    
        while(hg1x3 <= 0)
        
            x = r.get_poses();
            Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
            Plt_data2 = [Plt_data2, [x(1,2); x(2,2)]];
            Plt_data3 = [Plt_data3, [x(1,3); x(2,3)]];
            
            p1.XData = Plt_data1(1,:);
            p1.YData = Plt_data1(2,:);
            p2.XData = Plt_data2(1,:);
            p2.YData = Plt_data2(2,:);
            p3.XData = Plt_data3(1,:);
            p3.YData = Plt_data3(2,:);
            
            X = [x(1:2,1); x(1:2,2); x(1:2,3)];
        
            [hg1x3, dx] = ReachA(X);
            
            dx = [dx(1:2), dx(3:4), dx(5:6)];
            dx = si_barrier_cert(dx, x);
            dx = si_to_uni_dyn(dx, x);
                        
            r.set_velocities(1:N, dx);
        
%             writeVideo(vid, getframe(gcf));             
            
            r.step();
        
        end
    
    end
    
end

%% Solve the second reachability objective
% writeVideo(vid, getframe(gcf));

for t = 0:iterations
    
    if (hg2x3 >= 0)
        
        break;
        
    else
    
        while(hg2x3 <= 0)

            x = r.get_poses();
            Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
            Plt_data2 = [Plt_data2, [x(1,2); x(2,2)]];
            Plt_data3 = [Plt_data3, [x(1,3); x(2,3)]];
            X = [x(1:2,1); x(1:2,2); x(1:2,3)];

            p1.XData = Plt_data1(1,:);
            p1.YData = Plt_data1(2,:);
            p2.XData = Plt_data2(1,:);
            p2.YData = Plt_data2(2,:);
            p3.XData = Plt_data3(1,:);
            p3.YData = Plt_data3(2,:);
        
            [hg2x3, dx] = ReachB(X);
    
            dx = [dx(1:2), dx(3:4), dx(5:6)];
            dx = si_barrier_cert(dx, x);
            dx = si_to_uni_dyn(dx, x);
                        
            r.set_velocities(1:N, dx);
        
%             writeVideo(vid, getframe(gcf)); 
                     
            r.step();
        
        end
    
    end
        
end

%% Solve the third reachability objective
% writeVideo(vid, getframe(gcf));

for t = 0:iterations
    
    if (hg3x3 >= 0)     
        break;       
    else
    
        while(hg3x3 <= 0) 
        
            x = r.get_poses();
            Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
            Plt_data2 = [Plt_data2, [x(1,2); x(2,2)]];
            Plt_data3 = [Plt_data3, [x(1,3); x(2,3)]];

            p1.XData = Plt_data1(1,:);
            p1.YData = Plt_data1(2,:);
            p2.XData = Plt_data2(1,:);
            p2.YData = Plt_data2(2,:);
            p3.XData = Plt_data3(1,:);
            p3.YData = Plt_data3(2,:);

            X = [x(1:2,1); x(1:2,2); x(1:2,3)];
        
            [hg3x3, dx] = ReachC(X);
            
            dx = [dx(1:2), dx(3:4), dx(5:6)];
            dx = si_barrier_cert(dx, x);
            dx = si_to_uni_dyn(dx, x);
                                                
            r.set_velocities(1:N, dx);
        
%             writeVideo(vid, getframe(gcf)); 
            
            r.step();
        
        end  
    end
        
end

count = 0;
T = [];
T = [T, count];

U = [];

%% Solve the fourth and fifth reachability objective
% writeVideo(vid, getframe(gcf));

while(hg1x1 <= 0 || hg2x2 <= 0 || hg3x1 <= 0 || hg3x2 <= 0)

    x = r.get_poses();

    base = [0 ; 0];
    base = si_pos_controller(x(1:2, 3), [-1.2 ; 0.8]);

    Plt_data1 = [Plt_data1, [x(1,1); x(2,1)]];
    Plt_data2 = [Plt_data2, [x(1,2); x(2,2)]];
    Plt_data3 = [Plt_data3, [x(1,3); x(2,3)]];

    p1.XData = Plt_data1(1,:);
    p1.YData = Plt_data1(2,:);
    p2.XData = Plt_data2(1,:);
    p2.YData = Plt_data2(2,:);
    p3.XData = Plt_data3(1,:);
    p3.YData = Plt_data3(2,:);

    X = [x(1:2,1);x(1:2,2); x(1:2,3)];

    if (hg1x1 > 0 && hg2x2 > 0)
        [hgoal1x1, hgoal2x2, hg3, hg3x1, hg3x2, dconn, h_obs1c, h_obs2c, dx] = GC(X);
        U = [U, dx];
        H_obs1_c = [H_obs1_c, h_obs1c];
        H_obs2_c = [H_obs2_c, h_obs2c];
        H_B_1 = [H_B_1, hg3x1];
        H_B_2 = [H_B_2, hg3x2];
        dx = [dx(1:2), dx(3:4), base];
        dx = si_barrier_cert(dx, x);
        dx = si_to_uni_dyn(dx, x);

        dx = 0.35*dx;
        r.set_velocities(1:N, dx);

%             writeVideo(vid, getframe(gcf)); 

        r.step();

    else                                                                

        [hg1x1, hg2x2, hg3, hgoal3x1, hgoal3x2, dconn, h_obs1ab, h_obs2ab, dx] = GAB(X);
        H_obs1_ab = [H_obs1_ab, h_obs1ab];
        H_obs2_ab = [H_obs2_ab, h_obs2ab];
        H_T1_1 = [H_T1_1, hg1x1];
        H_T2_2 = [H_T2_2, hg2x2];
        dx = [dx(1:2), dx(3:4), base];
        dx = si_barrier_cert(dx, x);
        dx = si_to_uni_dyn(dx, x);

        dx = 0.35*dx;

        r.set_velocities(1:N, dx);

%             writeVideo(vid, getframe(gcf)); 

        r.step();

    end

    count = count + 0.033;
    T = [T, count];

end

if(hg3x1 > 0 && hg3x2 > 0)             
    hg1x1 = -1;                                                                 
    hg2x2 = -1;                                                                 
    hg3x1 = -1;
    hg3x2 = -1;        
end
            
% close(vid);

% Plot graphs
figure(2)
subplot(2,2,1)
plot(T(1:length(H_T1_1)), H_T1_1)
ylabel('Progress of R_{1} towards Goal A')
subplot(2,2,2)
plot(T(1:length(H_T2_2)), H_T2_2)
ylabel('Progress of R_{2} towards Goal B')
subplot(2,2,3)
plot(T(1:length(H_T1_1)), H_T1_1+H_T2_2);
ylabel('Total Progress towards Goal A and Goal B')

r.debug();