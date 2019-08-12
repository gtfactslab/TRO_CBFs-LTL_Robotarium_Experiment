function PlotEverything(X)

c1 = -0.3; c2 = 0; c3 = 0.6; c4 = 0; c5 = 0; c6 = 0;
            
P1 = [1/(0.6)^2 0; 0 1/(0.5)^2];
P2 = [1/(0.6)^2 0; 0 1/(0.5)^2];
P3 = [1/(0.35)^2 0; 0 1/(0.2)^2];

figure(1)
PlotGoalsObstacles(P1, P2, P3, c1, c2, c3, c4, c5, c6);
hold on
PlotTrajectory(X);
hold on

end