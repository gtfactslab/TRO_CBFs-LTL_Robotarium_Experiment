function PlotEverything(X)

PA = [1/(0.4)^2 0; 0 1/(0.2)^2];
PB = [1/(0.4)^2 0; 0 1/(0.2)^2];
cx_A = 0.9; cy_A = 0.7; cx_B = 0.6; cy_B = -0.6;

figure(1)
PlotGoalsObstacles(PA, PB, cx_A, cy_A, cx_B, cy_B);
hold on
PlotTrajectory(X);
hold on

end