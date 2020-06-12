function PlotTrajectory(X)
    
    x = X(1, :);
    y = X(2, :);
    plot(x,y,'k')
    axis([-1.6 1.6 -1 1])
    hold on
 
    x = X(3, :);
    y = X(4, :);
    plot(x,y,'g')
    axis([-1.6 1.6 -1 1])
    hold on
                 
end