function PlotTrajectory(X)
    
        x = X(1, :);
        y = X(2, :);
        plot(x,y,'k')
        axis([-1.5 1.5 -1.5 1.5])
        hold on
                 
end