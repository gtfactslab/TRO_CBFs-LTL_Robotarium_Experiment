function PlotGoalsObstacles(P1, P2, c1, c2, c3, c4)
    
    grid on
    grid minor
    xlabel('$x_{1}$', 'interpreter', 'latex')
    ylabel('$x_{2}$', 'interpreter', 'latex')
    hold on
    
    R1 = eye(2);
    R2 = eye(2);
    axis([-1.6 1.6 -1 1])
    plot_ellipse(P1, c1, c2, R1, 'r', '-');
    hold on
    
    plot_ellipse(P2, c3, c4, R2, 'r', '-');
    hold on
    
end