function PlotGoalsObstacles(P1, P2, P3,c1, c2, c3, c4, c5, c6)
    
    grid on
    grid minor
    xlabel('$x_{1}$', 'interpreter', 'latex')
    ylabel('$x_{2}$', 'interpreter', 'latex')
    hold on
    
    R1 = eye(2);
    R2 = [1/sqrt(2) -1/sqrt(2); 1/sqrt(2) 1/sqrt(2)];
%     R2 = eye(2);
    R3 = eye(2);
    plot_ellipse(P1, c1, c2, R1, 'r', '-');
    hold on
    
    plot_ellipse(P2, c3, c4, R2, 'r', '-');
    hold on
    
    plot_ellipse(P3, c5, c6, R3, 'b', '-');
    hold on
    
    
end