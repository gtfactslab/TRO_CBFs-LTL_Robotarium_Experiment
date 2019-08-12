function PlotGoalsObstacles(P2, P3, P4, P5, c3, c4, c5, c6, c7, c8, c9, c10)

    plot_ellipse(P2, c3, c4, 'g');
    %dim = [.62 .31 .3 .3];
    str = 'TARGET 1';
    %annotation('textbox',dim,'String',str,'FitBoxToText','on','EdgeColor','none');
    text(0.8, 0.7, str, 'Color','black','FontSize',14, 'FontWeight', 'bold');
    hold on
    
    plot_ellipse(P3, c5, c6, 'g');
    %dim = [.57 0.06 .3 .3];
    str = 'TARGET 2';
    %annotation('textbox',dim,'String',str,'FitBoxToText','on','EdgeColor','none');
    text(0.7, -0.7, str, 'Color','black','FontSize',14, 'FontWeight', 'bold');
    hold on
    
    plot_ellipse(P4, c7, c8, 'g');
    %dim = [.27 .31 .3 .3];
    str = 'BASE';
    %annotation('textbox',dim,'String',str,'FitBoxToText','on','EdgeColor','none');
    text(-1.1, 0.4, str, 'Color','black','FontSize',14, 'FontWeight', 'bold');
    hold on
    
    plot_ellipse(P5, c9, c10, 'r');
    %dim = [.505 .2105 .3 .3];
    str = 'DANGER ZONE';
    %annotation('textbox',dim,'String',str,'FitBoxToText','on','EdgeColor','none');
    text(0.1, 0, str, 'Color','black','FontSize',14, 'FontWeight', 'bold');
    hold on     
    
end