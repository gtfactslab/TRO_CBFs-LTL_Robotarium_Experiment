function PlotGoalsObstacles(P2, P3, c3, c4, c5, c6)

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
    
end