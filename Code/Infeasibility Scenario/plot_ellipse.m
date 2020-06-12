function plot_ellipse(P, a, b, R,  c, type)

      theta = 0:0.00001:2*pi;
      x = (1/sqrt(P(1,1)))*cos(theta) + a;
      y = (1/sqrt(P(2,2)))*sin(theta) + b;
      C = [x; y];
      C = R*C;
      x = C(1,:);
      y = C(2,:);
      xlim([-1.6 1.6])
      ylim([-1 1])
      plot(x, y, 'Color', c, 'LineStyle', type);
      hold off

end 