function [hg1, hg2, flag, y] = GAB(X)

    gamma = 1;
    
    H = eye(4);
    
    P2 = [1/(0.3)^2 0; 0 1/(0.1)^2];
    P3 = [1/(0.3)^2 0; 0 1/(0.35)^2];

    c3 = 0.9; c4 = 0.7; c5 = 0.8; c6 = -0.4;
    
    zero = [0 0; 0 0];
    C2 = [c3; c4];
    C3 = [c5; c6];
    
    P_2 = [P2 zero; zero zero];
    P_3 = [zero zero; zero P3];
    
    norm21 = norm(X(1:2) - X(3:4));
 
    hg1 = 1 - (X - [C2; 0; 0])'*P_2*(X - [C2; 0; 0]);                       % Goal for Robot 1
    hg2 = 1 - (X - [0; 0; C3])'*P_3*(X - [0; 0; C3]);                       % Goal for Robot 2
    dconn = (X(3) + 0.1)^4 + 0.1;
    hg3 = dconn^2 - norm21^2;
    
    A1 = 2*(X - [C2; 0; 0])'*P_2;
    A2 = 2*(X - [0 ; 0; C3])'*P_3;
    A3 = - [0, 0, (2*dconn)*(4*(X(3) + 0.1)^3), 0] - [2*(X(3) - X(1)), 2*(X(4) - X(2)), -2*(X(3) - X(1)), - 2*(X(4) - X(2))];
    
    B1 = gamma*sign(hg1);
    B2 = gamma*sign(hg2);
    B3 = gamma*hg3;

    a = [A1; A2; A3];
    b = [B1; B2; B3];
    
    opts = optimoptions(@quadprog, 'Display', 'off', 'ConstraintTolerance', 1e-30);
    [y, ~, flag] = quadprog(H, [], a, b, [], [], [], [], [], opts);
    disp(flag)
    if flag < 0
        disp('Infeasibility!')
    else
        disp('All good!')
    end
    
end