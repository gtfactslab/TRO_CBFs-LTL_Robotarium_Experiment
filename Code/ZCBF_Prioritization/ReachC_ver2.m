function [hAtx, hBtx, y] = ReachC_ver2(X, eps1, eps2, PA, PB, PC, CA, CB, CC, alphaA, alphaB)

    gamma = 1;

    RA = eye(2);
    RB = [1/sqrt(2) -1/sqrt(2); 1/sqrt(2) 1/sqrt(2)];
    RC = eye(2);
    
    H = [1 0 0 0; 0 1 0 0; 0 0 alphaA 0; 0 0 0 alphaB];
    
    YA = RA*(X - CA);
    YB = RB*(X - CB);
    YC = RC*(X - CC);
    
%     hAx = 1 - YA'*RA*PA*RA'*YA;
%     hBx = 1 - YB'*RB*PB*RB'*YB;
    hgCx = 1 - YC'*RC*PC*RC'*YC;

    hAx = (X - CA)'*PA*(X - CA) - 1;
    hBx = (X - CB)'*PB*(X - CB) - 1;

    hAtx = hAx;
    hBtx = hBx;
    
    A1 = 2*(X - CC)'*PC;
    A2 = -2*(X - CA)'*PA;
    A3 = -2*(X - CB)'*PB;
        
    B1 = gamma*sign(hgCx);
    B2 = gamma*(hAtx)^3;
    B3 = gamma*(hBtx)^3;
         
    a = [A1 0 0; A2 -1 0; A3 0 -1; 0 0 -1 0; 0 0 0 -1];
    b = [B1; B2; B3; 0; 0];
    
    tic
    y = quadprog(H, [], a, b, [], [], [], [], []);
    toc
    
end