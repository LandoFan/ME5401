function K = pole_placement(A,B,p)
    %x0 = [0;0;0;0];
    syms s;
    polynomial = (s-p(1))*(s-p(2))*(s-p(3))*(s-p(4));
    pol_cof = double(coeffs(polynomial));

    Ad = [0, 1, 0, 0;
          -pol_cof(3:4) ,0, 0;
          0 ,0, 0, 1;
          0, 0 ,-pol_cof(1:2)];
    
    %calcuate the Controllable matrix
    Wc = ctrb(A, B);
    Wc = Wc;
    is_controllable = (rank(Wc) == length(A));
    %change the sequence of Qc
    Con = [];
    temp = [];
    j = 1;
    jj= 5;
    for i = 1 : length(Wc)
        if mod(i,2) == 0
            Con(:,jj) = Wc(:,i);
            jj = jj + 1;
        else
            Con(:,j) = Wc(:,i);
            j = j + 1;
        end
    end
    temp = Con(:,5);
    Con(:,5) = Con(:,3);
    Con(:,3) = temp;
    
    temp = Con(:,6);
    Con(:,6) = Con(:,4);
    Con(:,4) = temp;
    
    is_controllable1 = (rank(Con(:,1:4)) == length(A));
    inv_Con = inv(Con(:,1:4));
    T = [];
    for i = 1 : length(A) - 2
        T(i,:) = inv_Con(2,:) * A^(i-1);
    end
    for i = 3 : length(A) 
        T(i,:) = inv_Con(4,:) * A^(i-3);
    end
    %T(4,:) = inv_Con(4,:);
    A_bar = T*A/T;
    B_bar = T*B;
    A_bar = round(A_bar);
    B_bar = round(B_bar);
    syms k11 k12 k13 k14 k21 k22 k23 k24;
    K_bar = [k11 k12 k13 k14;
             k21 k22 k23 k24];
    A_last = A_bar - B_bar * K_bar;
    
    eqn = solve(Ad == A_last);
    S = double(struct2array(eqn));
    K_bar = [S(1:4);S(5:8)];
    % K_bar = [S(1),S(3),S(5),S(7);
    %         S(2),S(4),S(6),S(8)];
    K = K_bar * T;
    K = round(K);
    % K = place(A, B, p);
    

    
end





















