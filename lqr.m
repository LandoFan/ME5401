function K = lqr(A,B,Q,R)
    %x0 = [0.5;-0.1;0.3;-0.8];
    %question2
    syms P;
    %AREeqn = P * A + A' * P - P * B / R * B' * P + Q == 0;
    matrix = [A ,-B/R*B';
              -Q , -A'];
    [V, DD] = eig(matrix);
    [row, col] = size(DD);
    d = diag(DD);
    dd = [];
    record_vector = [];
    j = 1;
    for i = 1 : length(d)
        if d(i) < 0
            dd(j) = d(i);
            record_vector(j) = i;
            j = j + 1;
        end
    end
    final_vector = [];
    for i = 1 : length(dd)
        final_vector(:,i) = V(:,record_vector(i));
    end
    v = [];
    mu = [];
    width = size(final_vector,1);
    v = final_vector(1:width/2,:);
    mu = final_vector(width/2+1:width,:);
    P = mu / v;
    K = real(inv(R) * B' * P);
    
    % [K2,S2,P2] = lqr(A,B,Q,R);
    
    
    % t = 0:0.1:30;
    % sys2 = ss(A-B*K, B, C, D);
    % [y, t, x] = initial(sys2, x0, t);
    % step(sys2);
    % figure;
    % plot(t,y)
end

