function K = servo_control_state(A,B,C,Q,R)
Aba = [A zeros(4,2)
       -C zeros(2,2)];
Bba = [B;zeros(2,2)];
Cba = [C zeros(2,2)];
Qc = ctrb(Aba,Bba);
isControllable = (rank(Qc) == size(Aba, 1));
K = lqr(Aba,Bba,Q,R);
K1 = K(:,1:4);
K2 = K(:,5:6);
end