function [K L] = servo_control(A,B,C,po)
Aba = [A zeros(4,2)
       -C zeros(2,2)];
Bba = [B;zeros(2,2)];
Cba = [C zeros(2,2)];
Qc = ctrb(Aba,Bba);
isControllable = (rank(Qc) == size(Aba, 1));
Q = diag([1 1 1 1 1 1]);
R = diag([1 1]);
K = lqr(Aba,Bba,Q,R);
K1 = K(:,1:4);
K2 = K(:,5:6);
L = pole_placement(A',C',po);
L = L';

end