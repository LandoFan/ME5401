clc
clear all
a = 5;
b = 0;
c = 6;
d = 0;
A = [-8.8487+(a-b)/5,-0.0399,-5.55+(c+d)/10,3.5846;
     -4.574,2.5010*(d+5)/(c+5),-4.3662,-1.1183-(a-c)/20;
     3.7698,16.1212-c/5,-18.2103+(a+d)/(b+4),4.4936;
     -8.5645-(a-b)/(c+d+2),8.3742,-4.4331,-7.7181*(c+5)/(b+5)];
B = [0.0564+b/(10+c),0.0319;
    0.0165-(c+d-5)/(1000+20*a),-0.02;
    4.4939,1.5985*(a+10)/(b+12);
    -1.4269,-0.2730];
C = [-3.2988,-2.1932+(10*c+d)/(100+5*a),0.037,-0.0109;
    0.2922-a*b/500,-2.1506,-0.0104,0.0163];
D = zeros(size(C,1), size(B,2));
x0 = [0.5;-0.1;0.3;-0.8];
%% question1 根据二阶系统，令kesi = 0.7, wn = 1/1.4
p = [-1-0.5i, -1+0.5i, -4, -9]; % kesi = 0.89  wn = 1.12  
K1 = pole_placement(A,B,p);
K = place(A, B, p);
%% question2
Q = diag([1 1 1 1]);
R = diag([1 1]);
K2 = lqr(A,B,Q,R);

%% question 3
O = obsv(A,C);
isObservable = (rank(O) == size(A, 1));
po = [-12,-16,-20,-24];
L = pole_placement(A',C',po);
L = L';
question3_draw(A,B,C,D,x0,L,K2)

%% qustion4 
syms s
G = C*inv(s*eye(4)-A)*B;
[rowG,columnG] = size(G);

% delta = [];
% B_star = [];
% C_star = [];
% for i = 1 : rowG
%     for j = 1 : columnG
%         if C(i,:)*A^(j-1)*B ~= 0
%             delta(i) = j;
%             break;
%         end
%         delta(i) = columnG;
%     end
% end
% 
% for i = 1 : rowG
%     B_star(i,:) = C(i,:)*A^(delta(i)-1)*B;
%     C_star(i,:) = C(i,:)*A^delta(i);
% end
% K_41 = inv(B_star) * C_star;
B_star = [];
C_star2 = [];
B_star(1,:) = C(1,:)*A*B;
B_star(2,:) = C(2,:)*A*B;
C_star2(1,:) = C(1,:)*(A+4*eye(4))^2;
C_star2(2,:) = C(2,:)*(A+2*eye(4))^2;
K_4 = inv(B_star) * C_star2;
F = inv(B_star);
H = C*inv(s*eye(4)-(A-B*K_4))*B*F;


%% question5 servo control
po = [-12,-16,-20,-24];
[K5,L5] = servo_control(A,B,C,po);

%% question6 
xsp = [0;0.5;-0.4;0.3];
xsp1 = [0;0.5];
xsp2 = [-0.4;0.3];
C1 = [1 0 0 0;
      0 1 0 0;];
C2 = [0 0 1 0;
      0 0 0 1;];
Q1 = diag([500 500 10 100 150 50]);
R1 = diag([1 1]);
K61 = servo_control_state(A,B,C1,Q1,R1);
K62 = servo_control_state(A,B,C2,Q1,R1);
