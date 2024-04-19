% Given system matrices and initial conditions
a = 5;
b = 0;
c = 6;
d = 0;
A = [-8.8487+(a-b)/5, -0.0399, -5.55+(c+d)/10, 3.5846;
     -4.574, 2.5010*(d+5)/(c+5), -4.3662, -1.1183-(a-c)/20;
     3.7698, 16.1212-c/5, -18.2103+(a+d)/(b+4), 4.4936;
     -8.5645-(a-b)/(c+d+2), 8.3742, -4.4331, -7.7181*(c+5)/(b+5)];
B = [0.0564+b/(10+c), 0.0319;
    0.0165-(c+d-5)/(1000+20*a), -0.02;
    4.4939, 1.5985*(a+10)/(b+12);
    -1.4269, -0.2730];
C = [-3.2988, -2.1932+(10*c+d)/(100+5*a), 0.037, -0.0109;
    0.2922-a*b/500, -2.1506, -0.0104, 0.0163];
D = zeros(size(C,1), size(B,2));
x0 = [0.5; -0.1; 0.3; -0.8];

% Define the observer poles (desired eigenvalues) - Ensure it's of rank(B) or less
observer_poles = [-5, -5];

% Design the state observer
L = place(A', C', observer_poles)'; 

% Define the control law using LQR
Q = eye(size(A));
R = eye(size(B, 2));
K = lqr(A, B, Q, R);


% Define the augmented system
Aaug = [A, -B*K; L*C, A - L*C - B*K];
Baug = [B; zeros(size(B))];
Caug = [C, zeros(size(C))];
Daug = D;

% Define the initial condition for the augmented system
xaug0 = [x0; zeros(size(x0))];

% Define simulation time
t = 0:0.01:20;

% Simulate the augmented system
[~, xaug, ~] = lsim(Aaug, Baug, Caug, Daug, xaugh0, t);

% Extract the state and observer state
x_est = xaug(:, 1:4);
x_observer = xaug(:, 5:8);

% Plot the state estimation error
figure;
plot(t, x_est - x_observer);
title('State Estimation Error');
xlabel('Time (s)');
ylabel('Error');

% Simulate the original system with observer-based control
sys_cl = ss(Aaug - Baug*K - Laug*Caug, Baug, Caug, Daug);
t = 0:0.01:20;
u = zeros(size(t));
[y, t, x] = lsim(sys_cl, u, t, xaugh0);

% Plot the response of the controlled system
figure;
plot(t, y);
title('Controlled System Response');
xlabel('Time (s)');
ylabel('Output');
