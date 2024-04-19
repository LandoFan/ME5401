function draw3 = question3_draw(A,B,C,D,x0,L,K)
Ao = A - B * K;
Bo = B * K;
Do = A - L * C;
Co = zeros(4,4);

t = 0:0.01:30; % Time span for simulation
sys = ss(Ao, Bo, Co, Do);
[y, t, x] = initial(sys, x0, t);
%Plot the step response:
step(sys);
title('Step Response of Closed-Loop System');
figure;
subplot(4,1,1);
plot(t, x(:,1));
title('State x1');
subplot(4,1,2);
plot(t, x(:,2));
title('State x2');
subplot(4,1,3);
plot(t, x(:,3));
title('State x3');
subplot(4,1,4);
plot(t, x(:,4));
title('State x4');

u = -K*x';
figure;
plot(t, u);
title('Control Signal u');
figure;
plot(t,y);
title("The Response Performance of Output")
plot(t,C*x');
title("The Response Performance of Cx")

end