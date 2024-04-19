function draw = question1_draw(A,B,C,D,x0,K)
    A_cl = A - B*K;
        B_cl = B;
        C_cl = C;
        D_cl = D;
        %Simulate the system with an initial state:
        t = 0:0.01:30; % Time span for simulation
        sys = ss(A_cl, B_cl, C_cl, D_cl);
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
        % 
        % %check the overshoot
        % y1max = max(y(:,1));
        % y2max = max(y(:,2));
        % 
        % 
        %% CHECK DRAWING
        % 给系统施加阶跃输入并模拟瞬态响应
        % t = 0:0.01:10;
        % u1 = [ones(length(t), 1), zeros(length(t), 1)]; % 第一个输入通道的阶跃信号
        % u2 = [zeros(length(t), 1),ones(length(t), 1)]; % 第二个输入通道的阶跃信号
        % 
        % % 对第一个输入通道进行模拟
        % sys_cl1 = ss(A_cl, B_cl, C_cl, D_cl);
        % % linearSystemAnalyzer(sys_cl1)
        % [y1, t1, x1] = lsim(sys_cl1, u1, t);
        % 
        % % 对第二个输入通道进行模拟
        % sys_cl2 = ss(A_cl, B_cl, C_cl, D_cl);
        % [y2, t2, x2] = lsim(sys_cl2, u2, t);
        % % 绘制第一个输入通道的响应
        % figure;
        % plot(t1, y1);
        % title('Response to [1; 0]');
        % xlabel('Time (s)');
        % ylabel('Output');
        % 
        % % 绘制第二个输入通道的响应
        % figure;
        % plot(t2, y2);
        % title('Response to [0; 1]');
        % xlabel('Time (s)');
        % ylabel('Output');
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




end