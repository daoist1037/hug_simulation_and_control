clc;
clear;
% 与simulink结果不同，分析原因中
t = 0:0.1:8;
exp_pos = sin(pi()*t);
pos_1 = zeros(length(t)+1,1)';
vel_1 = zeros(length(t)+1,1)';
pos_2 = zeros(length(t)+1,1)';
vel_2 = zeros(length(t)+1,1)';
T = 0.1;
m = 1;
error_p = 0;
error_i = 0;
error_d = 0;
A_    = [1 T;0 1];
B_    = [T^2/m/2;T/m];       
for i = 1:length(t)
    F1 = MPCcontroller(t(i), pos_1(i), vel_1(i));
    vel_1(i+1) = vel_1(i) + F1 / m * T;
    pos_1(i+1) = pos_1(i) + (vel_1(i) + vel_1(i+1)) * T / 2;
    
    temp = exp_pos(i) - pos_2(i);
    if i == 1
        error_d = 0;
    else
        error_d = (temp - error_p) / T;
    end
    error_p = temp;
    error_i = error_i + error_p;
    F2 = PIDcontroller(error_p, 0, error_d);
    vel_2(i+1) = vel_2(i) + F2 / m * T;
    pos_2(i+1) = pos_2(i) + (vel_2(i) + vel_2(i+1)) * T / 2;
end

plot(t,pos_1(1:end-1), t,exp_pos, t,pos_2(1:end-1));
legend('mpc','origin','pid');


function u = MPCcontroller(time, pos, vel)
    %参数设置
    m    = 1.05;            %滑块质量,增加了5%作为建模误差
    T    = 0.1;             %控制周期100ms
    p    = 40;              %控制时域（预测时域）
    Q    = 10*eye(2*p);     %累计误差权重
    W    = 0.0001*eye(p);   %控制输出权重      
    umax = 100 * ones(p,1); %控制量限制，即最大的力
    Rk   = zeros(2*p,1);    %参考值序列
    timeseris = time:T:time+T*(p-1);
    Rk(1:2:end) = sin(pi()*(timeseris + T ));    
    Rk(2:2:end) = pi()*cos(pi()*(timeseris + T));    %参考速度
    %构建中间变量
    xk    = [pos;vel];    %xk
    A_    = [1 T;0 1];    %离散化预测模型参数A
    B_    = [T^2/m/2;T/m];%离散化预测模型参数B
    psi   = zeros(2*p,2); %psi
    for i=1:1:p
        psi(i*2-1:i*2,1:2) = A_^i;
    end
    theta = zeros(2*p,p); %theta
    for i=1:1:p
        for j=1:1:i
            theta(i*2-1:i*2,j) = A_^(i-j)*B_;
        end
    end
    E = psi*xk-Rk;            %E
    H = 2*(theta'*Q*theta+W); %H
    f = (2*E'*Q*theta)';      %f
    %优化求解
    coder.extrinsic('quadprog');
    Uk=quadprog(H,f,[],[],[],[],-umax,umax);
    %返回控制量序列第一个值
    u = 0.0;                %显示指定u的类型
    u = Uk(1);
end

function u = PIDcontroller(error_p, error_i, error_d)
    Kp = 0;
    Ki = 0;
    Kd = 16;
    u = Kp*error_p + Ki*error_i + Kd*error_d;
end
