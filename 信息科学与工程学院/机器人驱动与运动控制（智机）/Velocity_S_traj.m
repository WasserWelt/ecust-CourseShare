function [epsilon, omega, theta] = Velocity_S_traj(stop_time, dt, set_aa)
% 生成速度S曲线
% dtheta, theta为期望的角速度和位置
% stop_time为停止时间 dt为仿真步长
% set_aa为急动度

step = stop_time/dt + 1;
epsilon = zeros(step,1);
omega = zeros(step,1);
theta = zeros(step,1);

for i = 2:1:step
    t = (i-1)*dt;
    if t <= 1
        d3theta = set_aa;% 加加速段
    elseif t > 1 && t <= 1.5
        d3theta = 0; % 匀加速段
    elseif t > 1.5 && t <= 2.5
        d3theta = -set_aa; % 减加速段
    elseif t > 2.5 && t <= 3.5
        d3theta = 0;
        epsilon(i) = 0; % 匀速段
    elseif t > 3.5 && t <= 4.5
        d3theta = -set_aa; % 加减速段
    elseif t >= 4.5 && t < 5
        d3theta = 0; % 匀减速段
    elseif t >= 5 && t < 6
        d3theta = set_aa; % 减减速段
    elseif t >= 6
        d3theta = 0;
        epsilon(i) = 0;
        omega(i) = 0; % 静止段
    end
    epsilon(i) = epsilon(i-1) + d3theta*dt; % 迭代加速度
    omega(i) = omega(i-1) + epsilon(i)*dt; % 迭代速度
    theta(i) = theta(i-1) + omega(i)*dt; % 迭代加速度
end
end