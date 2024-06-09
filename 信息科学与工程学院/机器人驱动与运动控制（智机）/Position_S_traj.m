function [epsilon, omega, theta] = Position_S_traj(stop_time, dt, set_a)
% 生成位置S曲线
% omgea, theta为期望的角速度的和位置
% stop_time为停止时间,dt为仿真步长
% set_a为设定加速度

step = stop_time/dt + 1;
epsilon = zeros(step,1);
omega = zeros(step,1);
theta = zeros(step,1);

for i = 2:1:step
    t = (i-1)*dt;
    if t <= 1
        epsilon(i) = set_a;% 匀加速段
    elseif t > 1 && t <= 2
        epsilon(i) = 0; % 匀速段
    elseif t > 2 && t <= 3
        epsilon(i) = -set_a; % 匀减速段
    elseif t > 3
        epsilon(i) = 0;
        omega(i-1) = 0; % 静止段
    end
    omega(i) = omega(i-1) + epsilon(i)*dt; % 迭代速度
    theta(i) = theta(i-1) + omega(i)*dt; % 迭代位置
end
end
