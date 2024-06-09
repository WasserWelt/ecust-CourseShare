function [epsilon, omega, theta] = Slope_traj(stop_time, dt, set_omega, stop_theta)
% 生成斜坡曲线
% omgea, theta为期望的角速度的和位置
% epsilon为期望加速度，对于斜坡输入，理论上为无穷大
% stop_time为停止时间 dt为仿真步长
% set_omega为设定速度 stop_theta为停止角度

step = stop_time/dt + 1;
omega = zeros(step,1);
theta = zeros(step,1);
epsilon = zeros(step,1);

for i = 2:1:step
    if theta(i-1) < stop_theta
        theta(i) = theta(i-1) + set_omega*dt;
        omega(i) = set_omega;
    else
        theta(i) = stop_theta;
        omega(i) = 0;
    end
    epsilon(i) = (omega(i)-omega(i-1))/dt;
end
