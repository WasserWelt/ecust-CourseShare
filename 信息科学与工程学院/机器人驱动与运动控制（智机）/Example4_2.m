% 速度模式电机空载响应
clear;
clc;
clf;
% 电机参数
Uc = [12;24];%利用数组存储两个不同的电枢电压
Ku = 1;
Ra = 2.49;
La = 6.1e-4;
Ka = 8.22e-2;
Ke = 8.24e-2;
Br = 4.1e-4;
Ir = 1.19e-5;
B1 = 0; % 空载
I1 = 0; % 空载
%N = 1; % 无减速器
N=50;
Bm = Br + B1/N^2;
Im = Ir + I1/N^2;

% 通用电机模型参数
Tmv = Ra * Im / (Ke * Ka + Ra * Bm);
Kmv = Ka * Ku / (Ke * Ka + Ra * Bm);
Kdv = Kmv * Ra / (Ka * Ku);

% 仿真时间及步长
total_t = 0.05; % 总仿真时间
dt = 0.0001; % 步长
step = total_t / dt + 1; % 步数

% 初始化各变量，以数组形式存储不同电枢电压对应的变量结果
theta = [0;0]; % 角度
omega = [0;0]; % 角速度
% epsilon = [0;0]; % 角加速度
ia = [0;0]; % 电枢电流
Ue = [0;0]; % 反电动势
dia = [0;0]; % 电流改变量
Tqm = [0;0]; % 电机电磁驱动力矩

% 对存储仿真结果的数组进行初始化
omega_draw = zeros(2,step);%转子角速度仿真结果
ia_draw = zeros(2,step);%电枢电流仿真结果

% 主程序
for i = 1:step
    omega_draw(:,i) = omega(:); % 记录当前角速度
    ia_draw(:,i) = ia(:); % 记录当前电流
    Ua = Ku .* Uc; % 该时刻控制电压
    Ue = Ke .* omega; % 该时刻反电动势
    ia = (La/Ra)/(La/Ra + dt)*ia + (dt/Ra)/(La/Ra + dt)*(Ua - Ue); % 该时刻电流（一阶惯性系统的离散表达式）
%     omega = Tmv/(Tmv + dt)*omega + (dt*Kmv)/(Tmv + dt)*Ua; % 该时刻角速度（一阶惯性系统的离散表达式）,利用电机通用模型计算加速度
    Tqm = ia .* Ka; % 该时刻电磁驱动力矩
    omega = (Im/Bm)/(Im/Bm + dt) .* omega + (dt/Bm)/(Im/Bm + dt) .* Tqm; % 利用电机动力学模型计算电机速度，计算结果与利用通用模型时相同      
    theta = theta + omega.*dt; % 角度迭代
end

% 绘制仿真结果
t = 0:dt:total_t;
yyaxis left;
p=plot(t,omega_draw(1,:),'r-',t,omega_draw(2,:),'b-');
p(1).LineWidth=1.5;
p(2).LineWidth=1.5;
ylabel('\omega_m(t) (rad/s)');
set(gca,'ycolor','k');
yyaxis right;
p=plot(t,ia_draw(1,:),'r-.',t,ia_draw(2,:),'b-.');
p(1).LineWidth=1;
p(2).LineWidth=1;
ylabel('i_a(t) (A)');
set(gca,'ycolor','k');
legend("\omega_m1","\omega_m2","i_a1","i_a2");
xlabel('t (s)');
