%% 例题5-6~8：针对单自由度机器人，可设定轨迹类型、电机工作模式、速度前馈、加速度前馈和集中力矩前馈的独立关节PID控制器仿真例程
% 利用循环模拟控制系统时钟，假定每个采样周期都进行伺服控制，迭代计算电机速度，且每次伺服控制都更新伺服期望值定时器

%% 程序启动，清理内存和绘图
clear;
clc;
clf;

%% 设定期望轨迹、电机工作模式、控制器类型、减速比、模型偏差和闭环系统特性
%% 定义跟踪轨迹曲线类型符号
Hold_on = 0;
Traj_Slope = 1;
Traj_Pos_S = 2;
Traj_Vol_S = 3;

%% 选择跟踪轨迹

% Traj_Type = Hold_on;
% Traj_Type = Traj_Slope;
Traj_Type = Traj_Pos_S;
% Traj_Type = Traj_Vol_S;

%% 定义电机工作模式符号
Amp_Voltage = 0;
Amp_Current = 1;
% 选择电机工作模式
% Amp_Type = Amp_Voltage;
Amp_Type = Amp_Current;

%% 选择干扰力矩
Dis_Tq = 1; % 1-有干扰力矩，0-无干扰力矩；

%% 选择积分项和前馈
Int_Val = 1; % 积分项选择：1-有效，0-无效；
Vol_Forward = 1; % 速度前馈：1-有效，0-无效；
Acc_Forward = 1; % 加速度前馈：1-有效，0-无效；
Tq_Forward = 1; % 非线性力矩集中前馈：1-有效，0-无效；

%% 设定减速比
N = 30; 

%% 设定闭环系统响应特性
ts = 0.1;%调节时间
Zeta = 1;%阻尼比
if (Zeta == 1), Omega_n = 4.75/ts; end % 临界阻尼系统的自然频率
if (Zeta < 1), Omega_n = 3.5/(Zeta*ts); end % 欠阻尼系统的自然频率
if (Zeta > 1), Omega_n = 3.3/((Zeta- sqrt(Zeta^2-1))*ts); end % 过阻尼系统的自然频率

%% 设定模型偏差
D_m = 0.05; % 理论质量与实际质量的偏差率
D_L = 0.05; % 理论杆长与实际杆长的偏差率

%% 系统和电机参数初始化
m = 0.5; % 实际质量
g = 9.8;
L = 0.1; % 实际杆长
Ra = 2.49;
La = 6.1e-4;
Ka = 8.22e-2;
Ke = 8.24e-2;
Br = 4.1e-4;
Ir = 1.19e-5;
Bl = 2e-2;
Il = 5e-3;
Bm = Br + Bl/(N^2);
Im = Ir + Il/(N^2);
m_D = m * (1 - D_m); % 理论质量
L_D = L * (1 - D_L); % 理论杆长
Il_D = m_D * L_D^2; % 理论连杆惯量
Im_D = Ir + Il_D/N^2; % 理论等效惯量

%% 计算电机控制模型参数
%% 速度模式电机模型参数
Ku = 3;
Tmv = (Ra*Im_D)/(Ke*Ka+Ra*Bm); % 用理论等效惯量计算
Kmv = (Ka*Ku)/(Ke*Ka+Ra*Bm);
Kdv = Kmv * Ra/(Ka*Ku);
%% 力矩模式电机模型参数
Kg = 1;
Kt = Ka*Kg;
Tmt = Im_D/Bm; % 用理论等效惯量计算
Kmt = Kt/Bm;
Kdt = 1/Bm;
%% 通用电机模型参数
if(Amp_Type == Amp_Voltage) 
    Tm = Tmv;
    Km = Kmv;
    Kd = Kdv;
end
if(Amp_Type == Amp_Current) 
    Tm = Tmt;
    Km = Kmt;
    Kd = Kdt;
end

%% 计算控制器增益
%% PID控制器增益
Tv = Tm;
Kv = 2*Zeta*Omega_n/Km;
Kp = Omega_n/(2*Zeta);
%% 速度和加速度前馈增益
Kvff = 1/Km;
Kaff = Tm/Km;
%% 力矩前馈增益
if (Amp_Type == Amp_Voltage), Ktff = Kd/Km; end % 速度模式电机的力矩前馈增益
if (Amp_Type == Amp_Current), Ktff = 1/Kt; end % 力矩模式电机的力矩前馈增益

%% 设置仿真时间、步长和步数
dt = 0.0001;
if (Traj_Type == Hold_on), total_time = 1;end
if (Traj_Type == Traj_Slope), total_time = 2;end
if (Traj_Type == Traj_Pos_S), total_time = 4;end
if (Traj_Type == Traj_Vol_S), total_time = 7;end
time = 0:dt:total_time; % 时间序列
step = total_time/dt + 1; % 步数

%% 初始化仿真过程的中间变量，这里根据题义，假定从0°位置零速启动
J_theta = zeros(step,1); % 记录关节实际转角数组，输出图形的纵坐标轴
J_omega = zeros(step,1); % 记录关节实际转速，输出图形的纵坐标轴
J_epsilon = zeros(step,1); % 记录关节实际加速度，输出图形的纵坐标轴
M_theta_pre = 0; % 迭代计算前，当前电机实际转角
M_theta_aft = 0; % 迭代计算前，当前电机实际转角
M_omega_pre = 0; % 迭代计算前，当前电机实际转速
M_omega_aft = 0; % 迭代计算后，当前电机实际转速
Tqm = 0; % 电机控制力矩
Tqd = 0; % 等效到电机转子的扰动力矩
Tqa = 0; % 作用于电机转子的等效力矩
Tq_f = 0; % 当前理论非线性力矩，用于力矩前馈
Uc = 0; % 控制电压
Ud = 0; % 反应干扰力矩影响的放大器输入端等效扰动电压
Uc_d = 0; % 作用于放大器的等效电压
Ue = 0; % 电机感应电动势
ia_now = 0; % 当前电枢电流
q_E = 0; % 真实位置与期望位置误差
qd_E = 0; % 真实速度与输入速度误差
global qd_E_integral; % 定义误差积分为全局变量，使PID控制器调用时可持续累加
qd_E_integral = 0;

%% 设置关节空间加速度、速度和位置期望值
%% 位置保持
if (Traj_Type == Hold_on)
    J_theta_set = zeros(step,1);% 设置所有插补点处的关节期望转角，置为零
    J_omega_set = zeros(step,1);% 设置所有插补点处的关节期望速度，置为零
    J_epsilon_set = zeros(step,1);% 设置所有插补点处的关节期望速度，置为零
end
%% 位置斜坡轨迹
if (Traj_Type == Traj_Slope), [J_epsilon_set,J_omega_set,J_theta_set] = Slope_traj(total_time,dt,pi/4,pi/4);end
%% 位置S轨迹
if (Traj_Type == Traj_Pos_S), [J_epsilon_set, J_omega_set, J_theta_set] = Position_S_traj(total_time,dt,pi/8); end
% 速度S轨迹
if (Traj_Type == Traj_Vol_S), [J_epsilon_set, J_omega_set, J_theta_set] = Velocity_S_traj(total_time,dt,pi/20);end
theta_seet_mat=[time;J_theta_set'];
save("J_theta_set.mat","theta_seet_mat","-v7.3");
omega_set_mat=[time;J_omega_set'];
save("J_omega_set.mat","omega_set_mat");
epsilon_set_mat=[time;J_epsilon_set'];
save("J_spsilon_set.mat","epsilon_set_mat");

%% 主循环，伺服指令更新周期，即精插补周期
for i = 1:step
    %% 根据单自由度机器人非线性力矩模型，计算当前理论扰动力矩，用于力矩前馈补偿
    
    %% 获得驱动空间的控制期望值
    M_theta_d = J_theta_set(i) * N; % 电机期望位置
    M_omega_d = J_omega_set(i) * N; % 电机期望速度
    M_epsilon_d = J_epsilon_set(i) * N; % 电机期望加速度
   
    %% 调用PID控制器，得到控制电压
    Tq_f = Dis_Tq * m_D * g * L_D * cos(M_theta_pre/N)/N; % 用理论值计算前馈力矩
        
    Uc = PID_Feedforward_Controller(M_theta_d, M_omega_d, M_epsilon_d, M_theta_pre, M_omega_pre, Kp, Kv, Tv, Kvff, Kaff, Ktff, Vol_Forward, Acc_Forward, Tq_Forward,Tq_f, dt, Int_Val);
    if (Uc > 10),Uc_PID = 10; end % 控制电压上限
    if (Uc < -10),Uc_PID = -10; end % 控制电压下限

    %% 更新电机角速度
    % 根据电流-力矩关系利用电机动力学模型计算电机输出转速，比通用电机模型更准确 

    if (Amp_Type == Amp_Voltage) % 速度模式电机的电流计算
        ia_now = (La/Ra)/(La/Ra + dt)*ia_now + (dt/Ra)/(La/Ra + dt)*(Uc*Ku-Ue); % 电流迭代
        Ue = Ke*M_omega_pre; % 该时刻反电动势
    end
    if(Amp_Type == Amp_Current) % 力矩模式电机的电流计算
        ia_now = Kg * Uc; % 当前电枢电流
    end
    
    
    % 根据单自由度机器人非线性力矩模型，计算当前实际扰动力矩，用于模拟干扰力矩
    Tqd =  Dis_Tq * m*g*L*cos(M_theta_pre/N)/N; % 用实际值计算干扰力矩
    
    Tqm = Ka * ia_now; % 该时刻控制力矩
    Tqa = Tqm - Tqd; % 作用于电机转子的等效合力矩
    M_omega_aft = (Im/Bm)/(Im/Bm + dt)*M_omega_pre + (dt/Bm)/(Im/Bm + dt)*Tqa; % 利用电机动力学模型，计算当前电机实际转速
   
    %% 更新关节角度、角速度和角加速度
    M_theta_aft = M_theta_pre + M_omega_aft * dt; % 电机实际角度
    J_epsilon(i) = (M_omega_aft - M_omega_pre)/dt * (1/N);% 关节实际角加速度
    J_omega(i) = M_omega_aft * (1/N); % 关节实际转速
    J_theta(i) = M_theta_aft * (1/N); % 关节实际转角
    
    %% 更新迭代前的电机实际转速和位置
    M_omega_pre = M_omega_aft; 
    M_theta_pre = M_theta_aft;

end

%% 绘图
%% 关节加速度曲线
figure(1);
yyaxis left;
% ylabel('ε(t)');
p = plot(time,J_epsilon,'k-',time,J_epsilon_set,'b--');
p(1).LineWidth=1.5;
p(2).LineWidth=1;
if (Traj_Type == Traj_Pos_S), ylim([-2,2]);end
if (Traj_Type == Traj_Vol_S), ylim([-1,1]);end
set(gca,'ycolor','k');
yyaxis right;
% ylabel('ε_e(t)');
p = plot(time, J_epsilon_set - J_epsilon, 'r-.');
p.LineWidth = 0.8;
if (Traj_Type == Traj_Pos_S), ylim([-2,6]);end
if (Traj_Type == Traj_Vol_S), ylim([-2,6]);end
set(gca,'ycolor','k');
% xlabel('t');
% title(sprintf("加速度响应曲线"));

%% 关节速度曲线
figure(2);
yyaxis left;
% ylabel('ω(t)');
p=plot(time,J_omega,'k-',time,J_omega_set,'b--');
if (Int_Val == 0) % 积分项无效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-5e-2,1e-2]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-10e-2,1e-2]);end
end
if (Int_Val == 1) % 积分项有效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-5e-2,1e-2]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-10e-2,4e-2]);end
end
p(1).LineWidth=1.5;
p(2).LineWidth=1;
set(gca,'ycolor','k');
yyaxis right;
% ylabel('ω_e(t)');
p = plot(time, J_omega_set - J_omega, 'r-.');
p.LineWidth = 0.8;
if (Int_Val == 0) % 积分项无效时的纵坐标范围
    if (Traj_Type == Hold_on  && Amp_Type == Amp_Voltage), ylim([-1e-2,5e-2]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-1e-2,10e-2]);end
end
if (Int_Val == 1) % 积分项有效时的纵坐标范围
    if (Traj_Type == Hold_on  && Amp_Type == Amp_Voltage), ylim([-1e-2,5e-2]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-5e-2,10e-2]);end
end
% if (Traj_Type == Traj_Slope), ylim([-0.8,1]);end
% if (Traj_Type == Traj_Pos_S), ylim([-0.06,0.1]);end
% if (Traj_Type == Traj_Vol_S), ylim([-0.06,0.08]);end
set(gca,'ycolor','k');
% xlabel('t');
% title(sprintf("速度响应曲线"));

%% 关节角度曲线
figure(3);
yyaxis left;
% ylabel('θ(t)');
p=plot(time,J_theta,'k-',time,J_theta_set,'b--');
if (Int_Val == 0) % 积分项无效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-8e-3,1e-3]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-8e-3,1e-3]);end
end
if (Int_Val == 1) % 积分项有效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-8e-3,1e-3]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-8e-3,1e-3]);end
end
p(1).LineWidth=1.5;
p(2).LineWidth=1;
set(gca,'ycolor','k');
yyaxis right;
% ylabel('θ_e(t)');
p = plot(time, J_theta_set - J_theta, 'r-.');
p.LineWidth = 0.8;
if (Int_Val == 0) % 积分项无效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-1e-3,8e-3]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-1e-3,8e-3]);end
end
if (Int_Val == 1) % 积分项有效时的纵坐标范围
    if (Traj_Type == Hold_on && Amp_Type == Amp_Voltage), ylim([-1e-3,8e-3]);end
    if (Traj_Type == Hold_on && Amp_Type == Amp_Current), ylim([-1e-3,8e-3]);end
end
% if (Traj_Type == Traj_Slope), ylim([-0.005,0.035]);end
% if (Traj_Type == Traj_Pos_S), ylim([-0.005,0.045]);end
% if (Traj_Type == Traj_Vol_S), ylim([-0.005,0.07]);end
set(gca,'ycolor','k');
% xlabel('t');
% title(sprintf("位置响应曲线"));