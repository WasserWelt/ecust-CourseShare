%% PID控制器，包含速度、加速度和非线性力矩前馈，适用于多关节情况
function Uc_PID = PID_Feedforward_Controller(theta_d, omega_d, epsilon_d, theta, omega, Kp, Kv, Tv, Kvff, Kaff, Ktff, Vol_For_A, Acc_For_A, Tq_For_A,Tq_dd, dt, Int_Val)
    %% 常规PID控制器控制电压，包含无增益速度前馈
    global qd_E_integral;
    q_E = theta_d - theta; % 位置误差
    q_E = Kp * q_E ; % 位置比例环节
    qd_E = Vol_For_A .* omega_d + q_E - omega; % 无增益速度前馈+速度反馈
    qd_E_integral =Int_Val * (qd_E_integral + Kv * qd_E .* dt); % 积分项，Int_Val=1有效，Int_Val=0无效
    Uc_PID = Kv*Tv*qd_E + qd_E_integral;   
    %% 计算有增益速度、加速度和非线性力矩前馈控制电压
    Uc_vff = Vol_For_A .* Kvff * omega_d;
    Uc_aff = Acc_For_A .* Kaff * epsilon_d;
    Uc_tff = Tq_For_A .* Ktff * Tq_dd;
    %% 输出总控制电压，并根据驱动器阈值做限幅
    Uc_PID = Uc_PID + Uc_vff + Uc_aff + Uc_tff; % 总控制电压
end