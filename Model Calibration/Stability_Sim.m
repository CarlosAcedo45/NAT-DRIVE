% Simulations utilizing Calibrated results.
% This is to visualize and present string stability from the recorded
% driving behaviors.
% Refere to All_Calibrations code to get calibrated results for IDM, OVRV,
% and AOVRV models. 

% Simulation setup:
% Use a string of 10 vehicles plus leader. 
% Leader follows real oscillatory trajectory from Racebox.
% Plot Speed v Time

clc, clear all, close all;
display = true;
save = false;

%% -------- LEADER DATA -------- 
Leader_Data = readtable('Data/Oscillations.csv', VariableNamingRule='preserve');
leader.x = table2array(Leader_Data(:,2));
leader.v = table2array(Leader_Data(:,3));
leader.a = table2array(Leader_Data(:,4));
Tt = table2array(Leader_Data(:,1));
t_segments = length(Tt);

v_eq = leader.v(1);
Delta = 0.04;
l = 5;

%% ------- CALIBRATION ---------
% Racebox data synchronized to Radar => 
% [index, Time, Latitude, Longitude, Position, Speed, Acceleration]
file_speedProf = 'Data/Mesa-Sync_EV_Follower1_data_V2 10-04.csv';
% file_speedProf = 'Data/Mesa-Sync_ICE_Follower1_data 12-43.csv';
% file_speedProf = 'Data/I10-Sync_EV_Follower2_data 13-14.csv';

% Radar data file => 
% [index_org, index_zeroed, elapsed_Time, Timestamp, range_raw, velocity_raw,
% range_kf, velocity_kf, acceleration_kf, mahalanobis, kf_update_acepted]
file_follower = 'Data/Mesa_2_KF_Final_Rep_Removed.csv';
% file_follower = 'Data/Mesa_8_KF_Final_Rep_Removed.csv';
% file_follower = 'Data/I10_SecondVeh_Smoothed_June17.csv';

[IDM_Param, OVRV_Param, AOVRV_Param] = All_Models_Calibration(file_speedProf, file_follower, false, false);

%% ------------ PLATOON SIMULATION -----------
s_eq_IDM = (IDM_Param(3)+v_eq*IDM_Param(2))/sqrt(1-(v_eq/IDM_Param(1))^IDM_Param(4));
s_eq_OVRV = OVRV_Param(3) + OVRV_Param(4) * v_eq;
s_eq_AOVRV = AOVRV_Param(5) + AOVRV_Param(6) * v_eq;

IDM_Platoon(1).x = leader.x;
IDM_Platoon(1).v = leader.v;
OVRV_Platoon(1).x = leader.x;
OVRV_Platoon(1).v = leader.v;
AOVRV_Platoon(1).x = leader.x;
AOVRV_Platoon(1).v = leader.v;

for i = 1:10
    [IDM_Platoon(i+1).x, IDM_Platoon(i+1).v, ~, IDM_Platoon(i+1).s] = IDM_Dynamics_Loop(IDM_Param, Delta, l, ...
                                            IDM_Platoon(i).x, IDM_Platoon(i).v, ...
                                            v_eq, s_eq_IDM);

    [OVRV_Platoon(i+1).x, OVRV_Platoon(i+1).v, ~, ~] = OVRV_Dynamics_Loop(OVRV_Param, Delta, l, ...
                                            OVRV_Platoon(i).x, OVRV_Platoon(i).v, ...
                                            v_eq, s_eq_OVRV);

    [AOVRV_Platoon(i+1).x, AOVRV_Platoon(i+1).v, AOVRV_Platoon(i+1).a, AOVRV_Platoon(i+1).s] = AOVRV_Dynamics_Loop(AOVRV_Param, Delta, l, ...
                                            AOVRV_Platoon(i).x, AOVRV_Platoon(i).v, ...
                                            v_eq, s_eq_AOVRV);
end

%%
[IDM_Lambda2, IDM_Stable] = Lambda2(IDM_Param, s_eq_IDM, v_eq, 'IDM');
[OVRV_Lambda2, OVRV_Stable] = Lambda2(OVRV_Param, s_eq_OVRV, v_eq, 'OVRV');
[gD, gA, L_bound, AOVRV_D_Stable, AOVRV_A_Stable] = g_condition(AOVRV_Param);

%% ----------- PLOTS -------------
if display == true
    close all;

    figure(1)
    hold on;
    title('IDM')
    ylim([0, 40])
    xlim([0, max(Tt)])
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    plot(Tt, IDM_Platoon(1).v, 'LineWidth',2);
    for i = 2:11
        plot(Tt, IDM_Platoon(i).v);
    end
    grid on;
    legend('Leader');
    theme('light');
    set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
    fontsize(12,"points");
    set(1, 'Position', [100 100 560 420]);
    
    figure(2)
    hold on;
    title('OVRV')
    ylim([0, 40])
    xlim([0, max(Tt)])
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    plot(Tt, OVRV_Platoon(1).v, 'LineWidth',2);
    for i = 2:11
        plot(Tt, OVRV_Platoon(i).v);
    end
    grid on;
    legend('Leader');
    theme('light');
    set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
    fontsize(12,"points");
    set(2, 'Position', [100 100 560 420]);
    
    figure(3)
    hold on;
    title('AOVRV')
    ylim([0, 40])
    xlim([0, max(Tt)])
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    plot(Tt, AOVRV_Platoon(1).v, 'LineWidth',2);
    for i = 2:11
        plot(Tt, AOVRV_Platoon(i).v);
    end
    grid on;
    legend('Leader');
    theme('light');
    set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
    fontsize(12,"points");
    set(3, 'Position', [100 100 560 420]);

    % figure(4)
    % hold on;
    % title('Spacing IDM')
    % ylim([0, 40])
    % xlim([0, max(Tt)])
    % xlabel('Time (s)');
    % ylabel('acceleration (m/s^2)');
    % % plot(Tt, AOVRV_Platoon(1).a, 'LineWidth',2);
    % for i = 2:11
    %     plot(Tt, IDM_Platoon(i).s);
    % end
    % grid on;
    % legend('Leader');
    % theme('light');
    % set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
    % fontsize(12,"points");
    % set(4, 'Position', [100 100 560 420]);
end 

%% ---------- SAVE IMAGES -----------
if save == true
    saveas(1, 'Figures\IDM_FollowC_Oscillations', 'png');
    saveas(2, 'Figures\OVRV_FollowC_Oscillations', 'png');
    saveas(3, 'Figures\AOVRV_FollowC_Oscillations', 'png');
end

function [gD, gA, L_bound, D_Stable, A_Stable] = g_condition(Parameters)
    k1A = Parameters(1);
    k2A = Parameters(2);
    k1D = Parameters(3);
    k2D = Parameters(4);
    eta = Parameters(5);
    tau = Parameters(6);

    gD = (k1D*tau)/2 + k2D;
    gA = (k1A*tau)/2 + k2A;
    L_bound = 1/tau;

    D_Stable = gD > L_bound;
    A_Stable = gA > L_bound;

end