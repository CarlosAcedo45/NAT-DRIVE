% All model calibrations from same data
function [IDM_Param, OVRV_Param, AOVRV_Param] = All_Models_Calibration(file_leader, file_follower, display, save)
    
    % Final data_EV array shape =>
    % [index, Position, Speed, Acceleration]
    data_EV = readmatrix(file_leader, NumHeaderLines=1);
    data_EV(:,[2,3,4]) = [];
    data_EV(:,3) = data_EV(:,3)/3.6;
    
    % Final data_follower array shape =>
    % [index_zeroed, elapsed_Time, range_kf, velocity_kf, acceleration_kf]
    data_follower = readmatrix(file_follower, NumHeaderLines = 1);
    data_follower(:,[1,4,5,6,10,11]) = [];
    % Init_v = data(1,3);
    fclose("all");
    
    %% ----------- INITIAL DATA ----------
    Tt = data_follower(:,2);    % Time table
    Delta = 0.04;               % Time step
    l = 3;                      % vehicle length
    
    % Define leader vehicle states
    leader.x = data_EV(:,2);
    leader.v = data_EV(:,3);
    leader.a = data_EV(:,4);
    
    s_real = data_follower(:,3);
    deltaV_real = data_follower(:,4);
    
    % Calculate follower vehicle' states based on Radar readings
    follower.x = leader.x - s_real - l;
    follower.v = leader.v - deltaV_real;
    
    % Initialize simulation states
    initial_states.s = s_real(1);
    initial_states.v = follower.v(1);
    
    %% ----------- PARAMETERS AND STATES -------------
    
    IDM_Results = IDM_Calibration_V1(Delta, l, leader, follower, s_real);
    OVRV_Results = OVRV_Calibration_V1(Delta, l, leader, follower, s_real);
    AOVRV_Results = AOVRV_Calibration_V1(Delta, l, leader, follower, s_real);

    IDM_Param = IDM_Results.Opt_Param;
    OVRV_Param = OVRV_Results.Opt_Param;
    AOVRV_Param = AOVRV_Results.Opt_Param;
    
    %% -------------- RESULTS PLOTTED --------------
    if display == true
    
        RGB = orderedcolors("gem");
    
        figure(1);
        hold on
        xlabel('Time (s)');
        ylabel('Position (m)');
        xlim([0, Tt(end)]);
        ylim([0, round(max(leader.x)+50, 1, 'significant')]);
        plot(Tt,leader.x, 'LineWidth',2);
        plot(Tt,follower.x, 'LineWidth',2, 'LineStyle','-');
        plot(Tt,IDM_Results.x,'LineWidth',2, 'LineStyle',':');
        plot(Tt,OVRV_Results.x, 'LineWidth',2, 'LineStyle','-.');
        plot(Tt,AOVRV_Results.x,'LineWidth',2, 'LineStyle','--');
        legend({'Leader', 'Follower', 'IDM', 'OVRV', 'AOVRV'}, 'FontWeight','bold', 'Location','southeast');
        theme('light');
        set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
        fontsize(12,"points");
        set(1, 'Position', [100 100 560 420]);
        grid on
        hold off
        
        figure(2);
        hold on
        xlabel('Time (s)');
        ylabel('Speed (m/s)');
        xlim([0, Tt(end)]);
        ylim([0, round(max(leader.v)+5)]);
        plot(Tt,leader.v, 'LineWidth',2);
        plot(Tt,follower.v, 'LineWidth',2, 'LineStyle','-');
        plot(Tt,IDM_Results.v,'LineWidth',2, 'LineStyle',':');
        plot(Tt,OVRV_Results.v, 'LineWidth',2, 'LineStyle','-.');
        plot(Tt,AOVRV_Results.v,'LineWidth',2, 'LineStyle','--');
        legend({'Leader', 'Follower', 'IDM', 'OVRV', 'AOVRV'},'FontWeight','bold','Location','northeast');
        theme('light');
        set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
        fontsize(12,"points");
        set(2, 'Position', [100 100 560 420]);
        grid on
        hold off
    
        figure(3);
        hold on;
        xlabel('Time (s)');
        ylabel('Spacing (m)');
        xlim([0, Tt(end)]);
        ylim([0, round(max(s_real)+5)]);
        plot(Tt,s_real, 'LineWidth',2, 'Color', RGB(2,:));
        plot(Tt,IDM_Results.s,'LineWidth',2, 'LineStyle',':','Color', RGB(3,:));
        plot(Tt,OVRV_Results.s, 'LineWidth',2, 'LineStyle','-.', 'Color', RGB(4,:));
        plot(Tt,AOVRV_Results.s,'LineWidth',2, 'LineStyle','--', 'Color', RGB(5,:));
        legend({'Real', 'IDM', 'OVRV', 'AOVRV'}, 'FontWeight','bold');
        theme('light');
        set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
        fontsize(12,"points");
        set(3, 'Position', [100 100 560 420]);
        grid on
        hold off
    
        figure(4);
        hold on;
        xlabel('Time (s)');
        ylabel('Acceleration (m/s2)');
        xlim([0, Tt(end)]);
        ylim([-5, 5]);
        plot(Tt,leader.a, 'LineWidth',2);
        plot(Tt,IDM_Results.a,'LineWidth',2, 'LineStyle',':', 'Color', RGB(3,:));
        plot(Tt,OVRV_Results.a, 'LineWidth',2, 'LineStyle','-.','Color', RGB(4,:));
        plot(Tt,AOVRV_Results.a,'LineWidth',2, 'LineStyle','--', 'Color', RGB(5,:));
        legend({'Leader', 'IDM', 'OVRV', 'AOVRV'}, 'FontWeight','bold');
        theme('light');
        set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
        fontsize(12,"points");
        set(4, 'Position', [100 100 560 420]);
        grid on;
        hold off;
    
        % figure(5);
        % hold on;
        % xlabel('Spacing (m)');
        % ylabel('Acceleration (m/s2)');
        % xlim([0, round(max(s_real)+5, 2, 'significant')]);
        % ylim([-2, 2]);
        % % plot(s_real,leader.a, 'LineWidth',2);
        % plot(IDM_Results.s,IDM_Results.a,'LineWidth',2, 'LineStyle',':', 'Color', RGB(3,:));
        % plot(OVRV_Results.s,OVRV_Results.a, 'LineWidth',2, 'LineStyle','-.','Color', RGB(4,:));
        % plot(AOVRV_Results.s,AOVRV_Results.a,'LineWidth',2, 'LineStyle','--', 'Color', RGB(5,:));
        % legend({'IDM', 'OVRV', 'AOVRV'}, 'FontWeight','bold');
        % theme('light');
        % set(findall(gcf, '-property', 'FontWeight'), 'FontWeight', 'bold');
        % fontsize(12,"points");
        % set(5, 'Position', [100 100 560 420]);
        % grid on;
        % hold off;
    
        %% --------- Save Figures ----------
        if save == true
            saveas(1, 'Figures\I-10V2_Calibration_Position', 'png');
            saveas(2, 'Figures\I-10V2_Calibration_Speed', 'png');
            saveas(3, 'Figures\I-10V2_Calibration_Spacing', 'png');
            saveas(4, 'Figures\I-10V2_Calibration_Acceleration', 'png');
        end 
    end
end
