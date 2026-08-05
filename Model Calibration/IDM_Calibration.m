function Results = IDM_Calibration(Delta, l, leader, follower, s_real)
    %% ----------- INITIAL DATA AND PARAMETERS ----------
    
    Results = struct();
    Results.Opt_Param = [];
    Results.x = [];
    Results.v = [];
    Results.a = [];
    Results.s = [];
    Results.RMSE_s = [];
    Results.RMSE_v = [];


    %Table 1 Parameters
    %parameters of IDM, HVs
    v_0 = 20;   %m/s, desired speed
    a = 1;    %maximum acceleration
    b = 1.5;    %comfortable deceleration  
    T = 1.5;    %desired time gap
    s_0 = 2;    %jam distance
    ldelta = 4; %Speed ratio exponent

    % Initialize simulation states
    initial_states.s = s_real(1);
    initial_states.v = follower.v(1);

    % Initial IDM parameters and bounds
    IDM_Parameters = [v_0, T, s_0, ldelta, a, b];
    L_bound = [16.5, 0.1, 2, 0, 0.1, 2];
    U_bound = [40, 10, 30, 17, 5, 9];
    s_min = 0.5;
    
    Options = optimoptions('fmincon', ...
            'Display', 'off', ...
            'MaxFunctionEvaluations', 5000, ...
            'MaxIterations', 1000);
    
    error_function = @(x) RMSE(x, Delta, l, leader.x, leader.v, follower.v, s_real);
    nonlcon = @(x) collision_constraint(x, Delta, l, leader.x, leader.v, follower.v, s_real, s_min);
    
    %% ------------ OPTIMAL PROBLEM SOLVER -----------
    
    % Step 1: Repackage your problem into a struct lsqnonlin understands
    problem = createOptimProblem('fmincon', ...
        'objective', error_function, ...
        'x0', IDM_Parameters, ...   % your original guess (used as one of the starts)
        'lb', L_bound, ...
        'ub', U_bound, ...
        'options', Options,...
        'nonlcon', nonlcon);
    
    % Step 2: Create the MultiStart solver
    ms = MultiStart();
    rng(2026)
    % Step 3: Run it with N random starting points
    % Each start picks random [k1,k2,eta,tau] within your bounds and runs lsqnonlin
    [Opt_Param, error_val, exitflag, output, solutions] = run(ms, problem, 20);
    
    % 'solutions' contains ALL converged runs — useful for diagnostics
    % You can inspect how spread out the solutions are:
    fprintf('\n------------ IDM Optimal Parameters ------------\n')
    for i = 1:length(solutions)
        fprintf('Run %d: v0=%.3f T=%.3f s0=%.3f ldelta=%.3f a=%.3f b=%.3f | error=%.4f\n', ...
            i, solutions(i).X(1), solutions(i).X(2), ...
               solutions(i).X(3), solutions(i).X(4), ...
               solutions(i).X(5), solutions(i).X(6), ...
               solutions(i).Fval);
    end
    %}
    
    %% ------------- RMSE CALCULATION -------------
    
    [sim_Opt.x, sim_Opt.v, sim_Opt.a, sim_Opt.s] = IDM_Dynamics_Loop(Opt_Param, Delta, l , leader.x, leader.v, ...
                                                                initial_states.v, initial_states.s);
    
    RMSE_speed_Opt = rmse(sim_Opt.v, follower.v);  
    RMSE_spacing_Opt = rmse(sim_Opt.s, s_real);

    %% ----------------- RESULTS -----------------

    Results.Opt_Param = Opt_Param;
    Results.x = sim_Opt.x;
    Results.v = sim_Opt.v;
    Results.a = sim_Opt.a;
    Results.s = sim_Opt.s;
    Results.RMSE_s = RMSE_spacing_Opt;
    Results.RMSE_v = RMSE_speed_Opt;

end
%% ------- COLLISION CONSTRAINT ---------
function [c, ceq] = collision_constraint(Parameters, dt, l, x_lead, v_lead, v_follower, s_real, s_min)
    [~,~,~,s_sim] = IDM_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_follower, s_real);

    c = s_min - min(s_sim);
    ceq = [];

end
%% ------------ RMSE FUNCTION -----------
function error = RMSE(Parameters, dt, l, x_lead, v_lead, v_follower, s_real)
    [~,~,~,s_sim] = IDM_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_follower(1), s_real(1));
    error = rmse(s_sim, s_real);
end

