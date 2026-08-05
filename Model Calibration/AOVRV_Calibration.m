function Results = AOVRV_Calibration(Delta, l, leader, follower, s_real)
    %% ----------- INITIAL DATA AND PARAMETERS ----------
    
    Results = struct();
    Results.Opt_Param = [];
    Results.x = [];
    Results.v = [];
    Results.a = [];
    Results.s = [];
    Results.RMSE_s = [];
    Results.RMSE_v = [];

    %parameters of AOVRV (Shang et al.), AVs
    k1A = 0.015;    % spacing control gain when accelerating
    k2A = 0.091;    % relative speed control gain when accelerating
    k1D = 0.029;    % spacing control gain when breaking
    k2D = 0.196;    % relative speed control gain when decelerating
    eta = 21.51;    % jam distance
    tau = 1.71;     % effective time-gap
    
    % Initialize simulation states
    initial_states.s = s_real(1);
    initial_states.v = follower.v(1);
    
    % Initial OVRV parameters and bounds
    AOVRV_Parameters =  [k1A, k2A, k1D, k2D, eta, tau];
    L_bound = [1e-3, 1e-3, 1e-3, 1e-3, 2, 0.1];
    U_bound = [1, 1, 1, 1, 30.0, 10.0];
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
        'x0', AOVRV_Parameters, ...   % your original guess (used as one of the starts)
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
    fprintf('\n------------ AOVRV Optimal Parameters ------------\n')
    for i = 1:length(solutions)
        fprintf('Run %d: k1A=%.3f k2A=%.3f k1D=%.3f k2D=%.3f eta=%.3f tau=%.3f | error=%.4f\n', ...
            i, solutions(i).X(1), solutions(i).X(2), ...
               solutions(i).X(3), solutions(i).X(4), ...
               solutions(i).X(5), solutions(i).X(6), ...
               solutions(i).Fval);
    end
    %}
    
    %% ------------- RMSE CALCULATION -------------
    
    [sim_Opt.x, sim_Opt.v, sim_Opt.a, sim_Opt.s] = AOVRV_Dynamics_Loop(Opt_Param, Delta, l, leader.x, leader.v, ...
                                                                initial_states.v, initial_states.s);

    RMSE_speed_Opt = rmse(sim_Opt.v, follower.v);
    RMSE_spacing_Opt = rmse(sim_Opt.s, s_real);

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
    [~,~,~,s_sim] = AOVRV_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_follower, s_real);

    c = s_min - min(s_sim);
    ceq = [];

end
%% ------------ RMSE FUNCTION -----------
function error = RMSE(Parameters, dt, l, x_lead, v_lead, v_follower, s_real)
    [~,~,~,s_sim] = AOVRV_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_follower(1), s_real(1));
    error = rmse(s_sim, s_real);
end

