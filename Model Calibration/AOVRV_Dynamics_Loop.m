%% ------------ DYNAMICS LOOP -----------
function [x_sim, v_sim, a_sim, s] = AOVRV_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_sim, s)
    
    a_sim = zeros(length(v_lead),1);

    for t=1:length(v_lead)-1

        delta_v = v_lead(t,1) - v_sim(t,1);

        a_sim(t,1) = AOVRV_dynamics(Parameters, v_sim(t,1), s(t,1), delta_v);

        v_sim(t+1,1) = max(0, v_sim(t,1) + a_sim(t,1)*dt);
        s(t+1,1) = s(t,1) + delta_v*dt;

    end

    x_sim = x_lead - s - l;
end
%% ------------- AOVRV MODEL --------------
function acceleration = AOVRV_dynamics(Parameters, v, s, delta_v)

    k1A = Parameters(1);
    k2A = Parameters(2);
    k1D = Parameters(3);
    k2D = Parameters(4);
    eta = Parameters(5);
    tau = Parameters(6);

    if (delta_v <= 0)
        acceleration = k1D*(s - eta - tau*v) + k2D*(delta_v);
    else
        acceleration = k1A*(s - eta - tau*v) + k2A*(delta_v);
    end
    
end 