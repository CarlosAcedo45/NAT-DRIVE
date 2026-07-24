%% ------------ DYNAMICS LOOP -----------
function [x_sim, v_sim, a_sim, s] = OVRV_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_sim, s)
    
    a_sim = zeros(length(v_lead),1);

    for t=1:length(v_lead)-1

        delta_v = v_lead(t,1) - v_sim(t,1);

        a_sim(t,1) = OVRV_dynamics(Parameters, v_sim(t,1), s(t,1), delta_v);

        v_sim(t+1,1) = max(0, v_sim(t,1) + a_sim(t,1)*dt);
        s(t+1,1) = s(t,1) + delta_v*dt;

    end

    x_sim = x_lead - s - l;
end
%% ------------- OVRV MODEL --------------
function acceleration = OVRV_dynamics(Parameters, v, s, delta_v)

    k1 = Parameters(1);
    k2 = Parameters(2);
    eta = Parameters(3);
    tau = Parameters(4);

    acceleration = k1*(s - eta - tau*v) + k2*(delta_v);
    
end 
