%% ------------ DYNAMICS LOOP -----------
function [x_sim, v_sim, a_sim, s] = IDM_Dynamics_Loop(Parameters, dt, l, x_lead, v_lead, v_sim, s)

    a_sim = zeros(length(v_lead),1);

    for t=1:length(v_lead)-1

        delta_v = v_lead(t,1) - v_sim(t,1);

        a_sim(t,1) = IDM_dynamics(Parameters, v_sim(t,1), s(t,1), delta_v);

        v_sim(t+1,1) = max(0, v_sim(t,1) + a_sim(t,1)*dt);
        s(t+1,1) = s(t,1) + delta_v*dt;

    end

    x_sim = x_lead - s - l;
end

%% ------------- IDM MODEL --------------
function acceleration = IDM_dynamics(Parameters, v, s, delta_v)

    v0 = Parameters(1);
    T = Parameters(2);
    s0= Parameters(3);
    ldelta = Parameters(4);
    a = Parameters(5);
    b = Parameters(6);

    s_star = s0 + max(0,T*v - (v*(delta_v))/(2*sqrt(a*b)));
    acceleration = a*(1 - (v/v0)^ldelta - (s_star/(s))^2);
end 