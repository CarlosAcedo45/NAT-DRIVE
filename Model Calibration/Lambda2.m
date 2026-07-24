function [Lambda2, stable] = Lambda2(Parameters, s_eq, v_eq, Model)
    switch Model
        case 'IDM'
            v0 = Parameters(1);
            T = Parameters(2);
            s0 = Parameters(3);
            ldelta = Parameters(4);
            a = Parameters(5);
            b = Parameters(6);

            f_s = (2*a*(s0 + T*v_eq)^2) / (s_eq^3);
            f_v = ((-ldelta*a*(v_eq/v0)^(ldelta-1))/v0) - ((2*a*T*(s0+T*v_eq)) / (s_eq^2));
            f_DeltaV = (a*v_eq*(s0+T*v_eq)) / ((s_eq^2)*sqrt(a*b)); 
            [Lambda2, stable] = Lambda2_Condition(f_s, f_v, f_DeltaV);

        case 'OVRV'
            k1 = Parameters(1);
            k2 = Parameters(2);
            eta = Parameters(3);
            tau = Parameters(4);

            f_s = k1; 
            f_v = -k1*tau;
            f_DeltaV = k2;
            [Lambda2, stable] = Lambda2_Condition(f_s, f_v, f_DeltaV);

        otherwise
            error('Invalid model specified. Please choose either ''IDM'' or ''OVRV''.');
    end
end

function [Lambda2, stable] = Lambda2_Condition(f_s, f_v, f_DeltaV)

    Lambda2 = (f_s/(f_v^3)) * ((f_v^2)/2 - f_DeltaV*f_v - f_s);

% Indicate instability if Lambda2 is negative acording to R.E. Wilson 2011
    stable = Lambda2 < 0;
end