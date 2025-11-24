function Valid_Controllers = create_control_sys_LQR(sys, search_params)

r = search_params.r;                      % scalar (log domain OK)

% Observer pole angles (in radians)
damping_obsv_dom = search_params.damping_obsv_dom;   % dominant pole angle
damping_obsv_sub = search_params.damping_obsv_sub;   % subdominant pole angle

% Additional observer parameter
dom_sub = search_params.dom_sub_ratio;        % ratio of dominant to subdominant real axis speeds

% Controller–observer relative speed scaling
ratio_K_G = search_params.ratio_K_G;         % makes observer faster or slower than controller

A = sys.A;
B = sys.B;
C = sys.C;

% -------------------------
% 1) LQR Controller K
% -------------------------
Q = C' * C;
r = search_params.r;
R = r * eye(size(B,2));

try
    K = lqr(A, B, Q, R);
catch
    Valid_Controllers = struct('K',NaN, 'G',NaN);
    return;
end


% -------------------------
% 2) Observer G via pole placement
% -------------------------
thetadom_obs = acos(damping_obsv_dom);   % radians
thetasub_obs = acos(damping_obsv_sub);   % radians
% --- Controller closed-loop poles ---

poles_ctl = eig(A - B*K);

% Fastest controller pole magnitude (most negative real part)
fastest_ctl = max(abs(real(poles_ctl)));

% --- Observer dominant real part ---
dom_obsv = ratio_K_G * fastest_ctl;

% --- Observer real parts ---
sigma_dom = -dom_obsv;
sigma_sub = -dom_obsv / dom_sub;

% --- Imaginary parts from BO-tuned angles ---
omega_dom = abs(sigma_dom) * tan(thetadom_obs);
omega_sub = abs(sigma_sub) * tan(thetasub_obs);

% --- Complex conjugate pole pairs ---
lambda1 = sigma_dom + 1j * omega_dom;
lambda2 = sigma_dom - 1j * omega_dom;
lambda3 = sigma_sub + 1j * omega_sub;
lambda4 = sigma_sub - 1j * omega_sub;

poles_obsv = [lambda1, lambda2, lambda3, lambda4];


try
    G = place(A', C', poles_obsv)';   % transposed for observer
catch
    Valid_Controllers = struct('K',NaN,'G',NaN);
    return;
end

% Output
Valid_Controllers.K = K;
Valid_Controllers.G = G;
Valid_Controllers.poles_ctl = poles_ctl;
Valid_Controllers.poles_obs = poles_obsv;

end
