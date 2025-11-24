function [Valid_Controllers]  = create_control_sys(sys, params_cs, params, lin_points) % %Overshoot, rise time, settling time, 

%The assumption is that controllability and observability have already been
%evaluated

damping_ctl_dom= params_cs.damping_ctl_dom;
damping_ctl_sub= params_cs.damping_ctl_sub;
damping_obsv_dom= params_cs.damping_obsv_dom;
damping_obsv_sub= params_cs.damping_obsv_sub;
umax = params.umax;
ratio_K_G = params_cs.ratio_K_G;
dom_sub_ratio = params_cs.dom_sub_ratio;
dom_sub_obsv = params_cs.dom_sub_ratio;
starting_pole = params_cs.starting_pole;
A = sys.A; 
B = sys.B; 
C = sys.C; 


y0 = lin_points.y0/100;
y10 = y0(1,1); 
y20 = y0(2, 1); 
yc = params.yc/100;

u0 = lin_points.u0;

xhat_max = [y10; 0; (yc + y20 - 0.087); 0];



%-----------------------------------------------------------------------
%       Notes to improve
%-------------------------------------------------------------------
%



%disp("controller");
%best_re = inf;
%best_delta = NaN;
%best_im = NaN;
%dx = 0.25; 

%Define Constants and System params

thetadom_ctl  = acos(damping_ctl_dom);
thetasub_ctl  = acos(damping_ctl_sub);
ctl_searchspace = starting_pole;
Valid_Controllers = struct([]); 
index = 1; 


%for i = 1:numel(ctl_searchspace)
for i = 1:numel(ctl_searchspace)
    valid_K = true; 
    s_re_dom = -ctl_searchspace(i);
    if (~(damping_ctl_dom == 0 ))
        s_im_dom = tan(thetadom_ctl)*s_re_dom;
    else
        s_im_dom = 0;
    end
    s_re_sub = -ctl_searchspace(i)*dom_sub_ratio;
    if (~(damping_ctl_sub == 0) )
        s_im_sub = tan(thetasub_ctl)*abs(s_re_sub);
    else
        s_im_sub = 0;
    end
    poles_ctl = [s_re_dom + s_im_dom*1i, s_re_dom - s_im_dom*1i,  s_re_sub+ s_im_sub*1i,  s_re_sub-s_im_sub*1i];
    try
        K = place(sys.A, sys.B, poles_ctl);
    catch
        continue;
    end
    u_worst = -K*xhat_max + u0; 
    u_worst = abs(u_worst);
    if (max(u_worst)<umax && valid_K == true)
        Valid_Controllers(index).K = K; 
        Valid_Controllers(index).poles_ctl = poles_ctl; 

        index = index + 1; 
    end
end

thetadom_obs  = acos(damping_obsv_dom);
thetasub_obs =  acos(damping_obsv_sub);
invalid = false(1, numel(Valid_Controllers));


for i = 1:numel(Valid_Controllers)
    valid_G = true;
    s_re_dom = real(Valid_Controllers(i).poles_ctl(1, 1))*ratio_K_G;
    if (~(damping_obsv_dom ==0))
        s_im_dom = abs(s_re_dom)*tan(thetadom_obs);
    else 
        s_im_dom = 0;
    end
    s_re_sub = s_re_dom *dom_sub_obsv; 
    if (~(damping_obsv_sub ==0))
        s_im_sub = abs(s_re_sub)*tan(thetasub_obs);
    else
        s_im_sub = 0;
    end

    poles_obs = [s_re_dom + s_im_dom*1i, s_re_dom - s_im_dom*1i,  s_re_sub+ s_im_sub*1i,  s_re_sub-s_im_sub*1i];
    try
        G = place(sys.A', sys.C', poles_obs).';
    catch
            invalid(i) = true;
            valid_G = false; 
        continue;
    end
    if (valid_G == true)

        Valid_Controllers(i).G = G; 
        Valid_Controllers(i).poles_obs = poles_obs; 
    end
end

Valid_Controllers(invalid) = [];

if ~isfield(Valid_Controllers,'K') || isempty(Valid_Controllers.K) || ...
   ~isfield(Valid_Controllers,'G') || isempty(Valid_Controllers.G)

    Valid_Controllers = struct( ...
        'K', NaN, ...
        'G', NaN, ...
        'poles_ctl', [], ...
        'poles_obs', [] ...
    );
end
Acl =  A - B*K;
M = C*(Acl\B);    % = C * (A-BK)^(-1) * B
Kr = inv(M);      % final 2x2 feedforward gain

Valid_Controllers.Kr = Kr; 
 


%}
end