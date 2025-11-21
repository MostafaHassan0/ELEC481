function [G, K, poles_controller, poles_observer ]  = create_control_sys(sys, params_cs) % %Overshoot, rise time, settling time, 

%The assumption is that controllability and observability have already been
%evaluated

Per_OS = params_cs.Per_OS; 
tp =params_cs.tp;
ts = params_cs.ts;
ratio_K_G = params_cs.ratio_K_G;
dom_sub_ratio = params_cs.dom_sub_ratio;
%zeta_min_ratio = params_cs.zeta_min_ratio;

safety_factor = params_cs.safety_factor; 

%-----------------------------------------------------------------------
%       Notes to improve
%-------------------------------------------------------------------
%

%A 2ND order pole approximation will  be used as the the goal of the
%control system is to make sure R(p1), R(p2) >> R(p3), R(p4) 
%where p1, and p2 are the dominant poles and p1 -> p4 are poles after
%shifted
if(Per_OS >1)
    disp("Per_OS must be between 0 -> 1")
    return; 
end

disp("controller");
%best_re = inf;
%best_delta = NaN;
%best_im = NaN;
%dx = 0.25; 

%Define Constants and System params

damping_ratio_max = - log(Per_OS)/(sqrt(pi^2+(log(Per_OS))^2)); 

theta = acos(damping_ratio_max);

%damping_ratio_min = zeta_min_ratio* damping_ratio_max_max;


%Controller 


%damping_range = damping_ratio_min:0.01:damping_ratio_max;

s_Im_boundary_ctl = pi/(tp*safety_factor);

s_Re_boundary_ctl = 4/(ts*safety_factor);

x_intersection_ctl = s_Im_boundary_ctl / tan(theta) ;  % x where theta meets imaginary boundary 


if abs(s_Re_boundary_ctl) <= abs(x_intersection_ctl)
    % -------- Case A: Ts line right of intersection -> wedge is active --------
    best_im = s_Im_boundary_ctl ;
    best_re = -x_intersection_ctl;

    disp("Case: CONTROLLER – Wedge boundary dominates (intersection point)");

    %{
    x_min = s_Re_boundary;
    x_max = x_intersection_ctl;
 
    for x = x_min:dx:x_max
        S_Re = -x ;  % actual negative real part
        S_Im_limit = x * tan(theta);

        for delta = damping_range
            wn = x / delta;
            wd = wn * sqrt(1 - delta^2) ; % imag part

            % wedge (overshoot) is the active limit here
            if wd <= S_Im_limit
                % choose pole closest to imag axis (smallest |Re|)
                if abs(x) <= abs(best_re)
                    best_re    = S_Re
                    best_delta = delta
                    best_im    = wd
                end
            end
        end
    end
    %}
    
else
    % -------- Case B: Ts line left of intersection -> tp is active --------

    best_im = s_Im_boundary_ctl ;
    best_re = -s_Re_boundary_ctl; 

    disp("Case: CONTROLLER – Peak-time boundary dominates (ts line)");
    
    %{
    x_min = x_intersection_ctl;

    x_max = x_intersection_ctl+10/dx; 

    
    for x = x_min:dx:x_max
        S_Re = -x;

        for delta = damping_range
            wn = x / delta;
            wd = wn * sqrt(1 - delta^2);

            % tp (horizontal) is the active limit here
            if wd <= s_Im_boundary_ctl
                if abs(x) <= abs(best_re)
                    best_re    = S_Re;
                    best_delta = delta;
                    best_im    = wd;
                end
            end
        end
    end
    %}
end

wn_ctl = sqrt(best_re^2 + best_im^2);
zeta_ctl = abs(best_re) / wn_ctl;

fprintf("Selected Dominant Pole (Controller):  Re = %.4f,  Im = %.4f\n", best_re, best_im);
fprintf("Damping Ratio (Controller):           %.4f\n", zeta_ctl);
fprintf("Natural Frequency wn (Controller):    %.4f rad/s\n\n", wn_ctl);


%disp(best_delta)

    %if isnan(best_delta)
    %error("No valid poles found for δ in [0.75,1] that satisfy Im boundary controller.");
    %end

%S_Re = -max(abs(s_Re_eval1), abs(s_Re_boundary ));

poles_controller = [best_re + 1i*best_im  , best_re - 1i*best_im , dom_sub_ratio*best_re, dom_sub_ratio*best_re ]; 

K = place(sys.A, sys.B, poles_controller); 

ts_observer = ts/ratio_K_G; 
tp_observer= tp/ratio_K_G; 

%best_delta = NaN;

%best_re = inf;

%best_im = NaN;


%Observer

s_Im_boundary_obs = pi/tp_observer;

s_Re_boundary_obs = 4/ts_observer ;

x_intersection_obs = s_Im_boundary_obs / tan(theta) ;

% -------- Display Observer Case --------
disp("---- Observer Pole Selection ----");

if abs(s_Re_boundary_obs) <= abs(x_intersection_obs)
    disp("Case: OBSERVER – Wedge boundary dominates (intersection point)");
else
    disp("Case: OBSERVER – Peak-time boundary dominates (ts line)");
end

% Compute damping ratio
wn_obs = sqrt(best_re^2 + best_im^2);
zeta_obs = abs(best_re) / wn_obs;

fprintf("Selected Dominant Pole (Observer):    Re = %.4f,  Im = %.4f\n", best_re, best_im);
fprintf("Damping Ratio (Observer):             %.4f\n", zeta_obs);
fprintf("Natural Frequency wn (Observer):      %.4f rad/s\n\n", wn_obs);


if abs(s_Im_boundary_obs) <= abs(x_intersection_obs)
    % -------- Case A: Ts line right of intersection -> wedge active --------

    best_im = s_Im_boundary_obs ;
    best_re = -x_intersection_obs;

    disp("Case: OBSERVER – Wedge boundary dominates (intersection point)");


    %{
    x_min = s_Re_boundary_obs;
    x_max = x_intersection_obs;
    
    for x = x_min:dx:x_max
        S_Re = -x;   % actual negative real part

        for delta = damping_range
            wn = x / delta;
            wd = wn * sqrt(1 - delta^2);  % imag part

            % wedge (overshoot) is the active limit here
            if wd <= x * tan(theta)
                % choose pole closest to imag axis (smallest |Re|)
                if abs(x) <= abs(best_re)
                    best_re    = S_Re
                    best_delta = delta
                    best_im    = wd
                end
            end
        end
    end
    %}
    
else
    % -------- Case B: Ts line left of intersection -> tp is active --------

    best_im = s_Im_boundary_obs ;
    best_re = -s_Re_boundary_obs; 

    disp("Case: OBSERVER – Peak-time boundary dominates (ts line)");


    %{
    x_min = x_intersection_obs;
    x_max = x_intersection_obs+10/dx; 
    
    for x = x_min:dx:x_max
        S_Re = -x;

        for delta = damping_range
            wn = x / delta;
            wd = wn * sqrt(1 - delta^2);

            % tp (horizontal) is the active limit here
            if wd <= s_Im_boundary_obs
                if abs(x) <= abs(best_re)
                    best_re    = S_Re
                    best_delta = delta
                    best_im    = wd
                end
            end
        end
    end
    %}
end

wn_obs = sqrt(best_re^2 + best_im^2);
zeta_obs = abs(best_re) / wn_obs;

fprintf("Selected Dominant Pole (Observer):    Re = %.4f,  Im = %.4f\n", best_re, best_im);
fprintf("Damping Ratio (Observer):             %.4f\n", zeta_obs);
fprintf("Natural Frequency wn (Observer):      %.4f rad/s\n\n", wn_obs);

%end

%if isnan(best_delta)
%error("No valid poles found for δ in [0.75,1] that satisfy Im boundary observer.");
%end

poles_observer = [best_re + 1i*best_im , best_re - 1i*best_im, dom_sub_ratio*best_re, dom_sub_ratio*best_re ]; 

G = place(sys.A', sys.C', poles_observer).';




end