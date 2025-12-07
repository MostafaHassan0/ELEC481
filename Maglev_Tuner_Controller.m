function Performance = Maglev_Tuner_Controller(search_params, params)


%*************************************************************************

% The simulator code follows

%----------------------------------------------------------------------
%     System Params - You shouldn't change these
%---------------------------------------------------------------------

m = params.m; %kg
a = params.a; %counts / N*cm^4 -> cm^4 terms cancel with cm .^n within force cals
b = params.b;  %cm 
c = params.c; %N *cm^4
d = params.d; %cm 
yc = params.yc;  %cm 
g = params.g; %m/s^2
n = params.n; %exponent unitless
drag = params.drag; % N·s/m  (viscous)

%drag = 2.5e-1; %drag maximum
min_gap = params.min_gap; %cm, break condition for simulation
e = params.e; 
umax = params.umax; 


%--------------------------------------------------------------------
%       Sim Params - These are configured by user
%--------------------------------------------------------------------

%Note this sim does a lot of computation, and stores a lot of data
%Depending on your PC you can crash Matlab, or your PC by choosing an
%extremely small Ts, each Ts an ODE is solved, data is stored in the sim
%alone, if you run this with a controller the computation increases
%immensely. 

% to put back %

%Per_OS = 0.3; 
%tp = 0.1;
%ts = 0.4; 
%delta_ctl = 0.8, obs = 0.9
%ratio_K_G = 4; 
%dom_sub_ratio = 6; 

%define controller (0 = no controller, 1 = state_space)

%how much faster response is of observer than controller
ratio_K_G = search_params.ratio_K_G;
%ratio between dominant poles and sub poles
dom_sub_ratio = search_params.dom_sub_ratio; 

%performance specs

%Percentage overshoot: takes 0 -

damping_ctl_dom = search_params.damping_ctl_dom;
damping_ctl_sub = search_params.damping_ctl_sub;

damping_obsv_dom = search_params.damping_obsv_dom;
damping_obsv_sub = search_params.damping_obsv_sub;
dom_sub_obsv =  search_params.dom_sub_obsv;


%damping_obsv_dom = 1;
%damping_obsv_sub = 1; 
%damping_ctl_sub = 1; 

%represents abs(re{s})
starting_pole = search_params.starting_pole; 



%minimum sampling time is 0.000884s, please respect this; 
Ts = 0.000884; 

%The following defines length of simulation in seconds
t_sim = 5; 

%linearization coordinates in centimeters
y10 = 2; 
y20 = -2; 

%only valid if load_controller = 1

y1_goal = y10; 

y2_goal = y20; 


%THE FOLLOWING MUST BE DEFINED IN METERS

%y20 is in negative meters, with the top at 0, bottom at -0.12

%Define puck 1 initial conditions here, do not put in a stupid acceleration
y1_initial = 0.0001; 
dy1_initial = 0.00; 

%Define puck 2 initial conditions here, do not put in a stupid acceleration


%y2_initial = -(yc - 6.225)/100;
y2_initial = -0.04; 

dy2_initial = 0; 

if (y1_initial < 0 || y1_initial > yc + y2_initial/100)
    disp("invalid input y1")
    return
end


if (y2_initial > 0 || y2_initial <= (y1_initial-yc/100))
    disp("invalid input y2")
    return
end


params_cs = struct('dom_sub_ratio', dom_sub_ratio,'ratio_K_G',ratio_K_G,  ...
    'damping_ctl_dom', damping_ctl_dom, 'damping_ctl_sub' , damping_ctl_sub, ...
    'damping_obsv_dom', damping_obsv_dom, 'damping_obsv_sub', damping_obsv_sub,...
   'starting_pole', starting_pole, 'dom_sub_obsv', dom_sub_obsv);


%-------------------------------------------------------------------------
%           Controller Creation
%------------------------------------------------------------------------



% Choose complete or simplified create system depending on what you use to
% create your controller 

[sys, u10, u20, x10, x30, ~] = Create_System_Complete(params, y10, y20); 

lin_points.x0 = [x10; 0; x30; 0];
lin_points.y0 = [y10; y20];
lin_points.u0 = [u10; u20];

[Valid_Controllers]  = create_control_sys(sys, params_cs, params, lin_points);
if any(isnan(Valid_Controllers.K), 'all') || any(isnan(Valid_Controllers.G), 'all')
    Performance = failedPerformanceStruct();
    return;
end

Performance = struct();



    %-------------------------------------------------------------------------
    %       The Sim Loop 
    %-------------------------------------------------------------------------
    
    %Each loop runs as follow
    
    % x[n] + u[n] -> [ SIM ] -> x[n + 1] -> [ controller ] -> u[n + 1] 
    
    %The simulation works under the basis that that there are two odometry
    %channels that publish discrete data at a rate of Ts (syncronized), thus 
    % this is the max rate at which the controller gets an update to determine
    % u[n + 1] (it cannot come to a new conclusion faster than Ts at this is
    % the rate of information transfer
    
    %Once the controller gets information, it computes and produces u[n + 1] 
    %which is fed into fed into the next simulation cycle. 
    
    %The simulation assumes constant u[n] throughout Ts and thus the ode solver
    %proceeds with constant values for u[n] throughout Ts (explained above)
    %computing the next state

    K = Valid_Controllers.K;
    G = Valid_Controllers.G;
    Kr = Valid_Controllers.Kr;
    
    
    t = 0:Ts: t_sim; 
    
    %inital sim values
    
    x_init = [y1_initial ; dy1_initial ; y2_initial ; dy2_initial]; 
    
    
    u_init = [0; 0]; 
    
    %populating data matrix
    

       Data_matrix = zeros(13, numel(t)); 
    

    
    Data_matrix(1, :) = t ; %top row of data colum
    
    Data_matrix(2:5, 1) = x_init;  % rows 2:5 hold states y1, dy1, y2, dy2 respectively
    
    Data_matrix(6:7, 1) = u_init;  % rows 6:7 hold control effort u1, u2 respectively
    
    
    % By default there is no controller, each user will pull this code, and
    % create a unique branch. DO NOT MERGE YOUR VERSION WITH DEVEL BRANCH
    
    % FETCH DEVEL BRANCH -> CREATE YOUR OWN UNIQUE BRANCH FOR YOUR OWN TESTING
    
    
    for k = 1 : numel(t) 

        r = [0; 0];
    
        x_current = Data_matrix(2:5, k); 

        %x_hat = Data_matrix(8:11, k-1); 

        y = [x_current(1); x_current(3)]; 

        x_hat_current = Data_matrix(8:11, k);

        if (k ~= numel(t))

            [x_hat_next, y_hat_next, u_current] = controller(G, K, Kr, r , sys, x_hat_current, y, Ts, lin_points); 
            Data_matrix(8:11, k+1) = x_hat_next; 
        
            Data_matrix(12:13, k+1 ) = y_hat_next; 
    
            Data_matrix(6:7, k) = u_current; 

        end
    
        if (k ~= numel(t))
    
            [x_next, sim_error]  = maglev_sim(Ts, x_current, u_current, params);
          
            if(sim_error)
                break;
            end
        
            Data_matrix(2:5, k + 1) = x_next;
    
        end
    end

%-------------------------------------------------------------------
% Control System Evaluation
%-------------------------------------------------------------------

%Some simple plots are here for your use, feel free to add what you need 

    if sim_error 
        error_mask = (t<= Ts*(k-1)); 
        Data_matrix = Data_matrix(:, error_mask); 
        t = t(error_mask);  
    end
    
    
    u = Data_matrix(6:7, :) ;
    
    x = Data_matrix(2:5, :); % Actual Dynamics, Defined by Sim in cm
    x(1, :) = 100*x(1, :); 
    x(3, :)= 100*x(3, :) + yc;
    
    x_estimated =  Data_matrix(8:11, :)*m; %shift to output reference
    x_estimated(1, :) = x_estimated(1, :)*100; 
    x_estimated(3, :) = x_estimated(3, :)*100; 

    y2_goal = y2_goal + yc; 
    tol = 0.02;     % 2% settling band
    upper_y1 = y1_goal * (1 + tol);
    lower_y1 = y1_goal * (1 - tol);
    upper_y2 = y2_goal * (1 + tol);
    lower_y2 = y2_goal * (1 - tol);
    
    
    [y1_pks, y1_locs] = findpeaks(x(1, :)); 
    
    if ((~isempty(y1_pks)) && (any(y1_pks > y1_goal)))
        peak_index_y1 = find(y1_pks > y1_goal, 1, 'first');
        Per_OS_y1 = (y1_pks(peak_index_y1) -y1_goal)/y1_goal;
        for i = 1:y1_locs(peak_index_y1)
            if (x(1, i) >= 0.98*y1_pks(peak_index_y1))
                tp_y1_98 = t(i);
                break; 
            end
        end
    else
        Per_OS_y1 = 0;
        tp_y1_98 = nan; 
    end
    
    ts1 = NaN;   % default if it never settles
    
    for i = 1:length(t)
        % check if the signal is inside band from here to end
        if all(x(1,i:end) >= lower_y1 & x(1,i:end) <= upper_y1)
            ts1 = t(i);
            break;
        end
    end
    
    [y2_pks, y2_locs] = findpeaks(x(3, :)); 
    
    if ((~isempty(y2_pks)) && (any(y2_pks > y2_goal)))
        peak_index_y2 = find(y2_pks > y2_goal, 1, 'first');
        Per_OS_y2 = (y2_pks(peak_index_y2) -y2_goal)/y2_goal;
        for i = 1:y2_locs(peak_index_y2)
            if (x(3,i) >= 0.98*y2_pks(peak_index_y2))
                tp_y2_98= t(i);
                break; 
            end
        end
    else
        Per_OS_y2 = 0;
        tp_y2_98 = nan; 
    end
    
    ts2 = NaN;   % default if it never settles
    
    for i = 1:length(t)
        % check if the signal is inside band from here to end
        if all(x(3,i:end) >= lower_y2 & x(3,i:end) <= upper_y2)
            ts2 = t(i);
            if x(3, i) > yc-0.2
                Performance = failedPerformanceStruct();
            end
            break;
        end
    end

    
    % ========================================================
    % Save performance metrics
    % ========================================================

    % Actual outputs in cm
    y_actual = [x(1,:); x(3,:)];
    
    % Estimated outputs in cm
    y_hat = [x_estimated(1,:); x_estimated(3,:)];
    
    % RMS estimation error
    RMS_y1_est = sqrt(mean((y_actual(1,:) - y_hat(1,:)).^2));
    RMS_y2_est = sqrt(mean((y_actual(2,:) - y_hat(2,:)).^2));
    
    
    Performance.OS_lower  = Per_OS_y1;
    Performance.OS_upper  = Per_OS_y2;
    
    Performance.ts_lower  = ts1;
    Performance.ts_upper  = ts2;
    
    Performance.tp_lower  = tp_y1_98;
    Performance.tp_upper  = tp_y2_98;
    
    Performance.RMS_y1_est    = RMS_y1_est;
    Performance.RMS_y2_est   = RMS_y2_est;

end

% ===========================
%   Helper: penalty struct
% ===========================

function P = failedPerformanceStruct()

    HUGE = 1e6;

    P = struct( ...
        'OS_lower',    HUGE, ...
        'OS_upper',    HUGE, ...
        'ts_lower',    HUGE, ...
        'ts_upper',    HUGE, ...
        'tp_lower',    HUGE, ...
        'tp_upper',    HUGE, ...
        'RMS_y1_est',  HUGE, ...
        'RMS_y2_est',  HUGE );
end
