
%***********************************************************************
% By default there is no controller, each user will pull this code, and
% create a unique branch. DO NOT MERGE YOUR VERSION WITH DEVEL BRANCH

%In GITHUB make sure you have your own branch $YourTestBranch 

% FETCH DEVEL BRANCH LOCALLY -> PLAY WITH LOCALLY 

%   git switch devel
%   git pull
%   git switch -c $YourTestBranch 
%   ~ make modifications ~
%   git add .
%   git commit -m "My comments for the commit"
%   git push -u origin $YourTestBranch


%*************************************************************************

% The simulator code follows

%----------------------------------------------------------------------
%     System Params - You shouldn't change these
%---------------------------------------------------------------------

%To Do 
%Fix saturation in the output
%Full State Observer
%Reduced Order Observer
%Tracking 


m = 0.12; %kg
a = 1.65; %cm
b = 6.2; %cm
c = 23984 ; %N *cm^4
d = 3.7698 ; %cm
yc = 13;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 5e-2; %drag minimum 
%drag = 2.5e-1; %drag maximum
min_gap = 0.25; %cm, break condition for simulation
e = 1; %Bounce Efficiency
umax = 30000; 

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n, ...
    'drag',drag,'min_gap',min_gap, 'e', e, 'umax', umax);



%--------------------------------------------------------------------
%       Sim Params - These are configured by user
%--------------------------------------------------------------------

%Note this sim does a lot of computation, and stores a lot of data
%Depending on your PC you can crash Matlab, or your PC by choosing an
%extremely small Ts, each Ts an ODE is solved, data is stored in the sim
%alone, if you run this with a controller the computation increases
%immensely. 

% to put back 

Per_OS = 0.13;
tp = 0.15;
ts = 0.25; 

%Per_OS = 0.2; -> pretty good
%tp = 0.15;
%ts = 0.3; 

%Per_OS = 0.2; -> 0.162
%tp = 0.19; -> 0.139
%ts = 0.25; -> 0.530


%define controller (0 = no controller, 1 = state_space)
load_controller = 1; 
load_observer = 1; 

if (load_observer == 1 && load_controller == 0)
    error('Must run observer and controller at same time'); 
end

plot_velocity = false; 

%performance specs

%minimum sampling time is 0.000884s, please respect this; 
Ts = 0.000884; 

%The following defines length of simulation in seconds
t_sim = 10; 

%linearization coordinates in centimeters
y10 = 2; 
y20 = -2; 


%only valid if load_controller = 1

y1_goal = y10; 

y2_goal = y20; 

x_worst = [0.015; 0; -0.025; 0]; 

%THE FOLLOWING MUST BE DEFINED IN METERS

%y20 is in negative meters, with the top at 0, bottom at -0.12

%Define puck 1 initial conditions here, do not put in a stupid acceleration
y1_initial = 0.01; 
dy1_initial = 0.00; 

%Define puck 2 initial conditions here, do not put in a stupid acceleration


y2_initial = -0.03; 
dy2_initial = 0; 

if (y1_initial < 0 || y1_initial > yc + y2_initial/100)
    disp("invalid input y1")
    return
end

if (y2_initial > 0 || y2_initial <= (y1_initial-yc/100))
    disp("invalid input y2")
    return
end

params_cs = struct('ts', ts,'tp',tp, 'Per_OS', Per_OS, 'x_worst', x_worst);


%-------------------------------------------------------------------------
%           Controller Creation
%------------------------------------------------------------------------



% Choose complete or simplified create system depending on what you use to
% create your controller 

[sys, u10, u20, x10, x30, pass_linearize] = Create_System_Complete(params, y10, y20); 

x0 = [x10; 0; x30; 0];

u0 = [u10; u20]; 

lin_points.x0 = x0; 
lin_points.y0 = [y10; y20];
lin_points.u0 = u0; 

A = sys.A; 
B = sys.B; 
C = sys.C; 

if (load_controller == 1)
    %[Valid_Controllers]  = create_control_sys(sys, params_cs, params, lin_points); 
    [K, results_K, K_poles] = create_control_sys(sys, params_cs, params, lin_points);
   
    if (load_observer == 1)
         [G, results_G] = full_observor(sys, K_poles); 
    elseif (load_observer == 2)
         [G, F_ret, L_ret, T_ret, A_RO, B_RO] = reduced_observer(sys, K_poles); 
         unmeas_index = [2, 4];
         meas_index = [1, 3]; 
    end

    if (~pass_linearize)
        return; 
    end
end


%-------------------------------------------------------------------------
%                    The Sim Loop 
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


t = 0:Ts: t_sim; 

%1 == step
%2 == square
%3 == sinusoid
%(time, type, freq, amplitude)
r = generate_trajectory(t, 2, 0.5, 0.0025); 
[ulin, xlin]  = linearized_output(lin_points, r, params); 

%inital sim values

x_init = [y1_initial ; dy1_initial ; y2_initial ; dy2_initial]; 


u_init = [u10; u20]; 

%populating data matrix

if (load_controller == 0 && load_observer == 0)
    
    y1_goal = y10; 
    y2_goal = y20; 
    K = [0, 0, 0, 0; 0, 0, 0, 0]; 
    Data_matrix = zeros(7, numel(t)); 
elseif (load_controller == 1 && load_observer == 0)
    Data_matrix = zeros(7, numel(t)); 
    G = 0; 
    A_RO = 0; 
    B_RO = 0; 

elseif (load_observer == 1 && load_controller == 1)
    Data_matrix = zeros(11, numel(t)); 
elseif (load_observer == 2 && load_controller == 1)
     Data_matrix = zeros(9, numel(t)); 
end


Data_matrix(1, :) = t ; %top row of data colum

Data_matrix(2:5, 1) = x_init;  % rows 2:5 hold states y1, dy1, y2, dy2 respectively

Data_matrix(6:7, 1) = u_init;  % rows 6:7 hold control effort u1, u2 respectively


% By default there is no controller, each user will pull this code, and
% create a unique branch. DO NOT MERGE YOUR VERSION WITH DEVEL BRANCH

% FETCH DEVEL BRANCH -> CREATE YOUR OWN UNIQUE BRANCH FOR YOUR OWN TESTING


for k = 1 : numel(t) 

    x_current = Data_matrix(2:5, k); 

    if (load_controller == 0)

        u_current = Data_matrix(6:7, k);
        X_current  = x_current; 
        ulin_ = u0; 
        xlin_ = x0;

        if(k ~= numel(t))
            Data_matrix(6:7, k+1) = u_current;
        end

    elseif( load_observer== 0 && load_controller == 1)

        X_current = x_current; 
        ulin_ = ulin(:, k); 
        xlin_ = xlin(:, k); 

    elseif( load_observer== 1 && load_controller == 1)

        xhat_current = Data_matrix(8:11, k); 
        X_current = [x_current; xhat_current]; 
        ulin_ = ulin(:, k); 
        xlin_ = xlin(:, k); 

    elseif( load_observer == 2 && load_controller == 1)

    end

    if (k ~= numel(t))

        [x_next, sim_error]  = maglev_sim(Ts, X_current, params, lin_points, K, G, load_controller, load_observer, sys, A_RO, B_RO, r(:, k), ulin_, xlin_);
      
        if(sim_error)
            break;
        end
        
        if (load_observer == 2 && load_controller == 1)
            
        elseif (load_observer == 1 && load_controller == 1)

            Data_matrix(2:5, k + 1) = x_next(1:4);
            x_hat_next =  x_next(5:8); 
            u_next    = ulin(:, k+1) - K * (x_hat_next - xlin(:, k+1)) + r(k+ 1); 
            Data_matrix(6:7, k + 1) = u_next; 
            Data_matrix(8:11, k + 1) = x_hat_next;

        elseif(load_controller == 1 && load_observer == 0)
            
            Data_matrix(2:5, k + 1) = x_next;
            u_next = u0 - K*(x_next - xlin(:, k+1)) + r(:, k + 1); 
            Data_matrix(6:7, k + 1) = u_next; 

        elseif (load_observer == 0 && load_controller == 0)
             Data_matrix(2:5, k + 1) = x_next;
        end

    end
end

%-------------------------------------------------------------------
%               Control System Evaluation
%-------------------------------------------------------------------

%Some simple plots are here for your use, feel free to add what you need 

    if sim_error 
        error_mask = (t<= Ts*(k-1)); 
        Data_matrix = Data_matrix(:, error_mask); 
        t = t(error_mask);  
    end
    
    t_plot = zeros(1, numel(t)); 
    
    u = Data_matrix(6:7, :) ;
    
    x = Data_matrix(2:5, :); % Actual Dynamics, Defined by Sim in cm
    x(1, :) = 100*x(1, :); 
    x(3, :)= 100*x(3, :) + yc;

    if (load_observer == 1)
        x_estimated = Data_matrix(8:11, :); 
        x_estimated(1, :) = 100*x_estimated(1, :); 
        x_estimated(3, :)= 100*x_estimated(3, :) + yc;
    end

if (load_observer == 2)
    % deviation estimate stored in Data_matrix
    x_estimated = Data_matrix(8:9, :);
end


%-------------------------------------------------------------------
% Control System Evaluation
%-------------------------------------------------------------------

traj = zeros(2, numel(t)); 
traj(1, :) = r(1, :).*100 + y10; 
traj(2, :) = r(2, :).*100 + y20 + yc; 

if (load_controller == 1)
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
            break;
        end
    end


     disp(eig(A - B*K));

     if (load_observer == 1)

     disp(eig(A-G*C))
     end
     %rms_error = sqrt( mean((x - x_estimated).^2));
     %rms_error_total = sqrt( mean((x(:) - x_estimated(:)).^2));

    disp('Lower Magnet performance results: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n  \n',  Per_OS_y1, tp_y1_98, ts1);

    disp('Upper Magnet performance results: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n \n',  Per_OS_y2, tp_y2_98, ts2);

    %disp('RMS Error: ')
    %fprintf('Total RMS Error: %.3f, State 1: %.3fState 2: %.3f.3,  State 3: %.3f.3 State 3: %.3f.3', ...
       % rms_error_total, rms_error(1), rms_error(2), rms_error(3), rms_error(4));

end
    figure;

% ---------- Position Plot Lower Magnet ----------
subplot(2,1,1);
plot(t, x(1,:), 'g-', 'LineWidth', 1.2)
hold on; 
plot(t, traj(1,:), 'r--');
ylabel('Position (cm)');
ylim([0, 5]);
legend({'Position','Reference'})
title('Lower Magnet Position and Control Effort');
grid on;

% ---------- Control Effort Plot ----------
subplot(2,1,2);
plot(t, u(1,:), 'r-', 'LineWidth', 1.2);
ylabel('Control Effort (DAC counts)');
xlabel('Time (s)');
grid on;


figure;

% ---------- Position Plot Upper Magnet ----------
subplot(2,1,1);
plot(t, x(3, :), 'g-', 'LineWidth', 1.2)
hold on; 
plot(t, traj(2,:), 'r--');
ylabel('Position (cm)');
ylim([(yc-5), yc])
legend({'Position','Reference'}); 
title('Upper Magnet Position and Control Effort');
grid on;


% ---------- Control Effort Plot ----------
subplot(2,1,2);
plot(t, u(2,:), 'r-', 'LineWidth', 1.2);
ylabel('Control Effort (DAC counts)');
xlabel('Time (s)');
grid on;


if (load_observer == 1)
% assuming x and x_hat are 4×N arrays over time t
    figure;
    for i = 1:4
        subplot(4,1,i);
        plot(t, x(i,:), 'b-', t, x_estimated(i,:), 'r--');
        ylabel(sprintf('x_%d', i));
        if i==1, legend('state','estimate'); end
    end
    xlabel('Time (s)');
    title('State Estimation vs States sim: %d');

end

if (load_observer == 2)
% assuming x and x_hat are 4×N arrays over time t
    figure;
    for i = 1:2
        subplot(2,1,i);
        plot(t, x(2*i,:), 'b-', t, x_estimated(i,:), 'r--');
        ylabel(sprintf('x_%d', 2*i));
        if i==1, legend('state','estimate'); end
    end
    xlabel('Time (s)');
    title('State Estimation vs States sim: %d');

end





   





