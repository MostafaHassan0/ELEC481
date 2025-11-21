
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
e = 0.15; 

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n,'drag',drag,'min_gap',min_gap, 'e', e);

umax = 30000; 


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
load_controller = 1; 

plot_velocity = false; 

%how much faster response is of observer than controller
ratio_K_G = 3; 
%ratio between dominant poles and sub poles
dom_sub_ratio = 4; 

%performance specs

%Percentage overshoot: takes 0 -
Per_OS = 0.1; 
tp = 0.1;
ts = 4; 

safety_factor = 0.95; 


params_cs = struct('Per_OS',Per_OS ,'tp',tp,'ts',ts, 'dom_sub_ratio', dom_sub_ratio,'ratio_K_G',ratio_K_G, 'safety_factor', safety_factor );



%minimum sampling time is 0.000884s, please respect this; 
Ts = 0.000884; 

%The following defines length of simulation in seconds
t_sim = 15; 

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


y2_initial = -(yc - 6.225)/100; 
dy2_initial = 0; 

if (y1_initial < 0 || y1_initial > yc + y2_initial/100)
    dip("invalid input y1")
    return
end


if (y2_initial > 0 || y2_initial <= (y1_initial-yc/100))
    dip("invalid input y2")
    return
end

xhat_max = [(y10/100 - y1_initial ); 0;  ((y20 + yc)/100 + y2_initial); 0];

params_cs = struct('Per_OS',Per_OS ,'tp',tp,'ts',ts, 'dom_sub_ratio', dom_sub_ratio,'ratio_K_G',ratio_K_G,  ...
    'safety_factor', safety_factor);


%-------------------------------------------------------------------------
%           Controller Creation
%------------------------------------------------------------------------

% Choose complete or simplified create system depending on what you use to
% create your controller 

[sys, u10, u20, x10, x30, pass_linearize] = Create_System_Complete(params, y10, y20); 

if (load_controller == 1)
    [G, K, poles_controller, poles_observer]  = create_control_sys(sys, params_cs); 

    max_u = abs(K*xhat_max);

    sprintf('Max u predicted u = %d \n', abs(max_u(2, 1)))

    if (~pass_linearize)
        return; 
    end
end



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

lin_points.x0 = [x10; 0; x30; 0];
lin_points.y0 = [y10; y20];
lin_points.u0 = [u10; u20];

%de_linearize = @(tilda_coordinates) tilda_coordinates + lin_points; 

t = 0:Ts: t_sim; 

%inital sim values

x_init = [y1_initial ; dy1_initial ; y2_initial ; dy2_initial]; 


u_init = [0; 0]; 

%populating data matrix

if (load_controller == 0)
    
    Data_matrix = zeros(7, numel(t)); 
    y1_goal = y10; 
    y2_goal = y20; 

elseif (load_controller == 1)
    Data_matrix = zeros(13, numel(t)); 

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
        if(k ~= numel(t))
            Data_matrix(6:7, k+1) = u_current;
        end
    
    elseif (load_controller == 1)

        %x_hat = Data_matrix(8:11, k-1); 

        y = [x_current(1); x_current(3)]; 

        x_hat_current = Data_matrix(8:11, k);

        if (k ~= numel(t))

            [x_hat_next, y_hat_next, u_current] = controller(G, K, sys, x_hat_current, y, Ts, lin_points);
    
            Data_matrix(8:11, k+1) = x_hat_next; 
        
            Data_matrix(12:13, k+1 ) = y_hat_next; 
    
            Data_matrix(6:7, k) = u_current; 
        end


    end

    if (k ~= numel(t))


        [x_next, sim_error]  = maglev_sim(Ts, x_current, u_current, params);
      
        if(sim_error)
            break;
        end
    
        Data_matrix(2:5, k + 1) = x_next;

    end


end



%-------------------------------------------------------------------------
%                   Plotting
%-------------------------------------------------------------------------

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

if (load_controller == 1)
    x_estimated =  Data_matrix(8:11, :)*m; %shift to output reference
    x_estimated(1, :) = x_estimated(1, :)*100; 
    x_estimated(3, :) = x_estimated(3, :)*100; 
end


% Plot Control Effort vs Position

% u1, y1

figure;

% ---------- Position Plot Lower Magnet ----------
subplot(2,1,1);
plot(t, x(1,:), 'g-', 'LineWidth', 1.2);
ylabel('Position (cm)');
ylim([0, yc])
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
plot(t, x(3, :), 'g-', 'LineWidth', 1.2);
ylabel('Position (cm)');
ylim([0, yc])
title('Upper Magnet Position and Control Effort');
grid on;


% ---------- Control Effort Plot ----------
subplot(2,1,2);
plot(t, u(2,:), 'r-', 'LineWidth', 1.2);
ylabel('Control Effort (DAC counts)');
xlabel('Time (s)');
grid on;

if (load_controller == 1)
% assuming x and x_hat are 4×N arrays over time t
    figure;
    for i = 1:4
        subplot(4,1,i);
        plot(t, x(i,:), 'b-', t, x_estimated(i,:), 'r--');
        ylabel(sprintf('x_%d', i));
        if i==1, legend('state','estimate'); end
    end
    xlabel('Time (s)');
    title('State Estimation vs States');
   
end
if (plot_velocity == true)
    figure;
    
    yyaxis left
    plot(t, x(2, :), 'g-', 'LineWidth', 1.2); % upper magnet position (cm)
    ylabel('Velocity (m/s)');
    grid on;
    
    yyaxis right
    plot(t, u(1,:), 'r-', 'LineWidth', 1.2); % control effort (counts)
    ylabel('Control Effort (DAC counts)');
    
    xlabel('Time (s)');
    title('Lower Magnet Control Effort vs Velocity');
    legend('Position (cm)', 'Control Effort (counts)');
    
    %u2, dy2
    
    figure;
    
    yyaxis left
    plot(t, x(4, :), 'g-', 'LineWidth', 1.2); % upper magnet position (cm)
    ylabel('Velocity (m/s)');
    grid on;
    
    yyaxis right
    plot(t, u(2,:), 'r-', 'LineWidth', 1.2); % control effort (counts)
    ylabel('Control Effort (DAC counts)');
    
    xlabel('Time (s)');
    title('Upper Magnet Control Effort vs Velocity');
    legend('Position (cm)', 'Control Effort (counts)');
end

%-------------------------------------------------------------------
% Control System Evaluation
%-------------------------------------------------------------------
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
    

    disp('Lower Magnet performance goals: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n \n',  Per_OS , tp, ts);
    
    disp('Lower Magnet performance results: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n \n \n',  Per_OS_y1, tp_y1_98, ts1);

    disp('Upper Magnet performance goals: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n \n',  Per_OS , tp, ts);

    disp('Upper Magnet performance results: ');
    fprintf('Percentage OS: %.3f, Peak Time: %.3f, Settling Time: %.3f \n',  Per_OS_y2, tp_y2_98, ts2);
end




