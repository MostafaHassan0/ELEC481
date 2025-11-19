
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
drag = 2e-1; %drag coefficient was determined empirically
min_gap = 0.25; %cm, break condition for simulation
e = 0.15; 

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n,'drag',drag,'min_gap',min_gap, 'e', e);


y1_min = 0;

%after playing with magnets, the top one floats about 7cm above bottom one
y2_min = -(yc/100 - 8.255/100); 



%--------------------------------------------------------------------
%       Sim Params - These are configured by user
%--------------------------------------------------------------------

%Note this sim does a lot of computation, and stores a lot of data
%Depending on your PC you can crash Matlab, or your PC by choosing an
%extremely small Ts, each Ts an ODE is solved, data is stored in the sim
%alone, if you run this with a controller the computation increases
%immensely. 

%define controller (0 = no controller, 1 = state_space)
load_controller = 1; 

%how much faster response is of observer than controller
ratio_K_G = 4; 
%ratio between dominant poles and sub poles
dom_sub_ratio = 5; 

%performance specs

%Percentage overshoot: takes 0 -
Per_OS = 0.3; 
tp = 0.05;
ts = 0.8; 


%minimum sampling time is 0.000884s, please respect this; 
Ts = 0.000884; 

%The following defines length of simulation in seconds
t_sim = 3; 

%linearization coordinates in centimeters
y10 = 2; 
y20 = -2; 


%THE FOLLOWING MUST BE DEFINED IN METERS

%y20 is in negative meters, with the top at 0, bottom at -0.12

%Define puck 1 initial conditions here, do not put in a stupid acceleration
y1_initial = 0; 
dy1_initial = 0; 


%Define puck 2 initial conditions here, do not put in a stupid acceleration
y2_initial = -(yc- 6.255)/100; 
dy2_initial = 0; 

%-------------------------------------------------------------------------
%           Controller Creation
%------------------------------------------------------------------------

% Choose complete or simplified create system depending on what you use to
% create your controller 

[sys, u10, u20, x10, x20, pass_linearize] = Create_System_Simplified(params, y10, y20); 

if (load_controller == 1)
    [G, K, poles_controller, poles_observer]  = create_control_sys(sys, Per_OS, tp, ts, ratio_K_G, dom_sub_ratio); 

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

lin_points.x0 = [x10; 0; x20; 0];
lin_points.y0 = [y10; y20];
lin_points.u0 = [u10; u20];

%de_linearize = @(tilda_coordinates) tilda_coordinates + lin_points; 

t = 0:Ts: t_sim; 

%inital sim values

x_init = [y1_initial ; dy1_initial ; y2_initial ; dy2_initial]; 


u_init = [u10; u20]; 

%populating data matrix

if (load_controller == 0)
    
    Data_matrix = zeros(7, numel(t)); 

elseif (load_controller == 1)
    Data_matrix = zeros(13, numel(t)); 

    Data_matrix(8:11, 1) = [0; 0; 0; 0]; % rows 7:8 holds initial conditions x_hat

    Data_matrix(12:13, 1) = [0; 0]; % rows 12:13 hold initial y_hat (0?)
end

Data_matrix(1, :) = t ; %top row of data colum

Data_matrix(2:5, 1) = x_init;  % rows 2:5 hold states y1, dy1, y2, dy2 respectively

Data_matrix(6:7, 1) = u_init;  % rows 6:7 hold control effort u1, u2 respectively


% By default there is no controller, each user will pull this code, and
% create a unique branch. DO NOT MERGE YOUR VERSION WITH DEVEL BRANCH

% FETCH DEVEL BRANCH -> CREATE YOUR OWN UNIQUE BRANCH FOR YOUR OWN TESTING


for k = 1 : numel(t) - 1

    x_current = Data_matrix(2:5, k); 

    u_current = Data_matrix(6:7, k);

    if (load_controller == 0)

        u_next = u_current;
    
    elseif (load_controller == 1)

        x_hat = Data_matrix(8:11, k); 

        y = [x_current(1); x_current(3)]; 

        [x_hat_next, y_hat_next, u_next] = controller(G, K, sys, x_hat, y, Ts, lin_points);

        Data_matrix(8:11, k + 1) = x_hat_next; 
    
        Data_matrix(12:13, k + 1) = y_hat_next; 

    end


    [x_next, sim_error]  = maglev_sim(Ts, x_current, u_current, params);
  
    if(sim_error)
        break;
    end

    Data_matrix(2:5, k + 1) = x_next;

    Data_matrix(6:7, k + 1) = u_next; 

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

u = zeros(2, size(Data_matrix, 2)); 

u = Data_matrix(6:end, :) ;

x = zeros(4, numel(t)); 

x = Data_matrix(2:5, :); 

x3 = (x(3, :)*100 + yc);


% Plot Control Effort vs Position

% u1, y1

figure;

% ---------- Position Plot ----------
subplot(2,1,1);
plot(t, x(1,:)*100, 'g-', 'LineWidth', 1.2);
ylabel('Position (cm)');
title('Lower Magnet Position and Control Effort');
grid on;

% ---------- Control Effort Plot ----------
subplot(2,1,2);
plot(t, u(1,:), 'r-', 'LineWidth', 1.2);
ylabel('Control Effort (DAC counts)');
xlabel('Time (s)');
grid on;

figure;

% ---------- Position Plot ----------
subplot(2,1,1);
plot(t, x3, 'g-', 'LineWidth', 1.2);
ylabel('Position (cm)');
title('Upper Magnet Position and Control Effort');
grid on;

% ---------- Control Effort Plot ----------
subplot(2,1,2);
plot(t, u(2,:), 'r-', 'LineWidth', 1.2);
ylabel('Control Effort (DAC counts)');
xlabel('Time (s)');
grid on;

% assuming x and x_hat are 4×N arrays over time t
figure;
for i = 1:4
    subplot(4,1,i);
    plot(t, x(i,:), 'b-', t, x_hat(i,:), 'r--');
    ylabel(sprintf('x_%d', i));
    if i==1, legend('state','estimate'); end
end
xlabel('Time (s)');

figure;
for i = 1:4
    subplot(4,1,i);
    plot(t, x(i,:) - x_hat(i,:));
    ylabel(sprintf('e_%d', i));
end
xlabel('Time (s)');
sgtitle('State estimation error x - x\_hat');





%{

figure;
subplot(2, 1, 1);

%yyaxis left
plot(t, x(1, :)*100, 'g-', 'LineWidth', 1.2); 
ylabel('Position (cm)');
%ylim([0 yc]); 
%grid on;
subplot(2, 1, 2);
%yyaxis right
plot(t, u(1,:), 'r-', 'LineWidth', 1.2); 
%ylabel('Control Effort (DAC counts)');

%ylim([0 yc]); 
grid on;

xlabel('Time (s)');
title('Lower Magnet Position and Control Effort');
legend('Position (cm)', 'Control Effort (counts)');



%u2, y2
figure;
subplot(2, 1, 1);

%yyaxis left
plot(t, x3, 'g-', 'LineWidth', 1.2); % upper magnet position (cm)
ylabel('Position (cm)');
%ylim([0 yc]); 
%grid on;
subplot(2, 1, 2);
%yyaxis right
plot(t, u(2,:), 'r-', 'LineWidth', 1.2); % control effort (counts)
ylabel('Control Effort (DAC counts)');

%ylim([0 yc]); 
grid on;

xlabel('Time (s)');
title('Upper Magnet Position and Control Effort');
legend('Position (cm)', 'Control Effort (counts)');

%u1, dy1
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

%}