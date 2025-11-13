
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
c = 23213 ; %N *cm^4
d = 4.23 ; %cm
yc = 12;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 3e-2; %drag coefficient was determined empirically
min_gap = 0.25; %cm, break condition for simulation

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n,'drag',drag,'min_gap',min_gap);


y1_min = 0;

%after playing with magnets, the top one floats about 7cm above bottom one
y2_min = -(yc/100 - 0.07); 

%--------------------------------------------------------------------
%       Sim Params - These are configured by user
%--------------------------------------------------------------------

%Note this sim does a lot of computation, and stores a lot of data
%Depending on your PC you can crash Matlab, or your PC by choosing an
%extremely small Ts, each Ts an ODE is solved, data is stored in the sim
%alone, if you run this with a controller the computation increases
%immensely. 


%minimum sampling time is 0.000884s, please respect this; 
Ts = 0.000884; 

%The following defines length of simulation in seconds
t_sim = 5; 

%linearization coordinates in centimeters
y10 = 1; 
y20 = -1; 

%THE FOLLOWING MUST BE DEFINED IN METERS

%Define puck 1 initial conditions here, do not put in a stupid acceleration
y1_initial = y10/100; 
dy1_initial = 0; 


%Define puck 2 initial conditions here, do not put in a stupid acceleration
y2_initial = y20/100; 
dy2_initial = 0; 

%-------------------------------------------------------------------------
%           Controller Creation
%------------------------------------------------------------------------

[sys, u10, u20, ~] = Create_System_Complete(params, y10, y20); 

u1_init = u10; 
u2_init = u20; 

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

t = 0:Ts: t_sim; 

%inital sim values

x_init = [y1_initial ; dy1_initial ; y2_initial ; dy2_initial]; 

u_init = [u1_init; u2_init]; 

%populating data matrix

Data_matrix = zeros(7, numel(t)); 

Data_matrix(1, :) = t ; %top row of data colum

Data_matrix(2:5, 1) = x_init;  % rows 2:5 hold states y1, dy1, y2, dy2 respectively

Data_matrix(6:end, 1) = u_init;  % rows 6:7 hold control effort u1, u2 respectively

% By default there is no controller, each user will pull this code, and
% create a unique branch. DO NOT MERGE YOUR VERSION WITH DEVEL BRANCH

% FETCH DEVEL BRANCH -> CREATE YOUR OWN UNIQUE BRANCH FOR YOUR OWN TESTING

for k = 1 : numel(t) - 1

    x_current = Data_matrix(2:5, k); 

    u_current = Data_matrix(6:end, k); 

    [x_next, sim_error]  = maglev_sim(Ts, x_current, u_current, params);

    % here a controller function would go to calculate u_next
    u_next =  [u10; u20];

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

yyaxis left
plot(t, x(1, :)*100, 'g-', 'LineWidth', 1.2); 
ylabel('Position (cm)');
ylim([0 12]); 
grid on;

yyaxis right
plot(t, u(1,:), 'r-', 'LineWidth', 1.2); 
ylabel('Control Effort (DAC counts)');

xlabel('Time (s)');
title('Lower Magnet Position and Control Effort');
legend('Position (cm)', 'Control Effort (counts)');

%}

%u2, y2
figure;

yyaxis left
plot(t, x3, 'g-', 'LineWidth', 1.2); % upper magnet position (cm)
ylabel('Position (cm)');
ylim([0 12]); 
grid on;

yyaxis right
plot(t, u(2,:), 'r-', 'LineWidth', 1.2); % control effort (counts)
ylabel('Control Effort (DAC counts)');

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

