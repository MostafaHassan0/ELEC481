function [x_next, sim_error]  = maglev_sim(Ts, x, u, params)

%This simulation will use the more complicated differential equations for a
%more realistic simulation

%This takes initial conditions and uses matlab numerical solvers to
%approximate the resultant yi, and dy at the end of each sensor cycle
%(at which point the controller would be updated with new values)

%x(1) = y1
%x(2) = dy1
%x(3) = y2
%x(4) = dy2


sim_error = false;

odefun = @(t, x) maglev_ode(t, x, u, params);

% Use small steps to force robust integration during Ts
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', Ts/4);

% Integrate one controller cycle
[~, x_sol] = ode45(odefun, [0 Ts], x, opts);

% Candidate next state from ODE45
x_next = x_sol(end, :)';
y1 = x_next(1);     % meters
dy1 = x_next(2);
y2 = x_next(3);     % meters
dy2 = x_next(4);

%% -------------------------------------------------------------
% Convert to centimeters for gap logic
%% -------------------------------------------------------------
y1_cm = y1 * 100;
y2_cm = y2 * 100;

gap_cm = params.yc + y2_cm - y1_cm;    % upper - lower

%% -------------------------------------------------------------
% 1) HARD COLLISION CHECK: PUCK–PUCK COLLISION
%% -------------------------------------------------------------
if gap_cm < params.min_gap
    fprintf('FATAL COLLISION: gap %.3f cm < %.3f cm\n', gap_cm, params.min_gap);
    sim_error = true;
    return;
end


%% -------------------------------------------------------------
% 2) LOWER PUCK BOTTOM-PLATE BOUNCE (y1 = 0 m)
%% -------------------------------------------------------------
if y1 < 0
    y1 = 0;
    if dy1 < 0
        dy1 = -params.e * dy1;   % rebound with restitution coefficient
    else
        dy1 = 0;                 % prevent tiny positive drift
    end
end


%% -------------------------------------------------------------
% 3) UPPER PUCK TOP-PLATE BOUNCE (y2 = 0 m)
%% -------------------------------------------------------------
if y2 > 0
    y2 = 0;
    if dy2 > 0
        dy2 = -params.e * dy2;
    else
        dy2 = 0;
    end
end


%% -------------------------------------------------------------
% Save modified state as next state
%% -------------------------------------------------------------
x_next = [y1; dy1; y2; dy2];

end


