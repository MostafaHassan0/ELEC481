function x_next = maglev_sim(Ts, x, u, params)

%This simulation will use the more complicated differential equations for a
%more realistic simulation

%This takes initial conditions and uses matlab numerical solvers to
%approximate the resultant yi, and dy at the end of each sensor cycle
%(at which point the controller would be updated with new values)

%x(1) = y1
%x(2) = dy1
%x(3) = y2
%x(4) = dy2

%x_next follows the same logic

%arbitrarily 

odefun = @(t, x) maglev_ode (t, x, u, params);

opts = odeset('RelTol',1e-6, 'AbsTol',1e-9, 'MaxStep', Ts/4);

[~, x_sol] = ode45(odefun, [0 Ts], x, opts);

x_next = x_sol(end, :).';  % final state approximations after Ts

end


%To fix, 
%implement logic simulates top and bottom bonk 

%implement minimum seperation between the pucks (look into it another day
%and have them bonk and seperate if need be, not just the gap enforcement

%your unit situation is fucked and you're an idiot
