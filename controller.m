function [x_hat_next, y_hat_next, u] = controller(G, K, sys, x_hat, y, Ts, lin_points)

% tilda means linearized frame of reference


A = sys.A;
B = sys.B;
C = sys.C;


x0 = lin_points.x0/100;
y0 = lin_points.y0/100;
u0 = lin_points.u0;


%observer state estimation from last execution cycle
x_hat_tilde = x_hat - x0; 

y_tilde = y - y0; 

 % Estimated output (tilde)
y_hat_tilde = C * x_hat_tilde;

%current execution cycle control effort
u_tilde = -K * x_hat_tilde;

% observer dynamics in tilde coordinates
x_hat_tilde_dot = A * x_hat_tilde + B * u_tilde + G * (y_tilde - y_hat_tilde);

%next state prediction tilde coordinates
x_hat_tilde_next = x_hat_tilde + Ts * x_hat_tilde_dot;

% Convert back to plant coordinate system
x_hat_next = x_hat_tilde_next + x0;
y_hat_next = C * x_hat_tilde_next + y0;
u = u0 + u_tilde; 

u_max = 30000; 

u_min = -30000; 

u = max(min(u, u_max), u_min); 
end