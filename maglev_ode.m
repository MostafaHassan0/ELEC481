function dxdt = maglev_ode(~, x, u, params)

%the silliness that proceeds is due to the procedure of the maglev manual
%which calculates the force laws in centimeters, however the differential
%equations are meters based

%for simplicity the simulation will assume feedback in meters which are
%then converted to centimeters for the purposes of initial sim 

m = params.m; %kg
a = params.a; %counts / N*cm^4 -> cm^4 terms cancel with cm .^n within force cals
b = params.b;  %cm 
c = params.c; %N *cm^4
d = params.d; %cm 
yc = params.yc;  %cm 
g = params.g; %m/s^2
n = params.n; %exponent unitless
drag = params.drag; % N·s/m  (viscous)

u1 = u(1); 
u2 = u(2); 

y1_cm = x(1) * 100; 
y2_cm = x(3) * 100; 

y1 = max(y1_cm, 0);
dy1 = x(2); 
y2 = min(y2_cm, 0); 
dy2 = x(4); 

Fu11 = u1 ./ (a .* (y1 + b) .^n );                  %N
Fu12 =  u1 ./ (a.* (yc + y2 + b) .^n );             %N
Fu21 = u2 ./ (a .* (yc - y1 + b) .^n );             %N
Fu22 =  u2 ./ (a .* (-y2 + b) .^ n );               %N
Fm12 = c ./ (yc + y2 - y1 + d).^n;                           %N

ddy1 =  (Fu11 - Fu21 - m.*g - Fm12 - drag .* dy1) ./ m; 

ddy2 = (Fu22 - Fu12 - m.*g + Fm12 - drag .* dy2) ./ m ; 

dxdt = [dy1; ddy1; dy2; ddy2]; 

end