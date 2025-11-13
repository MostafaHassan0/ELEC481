function [sys, u10, u20, pass] = Create_System_Simplified(params, y10, y20)
%The following system returns system value

pass = false; 
u10 = 0; 
u20 = 0; 
sys = []; 

m = params.m; %kg
a = params.a; %counts / N*cm^4 -> cm^4 terms cancel with cm .^n within force cals
b = params.b;  %cm 
c = params.c; %N *cm^4
d = params.d; %cm 
yc = params.yc;  %cm 
g = params.g; %m/s^2
n = params.n; %exponent unitless
drag = params.drag; % N·s/m  (viscous)


%look for singularities

gap = yc + y20 - y10 + d;        
d1  = (y10 + b);
d2  = (-y20 + b);

if any(~isfinite([gap,d1,d2])) || any(abs([gap,d1,d2]) < 1e-9)
    return
end

u10 = ((c./(yc + y20 - y10 +d).^4)+m.*g).*a.*(y10 + b).^4; 

u20 =  ((-c./(yc + y20 - y10 +d).^4)+ m.*g).* a.*(-y20 + b).^4; 

k12 = 4.*c ./ (yc + y20 - y10 + d).^5; 

ky1 = 4.* u10 ./(a.*(y10 + b).^5);

ky2 = 4.* u20 ./ (a.*(-y20 + b).^5);

ku1 = 1./(a.*(y10 + b).^4);

ku2 = 1./(a.*(-y20 + b).^4);

A = [0, 1, 0, 0 ; ...
    (-ky1 - k12)./m, 0, k12./m, 0; ...
    0, 0, 0, 1; ...
    k12./m, 0, (ky2 - k12)./m, 0];

B = [0, 0;... 
    ku1./m, 0; ...
    0, 0; ...
    0, ku2./m]; 

C = [1, 0, 0, 0; 0, 0, 1, 0];

D = [0, 0; 0, 0]; 

sys = ss(A, B, C, D);

if all(isfinite([u10,u20])) && all(isfinite(A(:))) && all(isfinite(B(:)))
    pass = true;
end

end