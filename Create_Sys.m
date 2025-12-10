function [sys, u10, u20, x10, x30, pass] = Create_Sys()
%The following system returns system value

pass = false; 
u10 = 0; 
u20 = 0; 
sys = []; 

m = 0.12; %kg
a = 1.65; %cm
b = 6.2; %cm
c = 23984 ; %N *cm^4
d = 3.7698 ; %cm
yc = 13;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 5e-2; %drag minimum 


%look for singularities

e = yc + y20 - y10 + d;        
f  = yc - y10 + b ;
h = -y20 + b ; 
k = yc + y20 + b; 
l = y10 + b; 

if (any(~isfinite([e, f, h, k, l])) || any(abs([e, f, h, k, l]) < 1e-9 )|| abs(k^4*f^4 - l^4 *h^4) < 1e-9)
    return
end

u10 = ( k^4 * a * l^4* ( c * ( f^4 - h^4 ) + e^4 * m * g * ( h^4 + f^4 ) ) ) / ...
                     (e^4 * (k^4 * f^4 - l^4 * h^4)); 

u20 = ( h^4 * a* f^4 * ( c * ( l^4 - k^4 ) + e^4 * m * g * ( l^4 + k^4 ) ) ) / ...
                     (e^4 * (k^4 * f^4 - l^4 * h^4)); 

x10 = y10/(100); 

x30 = (y20)/(100);  

km12 = 4 * c / e^5 ; 

ky11 = 4 * u10 / (a * l^5); 

ky12 = 4 * u10 / (a * k^5); 

ky21 = 4 * u20 / (a * f^5);

ky22 = 4 * u20 / (a * h^5);

ku11 = 1 / (a * l^4); 

ku12 = 1 / (a * k^4); 

ku21 = 1 / (a * f^4); 

ku22 = 1 / (a * h^4); 

A = [       0,                  1,             0,                   0; ...
    ((-km12 -ky11 + ky21)/m), -drag/m,      (km12/m),               0; ...
            0,                  0,          0,                      1;...
        (km12 / m),             0,   ((-km12 + ky22 -ky12)/m),  -drag/m]; 

B = [ 0, 0;  ...
    ku11/m, -ku21/m; ...
    0, 0; ...
    -ku12/m, ku22/m];

C = [1, 0, 0, 0; 0, 0, 1, 0]; 

D = zeros(2, 2); 

sys = ss(A, B, C, D);

if all(isfinite([u10,u20])) && all(isfinite(A(:))) && all(isfinite(B(:)))
    pass = true;
else
    disp('Creating system and linearization points failed')
end

end