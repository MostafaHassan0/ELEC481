m = 0.12; %kg
a = 1.65/10000; %cm
b = 6.2; %cm
c = 2.69; %cm
d = 4.23 ; %cm
yc = 12;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 2e-4; %drag coefficient

sim_params = [m, a, b, c, d, yc, g, n, drag]; 

%minimum sampling time is 0.000884s; 
Ts = 0.000884; 