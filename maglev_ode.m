function dxdt = maglev_ode(~, x, u, params)

m = params.m; 
a = params.a; 
b = params.b; 
c = params.c; 
d = params.d; 
yc = params.yc;  
g = params.g; 
n = params.n; 
drag = params.drag; 

i1 = u(1); 
i2 = u(2); 

y1 = max(x(1), 0);
dy1 = x(2); 
y2 = min(x(3), 0); 
dy2 = x(4); 
gap = max(yc + y2 - y1, 0.01); 

Fu11 = i1 ./ (a .* (y1 + b) .^n ); 
Fu12 =  i1 ./ (a.* (yc + y2 + b) .^n ); 
Fu21 = i2 ./ (a .* (yc - y1 + b) .^n ); 
Fu22 =  i2 ./ (a .* (-y2 + b) .^ n ); 
Fm12 = c ./ (gap + d).^n; 

ddy1 =  (Fu11 - Fu21 - m.*g - Fm12 - drag .* dy1) ./ m; 

ddy2 = (Fu22 - Fu12 - m.*g + Fm12 - drag .* dy2) ./ m ; 

dxdt = [dy1; ddy1; dy2; ddy2]; 

end