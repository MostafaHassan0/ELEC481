function  dXdt = maglev_cont_input(t, X, params, lin_points, K, G, sys, ulin, xlin)
    A = sys.A; 
    B = sys.B; 
    C = sys.C; 
    x0 = lin_points.x0; 
    u0 = lin_points.u0; 

    nx = length(lin_points.x0);     % number of states (should be 4)
    x     = X(1:nx);                % true plant state
    xhat  = X(nx+1 : 2*nx);         % estimated state
    xhat_rel = xhat - xlin;
    
    u = ulin - K * xhat_rel; %add r in when ready

    dxdt = maglev_ode(t, x, u, params);

    y    = C * x;           % actual measurement
    yhat = C * xhat;        % predicted measurement
    xhat_predict = A * xhat_rel + B * (u - u0);
    xhat_correct = G * (y - yhat);

    dxhat = xhat_predict + xhat_correct;
    dXdt = [ dxdt ; dxhat];
end