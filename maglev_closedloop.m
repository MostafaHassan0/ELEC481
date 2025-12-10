function dxdt = maglev_closedloop(t, x, params, K, lin_points, r, ulin, xlin)

    xhat = x - xlin;
    u = ulin - K * (xhat) + r; 

    dxdt = maglev_ode(t, x, u, params);
end