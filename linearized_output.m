function [u_lin, x_lin]  = linearized_output(lin_points, r, params)

    % --- Normalize r to correct shape: 2×N ---
    if size(r,1) ~= 2 && size(r,2) == 2
        r = r.';  % transpose if user passed Nx2 instead of 2xN
    end

    if size(r,1) ~= 2
        error('r must be 2×N (two outputs: y1 and y2).');
    end

    N = size(r,2);           % number of reference samples
    u_lin = zeros(2, N);     % 2×N feedforward inputs
    x_lin = zeros(4, N);

    y0  = lin_points.y0;     % equilibrium y1,y2
    y10 = y0(1);
    y20 = y0(2);

    for i = 1:N
        % desired *absolute* output positions
        y1_des = y10 + r(1,i)*100;
        y2_des = y20 + r(2,i)*100;

        % Compute nonlinear equilibrium inputs
        [~, u10, u20, x10, x20, ~] = Create_System_Complete(params, y1_des, y2_des);

        % store properly as a 2×1 column
        u_lin(:,i) = [u10; u20];
        x_lin (:, i) = [x10; 0; x20; 0]; 
    end
end
