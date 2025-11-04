function BodePlot(A, B, C, D)
% BodePlot - Plots Bode plots for our MIMO system

% Syntax:
%   BodePlot(A, B, C, D)

% Inputs:
%   A, B, C, D - State-space matrices of the system

    % Check if the input matrices define a valid state-space system
    try
        sys = ss(A, B, C, D);
    catch
        error('Invalid state-space matrices. Please check A, B, C, D dimensions.');
    end

    % Get number of inputs and outputs
    [ny, nu] = size(D);

    % ---------- Bode Plots ----------
    figure('Name','Bode Plots - MIMO System','NumberTitle','off');
    for i = 1:ny
        for j = 1:nu
            subplot(ny, nu, (i-1)*nu + j);
            bode(sys(i, j));
            grid on;
            title(sprintf('Bode Plot: Output %d <- Input %d', i, j));
        end
    end
end
