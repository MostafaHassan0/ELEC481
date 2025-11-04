function [y_impulse, t_impulse, y_step, t_step] = IandS_response(A, B, C, D)
% IandS_response - Compute impulse and step responses of the MIMO state-space system and plots them

% Syntax:
%   [y_impulse, t_impulse, y_step, t_step] = responseMIMO(A, B, C, D)

% Inputs:
%   A, B, C, D - State-space matrices defining the MIMO system

% Outputs:
%   y_impulse - Impulse response (time-domain output array)
%   t_impulse - Time vector for impulse response
%   y_step    - Step response (time-domain output array)
%   t_step    - Time vector for step response
%   Plots of both responses

    % Create state-space system
    sys = ss(A, B, C, D);

    % Compute impulse response
    [y_impulse, t_impulse] = impulse(sys);

    % Compute step response
    [y_step, t_step] = step(sys);

    % Display results as plots
    figure('Name','Impulse Response - MIMO System','NumberTitle','off');
    impulse(sys);
    grid on;

    figure('Name','Step Response - MIMO System','NumberTitle','off');
    step(sys);
    grid on;
end
