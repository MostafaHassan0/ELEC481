function [A_ctrl, B_ctrl, C_ctrl, D_ctrl] = CCF(A, B, C, D)
% CCF - Computes the controllable canonical form of a system
% Inputs:
%   A, B, C, D - State-space matrices
% Outputs:
%   A_ctrl, B_ctrl, C_ctrl, D_ctrl - Controllable canonical form matrices

    % Check controllability
    n = size(A, 1);
    Co = ctrb(A, B);
    if rank(Co) < n
        error('System is not controllable. Controlled canonical form cannot be derived.');
    end

    % Transformation matrix using controllability matrix
    T = Co(:, end-n+1:end);  % Take last n columns
    T_inv = inv(T);

    % Transform system
    A_ctrl = T_inv * A * T;
    B_ctrl = T_inv * B;
    C_ctrl = C * T;
    D_ctrl = D;
end