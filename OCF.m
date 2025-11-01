function [A_obs, B_obs, C_obs, D_obs] = OCF(A, B, C, D)
% OCF - Computes the observed canonical form of a system
% Inputs: A, B, C, D - State-space matrices
% Outputs: A_obs, B_obs, C_obs, D_obs - Observed canonical form matrices

    % Check observability
    n = size(A, 1);
    Ob = obsv(A, C);
    if rank(Ob) < n
        error('System is not observable. Observed canonical form cannot be derived.');
    end

    % Transformation matrix using observability matrix
    T = Ob(end-n+1:end, :)';  % Take last n rows and transpose
    T_inv = inv(T);

    % Transform system
    A_obs = T_inv * A * T;
    B_obs = T_inv * B;
    C_obs = C * T;
    D_obs = D;
end