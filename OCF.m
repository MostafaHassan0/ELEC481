function [Aoc, Boc, Coc, Doc, T_inv] = OCF(A, B, C, D)
% OBSERVABLE_CANONICAL_FORM_COMPANION Computes the Observable Companion
% Form realization of a state-space system.
%
% This function uses the 'compreal' command with the 'o' (observable) type.
% For a general MIMO system, this provides a canonical companion realization.
%
% Input Arguments:
%   A, B, C, D: State-space matrices of the system
%
% Output Arguments:
%   Aoc, Boc, Coc, Doc: State-space matrices in the companion form
%   T_inv: Inverse of the transformation matrix (such that x_oc = T * x)

    % 1. Create the state-space model object
    sys = ss(A, B, C, D);

    % 2. Compute the observable companion form
    % 'o' specifies the observable companion form
    [osys, T_inv_temp] = compreal(sys, 'o');
    
    % compreal('o') returns the transformation matrix T such that the
    % observable form is Ao = T * A * inv(T), Bo = T * B, Co = C * inv(T).
    % The output argument is usually the transformation matrix T, but the 
    % documentation is clearer by defining the similarity transformation:
    % x_oc = T * x_original. The T_inv_temp returned is T for the transformation 
    % A_new = T * A * inv(T).

    T = T_inv_temp; % Renaming for clarity based on compreal documentation

    % 3. Extract the transformed matrices
    Aoc = osys.A;
    Boc = osys.B;
    Coc = osys.C;
    Doc = osys.D;
    
    % Store the transformation matrix T
    T_out = T;

    % 4. Display the results
    disp('Observable Companion Form Matrices:');
    disp('Aoc ='); disp(Aoc);
    disp('Boc ='); disp(Boc);
    disp('Coc ='); disp(Coc);
    disp('Doc ='); disp(Doc);
    disp('Transformation Matrix T (x_oc = T * x) ='); disp(T_out);
end

% --- Example Usage ---
% Define example MIMO matrices (n=3 states, m=2 inputs, p=2 outputs)
% A_ex = [-1 0 0; 0 -2 0; 0 0 -3];
% B_ex = [1 0; 1 1; 0 1];
% C_ex = [1 0 0; 0 1 1];
% D_ex = zeros(2, 2);
% 
% % Call the function
% [Aoc, Boc, Coc, Doc, T] = observable_canonical_form_companion(A_ex, B_ex, C_ex, D_ex);