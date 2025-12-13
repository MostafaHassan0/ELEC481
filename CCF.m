function [Acc, Bcc, Ccc, Dcc, T] = CCF(A, B, C, D)
% CONTROLLABLE_CANONICAL_FORM_COMPANION Computes the Controllable Companion
% Form realization of a state-space system.
%
% This function uses the 'compreal' command, which produces the controllable
% companion realization for a single-input LTI model or a multi-input model
% controllable from the first input. For a general MIMO system, this form
% is a canonical realization, but not the non-unique Luenberger/Brunovsky
% Controllable Canonical Form.
%
% Input Arguments:
%   A, B, C, D: State-space matrices of the system
%
% Output Arguments:
%   Acc, Bcc, Ccc, Dcc: State-space matrices in the companion form
%   T: State transformation matrix (such that x_cc = T * x)

    % 1. Create the state-space model object
    sys = ss(A, B, C, D);

    % 2. Compute the controllable companion form
    % 'c' specifies the controllable companion form
    [csys, T] = compreal(sys, 'c');

    % 3. Extract the transformed matrices
    Acc = csys.A;
    Bcc = csys.B;
    Ccc = csys.C;
    Dcc = csys.D;

    % 4. Display the results
    disp('Controllable Companion Form Matrices:');
    disp('Acc ='); disp(Acc);
    disp('Bcc ='); disp(Bcc);
    disp('Ccc ='); disp(Ccc);
    disp('Dcc ='); disp(Dcc);
    disp('Transformation Matrix T (x_cc = T * x) ='); disp(T);
end

% --- Example Usage ---
% Define example MIMO matrices (n=3 states, m=2 inputs, p=2 outputs)
% This is just an illustrative example.
% A_ex = [-1 0 0; 0 -2 0; 0 0 -3];
% B_ex = [1 0; 1 1; 0 1];
% C_ex = [1 0 0; 0 1 1];
% D_ex = zeros(2, 2);
% 
% % Call the function
% [Acc, Bcc, Ccc, Dcc, T] = controllable_canonical_form_companion(A_ex, B_ex, C_ex, D_ex);