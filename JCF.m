function [Aj, Bj, Cj, Dj, V] = JCF(A, B, C, D)
% JORDAN_CANONICAL_FORM_MODAL Computes the Modal Form realization of a
% state-space system, which is the numerically robust equivalent of the JCF.
%
% This function uses 'modalreal' (or 'canon' with 'modal' type) to transform
% the A matrix into a block-diagonal form (JCF for distinct eigenvalues).
%
% Input Arguments:
%   A, B, C, D: State-space matrices of the system
%
% Output Arguments:
%   Aj, Bj, Cj, Dj: State-space matrices in modal/Jordan form
%   V: State transformation matrix (such that x_jcf = V * x)

    % 1. Create the state-space model object
    sys = ss(A, B, C, D);

    % 2. Compute the modal form (numerically robust JCF)
    % The output sys_jcf has A_jcf = V * A * inv(V).
    [jsys, V] = modalreal(sys); 

    % 3. Extract the transformed matrices
    Aj = jsys.A;
    Bj = jsys.B;
    Cj = jsys.C;
    Dj = jsys.D;

    % 4. Display the results
    disp('Modal (Jordan-like) Form Matrices:');
    disp('Aj (Block Diagonal):'); disp(Aj);
    disp('Bj ='); disp(Bj);
    disp('Cj ='); disp(Cj);
    disp('Dj ='); disp(Dj);
    disp('Transformation Matrix V (x_jcf = V * x) ='); disp(V);
    
    % Check the structure: Aj should be block-diagonal (Jordan structure)
    disp('---');
    disp('Eigenvalues (Diagonal of Aj):'); disp(diag(Aj));
end

% --- Example Usage (Using the same MIMO matrices) ---
% A_ex = [-1 0 0; 0 -2 0; 0 0 -3]; % Already diagonal (distinct eigenvalues)
% B_ex = [1 0; 1 1; 0 1];
% C_ex = [1 0 0; 0 1 1];
% D_ex = zeros(2, 2);
% 
% % Call the function
% [Aj, Bj, Cj, Dj, V] = jordan_canonical_form_modal(A_ex, B_ex, C_ex, D_ex);