function [A_jordan, B_jordan, C_jordan, D_jordan] = JCF(A, B, C, D)
% JFC - Computes the Jordan canonical form of a system
% Inputs: A, B, C, D - State-space matrices
% Outputs: A_jordan, B_jordan, C_jordan, D_jordan - Jordan canonical form matrices

    % Compute Jordan form and transformation matrix
    [V, J] = jordan(A);

    % Transform system
    A_jordan = J;
    B_jordan = inv(V) * B;
    C_jordan = C * V;
    D_jordan = D;
end