function [ss1, ss2, ss3, ss4] = Canonical_form(A, B, C, D)

%% Create MIMO state-space system
sys = ss(A, B, C, D);

%% ------------------------------------------------
%% 1. Extract the 2x2 transfer function matrix
%% ------------------------------------------------
G = tf(sys);

G11 = G(1,1);
G12 = G(1,2);
G21 = G(2,1);
G22 = G(2,2);

TF11 = G11;
TF12 = G12;
TF21 = G21;
TF22 = G22;

ss1.tf = TF11;
ss2.tf = TF12;
ss3.tf = TF21;
ss4.tf = TF22;

%% ------------------------------------------------
%% 2. Convert each SISO channel to state-space
%% ------------------------------------------------
sys11 = ss(G11);
sys12 = ss(G12);
sys21 = ss(G21);
sys22 = ss(G22);

%% ------------------------------------------------
%% 3. CONTROLLABLE CANONICAL FORM (SISO)
%% ------------------------------------------------
[sys11_c, T11c] = compreal(sys11,'c');
[sys12_c, T12c] = compreal(sys12,'c');
[sys21_c, T21c] = compreal(sys21,'c');
[sys22_c, T22c] = compreal(sys22,'c');

%% ------------------------------------------------
%% 4. OBSERVABLE CANONICAL FORM (SISO)
%% ------------------------------------------------
[sys11_o, T11o] = compreal(sys11,'o');
[sys12_o, T12o] = compreal(sys12,'o');
[sys21_o, T21o] = compreal(sys21,'o');
[sys22_o, T22o] = compreal(sys22,'o');

%% ------------------------------------------------
%% 5. JORDAN CANONICAL FORM (theoretical)
%% ------------------------------------------------
[T11j, J11] = jordan(sys11.A);
[T12j, J12] = jordan(sys12.A);
[T21j, J21] = jordan(sys21.A);
[T22j, J22] = jordan(sys22.A);

sys11_j = ss(J11, T11j \ sys11.B, sys11.C * T11j, sys11.D);
sys12_j = ss(J12, T12j \ sys12.B, sys12.C * T12j, sys12.D);
sys21_j = ss(J21, T21j \ sys21.B, sys21.C * T21j, sys21.D);
sys22_j = ss(J22, T22j \ sys22.B, sys22.C * T22j, sys22.D);

%% ------------------------------------------------
%% 6. Package outputs (clean & clear)
%% ------------------------------------------------
ss1.tf;
ss1.original     = sys11;
ss1.controllable = sys11_c;
ss1.observable   = sys11_o;
ss1.jordan       = sys11_j;

ss2.tf;
ss2.original     = sys12;
ss2.controllable = sys12_c;
ss2.observable   = sys12_o;
ss2.jordan       = sys12_j;

ss3.tf;
ss3.original     = sys21;
ss3.controllable = sys21_c;
ss3.observable   = sys21_o;
ss3.jordan       = sys21_j;

ss4.tf;
ss4.original     = sys22;
ss4.controllable = sys22_c;
ss4.observable   = sys22_o;
ss4.jordan       = sys22_j;


end
