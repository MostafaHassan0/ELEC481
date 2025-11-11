sys = ss(A,B,C,D);
G = tf(sys);    % 2x2 tf matrix

% Extract SISO channels
G11 = G(1,1);
G22 = G(2,2);

%% --- Quick numeric facts
fprintf('Poles of G11:\n'); disp(pole(G11));
fprintf('Zeros of G11:\n'); disp(zero(G11));
fprintf('Poles of G22:\n'); disp(pole(G22));
fprintf('Zeros of G22:\n'); disp(zero(G22));

%% --- Routh-Hurwitz: form characteristic polynomial for "1 + K*G(s) = 0"
% For open-loop stability of L(s) we'd analyze denominator of G(s) or of 1+K*G(s)
% Define the nominal plant denominator poly (same for G11 and G22)
den = cell2mat(G11.Denominator); % returns cell with coeffs for tf object

if iscell(den), den = den{1}; end

% Routh function (defined below) - test plant polynomial stability:
fprintf('Routh table for plant denominator:\n');
RT = routh(den);
disp(RT);
if all(RT(:,1) > 0)
    fprintf('All first-column elements positive -> all roots LHP (stable).\n');
else
    fprintf('Sign changes in first column -> unstable/poles in RHP exist.\n');
end

%% --- Root locus and asymptotes / centroid calculation (SISO)
% Use G11 and G22 separately
figure;
rlocus(G11); title('Root Locus of G11 (u1 -> y1)');

figure;
rlocus(G22); title('Root Locus of G22 (u2 -> y2)');

% Asymptote centroid and angles (analytic)
[numG11, denG11] = tfdata(G11,'v');
n = length(denG11)-1;   % order of denominator
m = length(numG11)-1;   % order of numerator
q = n - m;              % number of asymptotes
centroid = (sum(roots(denG11)) - sum(roots(numG11)))/q;
angles = (2*(0:(q-1))+1)*pi/q; % asymptote angles (radians)

fprintf('G11: order n=%d, zeros m=%d, asymptotes q=%d\n', n,m,q);
fprintf('Centroid = %g\n', centroid);
fprintf('Angles (deg) = '); disp(angles*180/pi);

%% --- Find real-axis breakaway / break-in points (numerical)
% For open-loop L(s) = G11(s) (assume unity feedback), breakaway points satisfy d/ds L(s) = 0 on real axis segments
s = sym('s');
L11 = poly2sym(numG11,s)/poly2sym(denG11,s);
dLds = simplify(diff(L11,s));
% Solve numerator(dLds)==0 for real roots
num_d = numden(dLds); num_d = expand(num_d(1));
cand = double(solve(num_d==0));
cand_real = cand(imag(cand)==0); % real candidates
fprintf('Breakaway candidates (real): '); disp(cand_real');

%% --- Root-locus based PID tuning examples
% 1) PI controller: C(s) = K*(1 + 1/(Ti*s)) = K*(Ti*s+1)/(Ti*s)
Ti = 0.5;  % choose integrator time constant — tune
Cpi = tf([Ti 1],[Ti 0]);  % PI structure without K
figure; rlocus(Cpi*G11); title('RLocus of PI*C*G11 (var K)');

% 2) PD controller (derivative with filter): C(s) = K*(1 + Td*s/(Tf*Td*s+1))
Td = 0.01; Tf = 0.01;
Cpd = tf([Tf*Td 1],[Tf 1]); % approx derivative form (you will multiply by K)
figure; rlocus(Cpd*G11); title('RLocus of PD*C*G11 (var K)');

% ********************************* TUNING *******************************************
% 3) Full PID you'd usually param as pid object or series of elements:
% You can tune K after placing zeros with PI/PD as above
K = 1000; % initial gain guess — tune via rlocus or margin
C_try = K * Cpi;  % example
% ********************************* TUNING *******************************************

% Closed-loop poles for a chosen K:
CL = feedback(C_try*G11,1); fprintf('Closed-loop poles for chosen K: '); disp(pole(CL));

% Step response
figure; step(CL); title('Example Closed-loop step (G11 with chosen controller)');









%% --- Helper function: Routh-Hurwitz table (returns table with first column check)
function R = routh(p)
% p: polynomial coefficient vector starting with highest power
% returns R: matrix where each row is Routh row; first column used to check stability
n = length(p)-1;
% number of rows = n+1
m = ceil((length(p))/2);
% Build first two rows
r1 = p(1:2:end); % coefficients for s^n, s^(n-2), ...
r2 = p(2:2:end); % coefficients for s^(n-1), s^(n-3), ...
% pad rows to equal length
if length(r2) < length(r1), r2(end+1)=0; end
R = zeros(n+1, length(r1));
R(1,1:length(r1)) = r1;
R(2,1:length(r2)) = r2;
% fill subsequent rows
for i = 3:n+1
    for j = 1:(size(R,2)-1)
        a = R(i-1,1);
        b = R(i-2,1+j);
        c = R(i-2,1);
        d = R(i-1,1+j);
        R(i,j) = (a*b - c*d)/a;
    end
    % shift row if zeros appear in first column (epsilon replacement) - rudimentary
    if abs(R(i,1)) < 1e-12
        R(i,1) = 1e-12;
    end
end
end



