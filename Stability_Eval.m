function [tau_dom, lambda] = Stability_Eval(A)
% Extract dominant time constant, damping ratio, natural frequency
eigA = eig(A);
[~, idx] = max(real(eigA));    % index of the most unstable or slowest decaying pole

lambda = eigA(idx);

tau_dom = 1/abs(real(lambda));   % seconds (or equivalent units)


end