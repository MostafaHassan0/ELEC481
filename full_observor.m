function [G_ret, results_G] = full_observor(sys, K_poles)

results_G = struct('G', {}, 'trace_J', {}, 'cond', {}, 'poles', {});

A = sys.A; 
C = sys.C; 

index = 1; 

G_poles = zeros(size(K_poles)); 

KG_ratio = linspace(2.5, 5, 10); 
min_max = linspace(2, 5, 6); 

[~, idx] = sort(real(K_poles), 'descend');

sub_idx = idx(3);

for m = 1:numel(KG_ratio)
    alpha = KG_ratio(m); 
    for n = 1:numel(min_max)
        beta = min_max(n); 
        range = linspace(1, beta, 4); 
        pole_dom = alpha*real(K_poles(sub_idx)); 

        G_poles = pole_dom.*range; 
        
        try 
            G = place(A', C', G_poles).'; %G transpose 
        catch ME
            fprintf("place() failed:\n  %s\n", ME.message);
            fprintf("  poles tried: %s\n", mat2str(G_poles,4));
            continue
        end
         Ae = A - G*C; 
        if any(real(eig(Ae)) > 0)
            continue
        end
        We = lyap(Ae, eye(size(A)));   % Lyapunov 
        cond_J  = cond(We);
        trace_J = trace(We); 
        results_G(index).G = G;
        results_G(index).poles = G_poles;
        results_G(index).trace_J = trace_J ;
        results_G(index).cond = cond_J; 
        index = index + 1;  

    end
end

if ~isempty(results_G)
    all_trace = [results_G.trace_J];
    all_cond  = [results_G.cond];
    
    Tmin = min(all_trace);
    eps_band = 0.05;
    
    S = find(all_trace <= Tmin * (1 + eps_band));
    [~, idx_rel] = min(all_cond(S));
    best_idx = S(idx_rel);
    
    G_ret     = results_G(best_idx).G;
else
    error('No feasible observer found.');
end


end