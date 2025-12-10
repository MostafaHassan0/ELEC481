function [K_ret, results_K, K_poles] = create_control_sys(sys, params_cs, params, lin_points) % %Overshoot, rise time, settling time, 

%The assumption is that controllability and observability have already been
%evaluated

ts= params_cs.ts;
tp = params_cs.tp;
Per_OS= params_cs.Per_OS;
x_worst = params_cs.x_worst;


results_K = struct('K', {}, 'lambda_min', {}, 'poles', {});

umax = params.umax; 

A = sys.A; 
B = sys.B; 
C = sys.C; 

u0 = lin_points.u0;

x0 = lin_points.x0; 

eval_range_x = linspace(5, 2, 30); 

eval_range_y = 10; 

damping_ratio = -log(Per_OS)./(sqrt(pi^2 + (log(Per_OS))^2)); 

wd_min = pi/tp; 

delta_min = -4/ts; 

theta = acos(damping_ratio); 

Re_bounds = max(abs(wd_min/tan(theta)), abs(delta_min)); 

Re_dom = -Re_bounds - 1e-4; 

Im_dom = wd_min + 1e-4; 

index = 1; 


for n = 1: numel(eval_range_x)
    Re_sub = Re_dom * eval_range_x(n); 
    y_max = abs(Re_sub) * tan(theta); 
    y_eval = linspace(0, y_max - 1e-4, eval_range_y); 

    for j = 1:numel(y_eval)
        poles = [Re_dom + Im_dom*1i;  Re_dom - Im_dom*1i;  Re_sub + y_eval(j)*1i;  Re_sub - y_eval(j)*1i ]; 
        try
            K = place(A, B, poles);
        catch
            continue
        end

        %--------------------------------------------------------------
        %               Evaluation Metrics
        %--------------------------------------------------------------

        %Saturation 
        u_ = u0 - K*(x_worst - x0); 

        if (any(abs(u_) > umax))
            continue
        end
        Acl = A - B*K; 
        if any(real(eig(Acl)) > 0)
            continue
        end

        Wc = lyap(Acl, B*B.');
        lambda_min = min(eig(Wc)); 
        results_K(index).K = K;
        results_K(index).poles = poles;
        results_K(index).lambda_min = lambda_min;
        index = index + 1;  
    end
end

if ~isempty(results_K)
    all_lam = [results_K.lambda_min];
    [~, best_idx] = max(all_lam);

    K_ret = results_K(best_idx).K;
    K_poles = results_K(best_idx).poles;

else
    error('No feasible controllers found.');
end

end