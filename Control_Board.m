
Baysian_search = true; 
BaysianSearchController = true; 

m = 0.12; %kg
a = 1.65; %cm
b = 6.2; %cm
c = 23984 ; %N *cm^4
d = 3.7698 ; %cm
yc = 12.7;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 5e-2; %drag minimum 
%drag = 2.5e-1; %drag maximum
min_gap = 0.25; %cm, break condition for simulation
e = 0.15; 
umax = 30000; 

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n, ...
    'drag',drag,'min_gap',min_gap, 'e', e, 'umax', umax);

if (Baysian_search == 1)

    if (BaysianSearchController == true)
        % --- Define optimization variables ---

            optimVars = [
    optimizableVariable('starting_pole',   [3, 5])              % controller dominant real part
    optimizableVariable('damping_ctl_dom', [0.5, 1.0])        % controller dominant damping
    optimizableVariable('damping_ctl_sub', [0.5, 1.0])        % controller subdominant damping
    optimizableVariable('dom_sub_ratio',   [3.5, 4.5])            % controller dom/sub real-part ratio
    
    optimizableVariable('damping_obsv_dom',[0.5, 1.0])        % observer dominant damping
    optimizableVariable('damping_obsv_sub',[0.5, 1.0])        % observer subdominant damping
    optimizableVariable('dom_sub_obsv',    [2.0, 6.0])           % observer dom/sub real-part ratio
    optimizableVariable('ratio_K_G',       [3, 5])              % observer speed relative to controller
];

        
        % --- Run Bayesian Optimization ---
        results = bayesopt(@(search_params) maglev_objective(search_params, params, BaysianSearchController), ...
                       optimVars, ...
                       'MaxObjectiveEvaluations', 100, ...
                       'IsObjectiveDeterministic', false, ...
                       'Verbose', 1);
    
        
        % --- Extract best controller parameters ---
        best_vars = bestPoint(results);
        
        % --- Run simulation with best variables ---
        metrics = Maglev_Tuner_Controller(best_vars, params);
        
        % --- Display or plot results ---
        disp(metrics);
    end
end