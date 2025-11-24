%% main.m  — Entry point for automated tuning

clear; clc; close all;

% --- Define optimization variables ---
optimVars = [
    optimizableVariable('starting_pole',[3,5])
    optimizableVariable('damping_ctl_dom',[0.75,0.82])
    optimizableVariable('dom_sub_ratio',[2,3])
    optimizableVariable('ratio_K_G',[3,5])
    optimizableVariable('damping_ctl_sub',[0.85,1.0])
];

% --- Run Bayesian Optimization ---
results = bayesopt(@maglev_objective, optimVars, ...
    'MaxObjectiveEvaluations', 75, ...
    'IsObjectiveDeterministic', false, ...
    'Verbose', 1);

% --- Extract best controller parameters ---
best_vars = bestPoint(results);

% --- Run simulation with best variables ---
metrics = Maglev_Tuner(best_vars);

% --- Display or plot results ---
disp(metrics);
