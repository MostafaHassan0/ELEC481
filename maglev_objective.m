function cost = maglev_objective(vars, params, BaysianSearchController)

if (BaysianSearchController == true)

    metrics = Maglev_Tuner_Controller(vars, params);

    cost = maglev_cost_function_controller(metrics);
end

end