function cost = maglev_cost_function_controller(metrics)

    HUGE = 1e6;
    BIG  = 1e12;

    % If any field is invalid → catastrophic penalty
    if any(~isfinite(struct2array(metrics)))
        cost = BIG;
        return;
    end

    % Extract and sanitize
    OS_lower   = sanitize(metrics.OS_lower,    HUGE);
    OS_upper   = sanitize(metrics.OS_upper,    HUGE);

    ts_lower   = sanitize(metrics.ts_lower,    HUGE);
    ts_upper   = sanitize(metrics.ts_upper,    HUGE);

    tp_lower   = sanitize(metrics.tp_lower,    HUGE);
    tp_upper   = sanitize(metrics.tp_upper,    HUGE);

    RMS_y1_est = sanitize(metrics.RMS_y1_est,  HUGE);
    RMS_y2_est = sanitize(metrics.RMS_y2_est,  HUGE);

    % ========================
    % Weights (tunable)
    % ========================
    w_OS = 40;
    w_ts = 40;
    w_tp = 40;

    RMS= 25;

    % ========================
    % Objective
    % ========================
cost = ...
    w_OS *(OS_lower^2 + OS_upper^2) + ...
    w_ts*(ts_lower^2 + ts_upper^2) + ...
    w_tp *(tp_lower^2/2 + 2*tp_upper^2) + ...
    RMS*(RMS_y1_est^2 + RMS_y2_est^2);
end



function x = sanitize(x, HUGE)
    if ~isfinite(x)
        x = HUGE;
    end
end
