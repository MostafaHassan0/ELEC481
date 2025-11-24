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
    w_OS = 20;
    w_ts = 20;
    w_tp = 5;

    w_rms_y1 = 20;
    w_rms_y2 = 20;

    % ========================
    % Objective
    % ========================
    cost = ...
        w_OS * (2*OS_lower + OS_upper) + ...
        w_ts * (2*ts_lower + ts_upper) + ...
        w_tp * (tp_lower + tp_upper) + ...
        w_rms_y1 * RMS_y1_est + ...
        w_rms_y2 * RMS_y2_est;

end


function x = sanitize(x, HUGE)
    if ~isfinite(x)
        x = HUGE;
    end
end
