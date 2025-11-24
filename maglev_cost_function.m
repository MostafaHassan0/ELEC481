function cost = maglev_cost_function(metrics)

    huge = 1e6;

    % ===========================================================
    % 1) Reject obviously non-physical cases
    % ===========================================================
    if metrics.OS_lower < 0 || metrics.OS_upper < 0
        cost = huge;
        return;
    end

    % ===========================================================
    % 2) Smooth penalty zone (instead of hard rejection)
    % ===========================================================
    badness = 0;

    % Settling time NaN penalties
    if isnan(metrics.ts_lower)
        badness = badness + 1;
    end
    if isnan(metrics.ts_upper)
        badness = badness + 1;
    end

    % Overshoot too high penalties (> 40%)
    if metrics.OS_lower > 0.4
        badness = badness + (metrics.OS_lower - 0.4)*10;
    end
    if metrics.OS_upper > 0.4
        badness = badness + (metrics.OS_upper - 0.4)*10;
    end

    if badness > 0
        cost = 1e5 * badness;
        return;
    end

    % ===========================================================
    % 3) Extract metrics and sanitize
    % ===========================================================
    f_RMS_total  = sanitize(metrics.RMS_total,  huge);
    f_RMS_states = sanitize(3*( ...
                        metrics.RMS_y1  + ...
                        metrics.RMS_dy1 + ...
                        metrics.RMS_y2  + ...
                        metrics.RMS_dy2 ), huge);

    f_OS = sanitize(metrics.OS_lower + metrics.OS_upper, huge);

    f_ts = sanitize(metrics.ts_lower + metrics.ts_upper, huge);

    f_tp = sanitize(metrics.tp98_lower + metrics.tp98_upper, huge);

    % ===========================================================
    % 4) Weighted objective (NO undefined variables)
    % ===========================================================
    cost = ...
        7  * f_RMS_total  + ...
        3  * f_RMS_states + ...
        10 * f_OS         + ...
        2  * f_ts         + ...
        1  * f_tp;

    % ===========================================================
    % 5) dom/sub ratio penalty
    % ===========================================================
    if isfield(metrics,'dom_sub_ratio')
        if metrics.dom_sub_ratio > 3
            cost = cost + 100;
        end
    end

end

% ===========================================================
% Helper: replace inf or nan with large penalty value
% ===========================================================
function x = sanitize(x, huge)
    if ~isfinite(x)
        x = huge;
    end
end
