function [value, isterminal, direction] = maglev_events(~, x, params)


y1 = x(1);          % meters
y2 = x(3);          % meters

if (y2 > 0)

    disp("Y2 > 0")
end

 


% Convert to cm for gap model
y1_cm = y1 * 100;
y2_cm = y2 * 100;

gap_cm = params.yc + y2_cm - y1_cm;

min_gap_cm = params.min_gap;

value = [
    gap_cm - min_gap_cm;        % collision →  pos number : zero
    y1_cm;                        
    params.yc - y2_cm; 
];

isterminal = [ 1; 1; 1] ;    % stop simulation
direction   = [-1; -1 ; -1];  % detect crossing toward zero
end