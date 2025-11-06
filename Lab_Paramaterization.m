function [sys_final, y10, y20] = Lab_Paramaterization()

m = 0.12; %kg
a = 1.65; %cm
b = 6.2; %cm
c = 18723 ; %N *cm^4
d = 4.23 ; %cm
yc = 12;  % units cm 
g = 9.81; %m/s^2
n = 4; 
drag = 2e-2; %drag coefficient, it cant oscillate forever that's silly
min_gap = 0.25; %cm, break condition for simulation

params = struct('m',m,'a',a,'b',b,'c',c,'d',d,'yc',yc,'g',g,'n',n,'drag',drag,'min_gap',min_gap);


y10_range = 0.1:0.1:6.5; %initial try 
y20_range = -0.1:-0.1:-6.5; 

results_max = length(y10_range)*length(y20_range); 

Results = cell(results_max + 1, 10);
Results(1,:) = {"y10","y20","u10","u20","cond(o)","cond(c)", ...
                "TF12_num","TF21_num","tau_dom","dom eig"};
row = 2; 

%Lessons learned, symmetry is important. We will only hold onto symmetrical
% linearization points, with the exception of start up, if it turns into a
% situation where to get it to the linearization point is spicy, I have a
% trick. 




for i_ = 1:length(y10_range)
    for j_ = 1:length(y20_range)
        if (i_ == j_)
        [sys_eval, u10_eval, u20_eval, pass] = Create_System(params, y10_range(i_), y20_range(j_)); 
        else 
            pass = false; 
        end
        if (u10_eval < 10000 && u20_eval < 10000 && pass)
            [result, cond_obsv, cond_ctl, num12, num21] = Evaluate_System(sys_eval);
            if (result)
                [tau_dom, dom_eig] = Stability_Eval(sys_eval.A);
                Results(row, :) = {y10_range(i_), y20_range(j_), u10_eval, u20_eval, cond_obsv, cond_ctl, num12, num21, tau_dom, dom_eig}; 
               row = row + 1; 
            end
        end
    end
end

Results = Results(1:row-1, :);

writecell(Results, 'maglev_results.xlsx');

%this is hardcoded for now, but it will display the a

disp("At y10 = 0.9cm, and y20 = -0.9cm, the following is a template A matrix "); 

[sys_final, ~, ~, ~] = Create_System(params, 0.9, -0.9); 

end



