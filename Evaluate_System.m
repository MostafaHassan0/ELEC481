function [result, cond_obsv, cond_ctl, num12, num21] = Evaluate_System(sys)

result = false; 
cond_obsv = 0; 
cond_ctl = 0; 

sys_tf = tf(sys);
G12 = sys_tf(1,2);   % input 2 → output 1
G21 = sys_tf(2,1);   % input 1 → output 2
[num12, ~] = tfdata(G12, 'v');
[num21, ~] = tfdata(G21, 'v');


Ctl = ctrb(sys);
Obsv = obsv(sys);
rC = rank(Ctl);
rO = rank(Obsv);

if (rC ==4 && rO ==4)
    result = true; 
    cond_obsv = cond(Obsv);
    cond_ctl = cond(Ctl); 
end


end