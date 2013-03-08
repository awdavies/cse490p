mj('clear');
mj('load', 'models/humanoid(Grasp).mjb');
m = mj('getmodel');
mj('reset');

addpath('functions');

BALL_X_JOINT = mj('getid', 'joint', 'ball_x');

q = mj('get', 'qpos');
q(BALL_X_JOINT+1) = degtorad(-25);
mj('set', 'qpos', q);

state = 0;
for i = 1:100000
    mj step1;
    [u, state] = arm_controller(state);
    mj('set', 'qfrc_external', u);
    
    
    
    
    mj step2;
    
    if (~mod(i, 20))
       mjplot; 
       drawnow;
    end
end