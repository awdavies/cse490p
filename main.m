% pd tuning parameters
params.kp = 560;
params.kd = 70;

params.cd = 1;
params.cv = 1;

% State change thresholds
THRESHOLD.stand = 0.8;
THRESHOLD.swing = pi / 10;
THRESHOLD.force = 500;



mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

state = states.SWING_RIGHT;
f = zeros(9,1);
timer = 0;

for i = 1:100000
    mj('step1');
    [q,v,x,n,com,dt] = mj('get','qpos','qvel','geom_xpos','contact','com','dt');
    
    % Object to pass state to controller
    model.q = q;
    model.v = v;
    model.x = x;
    model.n = n;
    model.com = com;
    
    J = zeros(3,m.nq,m.nbody);
    for j = 0:m.nbody-1
        J(:,:,j + 1) = mj('jacbodycom', j);
    end
        
    % switch states if necessary
    [state,timer] = change_state(state,timer,THRESHOLD,n);
    
    timer = timer + dt;
    f = controller(state,m,J,state.get_target(),model,params);
    
    mj('set','qfrc_external',f);
    mj('step2');
    if mod(i, 20) == 0
        mjplot;
    end
end
