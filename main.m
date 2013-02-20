% tuning parameters
params.kp = 0;
params.kd = 0;
params.cd = 0;
params.cv = 0;

% State change thresholds
THRESHOLD.stand = 0.7;
THRESHOLD.swing = 2.0;
THRESHOLD.force = 0;

% Target swing angles
SWING = zeros(9,1);

% Target stand angles
STAND = zeros(9,1);

mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

state = states.SWING_RIGHT;
f = zeros(9,1);
timer = 0;

for i = 1:10000
    mj('step1');
    [q,v,x,n,com,dt] = mj('get','qpos','qvel','geom_xpos','contact','com','dt');
    
    % Object to pass state to controller
    model.q = q;
    model.v = v;
    model.x = x;
    model.n = n;
    model.com = com;
    
    J = zeros(3,m.nq,m.nbody);
    for i = 0:m.nbody-1
        J(:,:,i+1) = mj('jacbodycom',i);
    end
        
    % switch states if necessary
    [state,timer] = change_state(state,timer,THRESHOLD,n);
    
    timer = timer + dt;
    
    if (state == states.SWING_RIGHT || state == states.SWING_LEFT)
        f = controller(state,m,J,SWING,model,params);
    else
        f = controller(state,m,J,STAND,model,params);
    end
    
    mj('set','qfrc_external',f);
    mj('step2');
    mjplot;
end
