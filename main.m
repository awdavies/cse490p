clear % Clear out everything!

% pd tuning parameters
params.kp = 860;
params.kd = 72;

params.cd = 2.0;
params.cv = 0.00;

% State change thresholds
THRESHOLD.stand = 0.1;
THRESHOLD.swing = 0.18;
THRESHOLD.force = 0;

mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

state = states.SWING_RIGHT;
f = zeros(joints.TOTAL_DOF,1);
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
    
    % Ignore the DOF regarding the position of the torso.
    % These numbers should always be in the beginning of the model
    % file for portability reasons.
    J = zeros(3, m.nq, m.nbody);
    for j = 0:m.nbody-1
        J(:,:,j + 1) = mj('jacbodycom', j);
    end
        
    % switch states if necessary
    [state, timer] = change_state(state, timer, THRESHOLD, n);
    
    timer = timer + dt;

    % Switch to world coordinates (this may end up being a bit wonky).
    q_torso = model.q(joints.TORSO_XZ);
    model.q(joints.RIGHT_THIGH_XZ) = model.q(joints.RIGHT_THIGH_XZ) - q_torso;
    model.q(joints.LEFT_THIGH_XZ) = model.q(joints.LEFT_THIGH_XZ) - q_torso;

    f = controller(state, m, J, state.get_target(), model, params);
    
    mj('set', 'qfrc_external', f);
    mj('step2');
    if mod(i, 20) == 0
        mjplot;
    end
end
