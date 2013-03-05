% Pre-sim cleanup.  Get model, etc.
clear
mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

% pd tuning parameters
params.kp = 480;   % Spring coefficient.
params.kd = 72;    % Damping coefficient.
params.cd = 1.8;   % D_COM angle scale factor.
params.cv = 0.14;  % V_COM angle scale factor.

% State change thresholds
THRESHOLD.swing = 0.3;  % Time in seconds to swing.
THRESHOLD.force = 0;    % Force in newtons (I think). Zero means on contact.

% Initial conditions.
state = states.SWING_RIGHT;
f = zeros(joints.TOTAL_DOF,1);
timer = 0;

%-----------------------
%    Main Loop
%-----------------------
for i = 1:100000
    mj('step1');
    [q,v,x,n,com,dt] = mj('get','qpos','qvel','geom_xpos','contact','com','dt');
    
    % Object to pass state to controller
    model.q = q;
    model.v = v;
    model.x = x;
    model.n = n;
    model.com = com;
    
    % Setup jacobian for com.  com is in three dimensions, hence the 3.
    % converts to one 3 x m.nq matrix per body.
    J = zeros(3, m.nq, m.nbody);
    for j = 0:m.nbody-1
        J(:,:,j + 1) = mj('jacbodycom', j);
    end
        
    % Check/change state.
    [state, timer] = change_state(state, timer, THRESHOLD, n);
    timer = timer + dt;

    % Run controller.
    f = controller(state, m, J, state.get_target(), model, params);

    mj('set', 'qfrc_external', f);
    mj('step2');
    if mod(i, 20) == 0
        mjplot;
    end
end
