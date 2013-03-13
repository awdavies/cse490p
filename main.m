% Pre-sim cleanup.  Get model, etc.
clear
mj('clear');
mj('load', 'humanoid.mjb');
m = mj('getmodel');
mj('reset');

addpath('functions');

% global for stopping/starting walking
global MODEL_WALK;

% pd tuning parameters
params.kp = ones(19, 1) * 800;   % Spring coefficient.
params.kd = ones(19, 1) * 72;    % Damping coefficient.
params.kp(joints.LEFT_SHIN_XZ) = 1000;
params.kp(joints.RIGHT_SHIN_XZ) = 1000;
params.cd = 0.8;   % D_COM angle scale factor.
params.cv = 0.18;  % V_COM angle scale factor.

% Params for fingers.
params.kp(12:15) = 2;
params.kd(12:15) = 0.5;

% State change thresholds
THRESHOLD.swing = 0.2;  % Time in seconds to swing.
THRESHOLD.force = 0;    % Force in newtons (I think). Zero means on contact.
THRESHOLD.stable = 1;   % Time to pause between stopping/grasping
THRESHOLD.vcom = 0.01;   % The abs value vcom threshold for stopping.
THRESHOLD.dcom = -0.015;   % The min dcom before taking another step (whilst stopping).

% Initial conditions.
state = states.SWING_RIGHT;
arm_state = 0;
f = zeros(joints.TOTAL_DOF,1);
timer = 0;
MODEL_WALK = 1;

%-----------------------
%    Main Loop
%----------------------- 
for i = 1:100000
%for i = 1:14521
    mj('step1');
    [q,v,x,n,com,dt] = mj('get','qpos','qvel','geom_xpos','contact','com','dt');
    
    % Object to pass state to controller
    model.q = q;
    model.v = v;
    model.x = x;
    model.n = n;
    model.com = com;
    
    ball_proximity(x);
    
    % Setup jacobian for com.  com is in three dimensions, hence the 3.
    % converts to one 3 x m.nq matrix per body.
    J = zeros(3, m.nq, m.nbody);
    for j = 0:m.nbody-1
        J(:,:,j + 1) = mj('jacbodycom', j);
    end
        
    % Check/change state.
    [state, timer] = change_state(state, timer, model, m, J, THRESHOLD, n);
    timer = timer + dt;

    % Run controller.
    f = controller(state, m, J, state.get_target(), model, params);
    
    % Run arm controller if stable, or if we've grabbed the ball and we're
    % simply regaining our balance.
    if (state == states.STABLE_LEFT || state == states.STABLE_RIGHT || ...
      arm_state == 3)
        [u, arm_state] = arm_controller(arm_state);
        f(joints.GRASP_ARM_DOF_RANGE) = u(joints.GRASP_ARM_DOF_RANGE);
    end
    
    mj('set', 'qfrc_external', f);
    mj('step2');
    if mod(i, 20) == 0
        mjplot;
    end
end
