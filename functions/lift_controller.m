function u = lift_controller(k1, k2)
global FINGER_OFFSET
global ARM_START_SITE ARM_END_SITE FINGER1_SITE FINGER2_SITE
global ARM_UPPER_BODY ARM_LOWER_BODY FINGER1_1_BODY FINGER1_2_BODY ...
       FINGER2_1_BODY FINGER2_2_BODY BALL_BODY
global ARM_UPPER_JOINT ARM_LOWER_JOINT FINGER1_1_JOINT FINGER1_2_JOINT ...
       FINGER2_1_JOINT FINGER2_2_JOINT BALL_X_JOINT BALL_Z_JOINT

Qzero = mj('get','qpos');
Qstar = Qzero;

Qstar(ARM_UPPER_JOINT+1:ARM_LOWER_JOINT+1) = [-pi; pi/2];

%% get current state of the arm and velocity
Q = mj('get','qpos');
V = mj('get','qvel');

%% controller
u = pd_controller(Q, V, k1, k2, Qstar, 0);
u(BALL_X_JOINT+1) = 0; 
u(BALL_Z_JOINT+1) = 0; 