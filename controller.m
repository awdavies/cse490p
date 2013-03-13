function f = controller(state,m,J,target,model,params)

% Switch to world coordinates (this may end up being a bit wonky).
q_torso = model.q(joints.TORSO_XZ);
model.q(joints.RIGHT_THIGH_XZ) = model.q(joints.RIGHT_THIGH_XZ) + q_torso;
model.q(joints.LEFT_THIGH_XZ) = model.q(joints.LEFT_THIGH_XZ) + q_torso;

% Determine swing and stand joints.
switch(state)
    case {states.STAND_RIGHT, states.SWING_RIGHT, states.STOP_RIGHT, states.BEGIN_STOP_RIGHT, states.STABLE_RIGHT}
        swing_joint = joints.RIGHT_THIGH_XZ;
        stand_joint = joints.LEFT_THIGH_XZ;
        stance_foot_x = model.x(joints.RIGHT_FOOT_XZ);
    case {states.STAND_LEFT, states.SWING_LEFT, states.STOP_LEFT, states.BEGIN_STOP_LEFT, states.STABLE_LEFT}
        swing_joint = joints.LEFT_THIGH_XZ;
        stand_joint = joints.RIGHT_THIGH_XZ;
        stance_foot_x = model.x(joints.LEFT_FOOT_XZ);
    otherwise
	      disp('WARNING: Unrecognized stance!');
end

% Determine whether we're swinging so as to apply balance
% feedback.  This will determine where to place the swing foot.
if state == states.SWING_RIGHT || state == states.SWING_LEFT || state == states.STOP_RIGHT || state == states.BEGIN_STOP_RIGHT || state == states.STOP_LEFT || state == states.BEGIN_STOP_LEFT
    v_com = get_vcom(model, m, J);

    % Find distance from ankle to center of mass and adjust
    % target (only pay attention to the XY plane).
    stance_foot_dist = stance_foot_x - model.com(1);

    % Scales the target swing angle proportional to d_com, v_com
    balance_comp = stance_foot_dist(1) * params.cd - v_com * params.cv;
    
    % The '1' is a placeholder until the velocity is calculated.
    target(swing_joint) = target(swing_joint) + balance_comp;
    
end

% Do initial push control.
f = params.kp .* (target - model.q) - params.kd .* (model.v);

% Calculate/set torque now that we know the stance legs.
t_swing = f(swing_joint);
t_stand = f(stand_joint);

% Create a virtual target torque for the torso, then set the
% difference between the two hip torques to mitigate for the lack
% of actuation.
t_torso = params.kp(joints.TORSO_XZ) * (target(joints.TORSO_XZ) - model.q(joints.TORSO_XZ)) - params.kd(joints.TORSO_XZ) * model.v(joints.TORSO_XZ);
t_stand = -t_torso - t_swing; % <-- set virtual torque to actuators.
f(stand_joint) = t_stand;

% Set torso's external forces to zero.
f(joints.TORSO_DOF_RANGE) = zeros(joints.TORSO_DOF_RANGE(1),joints.TORSO_DOF_RANGE(end));

% Keeps the ball force at zero!
f(18:19) = zeros();
