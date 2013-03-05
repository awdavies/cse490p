function f = controller(state,m,J,target,model,params)

% Switch to world coordinates (this may end up being a bit wonky).
q_torso = model.q(joints.TORSO_XZ);
model.q(joints.RIGHT_THIGH_XZ) = model.q(joints.RIGHT_THIGH_XZ) + q_torso;
model.q(joints.LEFT_THIGH_XZ) = model.q(joints.LEFT_THIGH_XZ) + q_torso;

% Determine swing and stand joints.
switch(state)
    case {states.STAND_RIGHT, states.SWING_RIGHT}
        swing_joint = joints.RIGHT_THIGH_XZ;
        stand_joint = joints.LEFT_THIGH_XZ;
        stance_foot_x = model.x(joints.RIGHT_FOOT_XZ);
    case {states.STAND_LEFT, states.SWING_LEFT}
        swing_joint = joints.LEFT_THIGH_XZ;
        stand_joint = joints.RIGHT_THIGH_XZ;
        stance_foot_x = model.x(joints.LEFT_FOOT_XZ);
    otherwise
	      disp('WARNING: Unrecognized stance!');
end

% Determine whether we're swinging so as to apply balance
% feedback.  This will determine where to place the swing foot.
if state == states.SWING_RIGHT || state == states.SWING_LEFT
    % Find velocity of the center of mass.  This will grab all of
    % the x vectors and add them together.
    xvel = zeros(3, m.nbody);
    for i = 1:m.nbody
        xvel(:, i) = J(:,:, i) * model.v * m.body_mass(i);
    end
    v_com = sum(xvel(1, :)) / sum(m.body_mass(:));

    % Find distance from ankle to center of mass and adjust
    % target (only pay attention to the XY plane).
    stance_foot_dist = stance_foot_x - model.com(1);

    % Scales the target swing angle proportional to d_com, v_com
    balance_comp = stance_foot_dist(1) * params.cd - v_com * params.cv;
    
    % The '1' is a placeholder until the velocity is calculated.
    target(swing_joint) = target(swing_joint) + balance_comp;
end

% Do initial push control.
f = params.kp * (target - model.q) - params.kd * (model.v);

% Calculate/set torque now that we know the stance legs.
t_swing = f(swing_joint);
t_stand = f(stand_joint);

% Create a virtual target torque for the torso, then set the
% difference between the two hip torques to mitigate for the lack
% of actuation.
t_torso = params.kp * (target(joints.TORSO_XZ) - model.q(joints.TORSO_XZ)) - params.kd * model.v(3);
t_stand = -t_torso - t_swing; % <-- set virtual torque to actuators.
f(stand_joint) = t_stand;

% Set torso's external forces to zero.
f(joints.TORSO_DOF_RANGE) = zeros(joints.TORSO_DOF_RANGE(1),joints.TORSO_DOF_RANGE(end));
