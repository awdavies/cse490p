function f = controller(state,m,J,target,model,params)

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
    balance_comp = stance_foot_dist(1) * params.cd + v_com * params.cv;
    
    % The '1' is a placeholder until the velocity is calculated.
    target(swing_joint) = target(swing_joint) + balance_comp;
end

% Do initial push control.
f = params.kp * (target - model.q) - params.kd * (model.v);

% Calculate/set torque now that we know the stance legs.
t_swing = f(swing_joint);
t_stand = f(stand_joint);
t_torso = params.kp * (target(joints.TORSO_XZ) - model.q(joints.TORSO_XZ)) - params.kd * model.v(3);
% t_torso = -t_stand - t_swing;
t_stand = -t_torso - t_swing;
f(stand_joint) = t_stand;

% Set torso's external forces to zero.
f(joints.TORSO_DOF_RANGE) = zeros(joints.TORSO_DOF_RANGE(1),joints.TORSO_DOF_RANGE(end));


% THIS IS BEING KEPT FOR REFERENCE AT THE MOMENT.
%
% if (state == states.SWING_RIGHT || state == states.SWING_LEFT)
%     % Find v_com
%     weighted_xvel = zeros(3,m.nbody);
%     for i = 1:m.nbody
%         weighted_xvel(:,i) = J(:,:,i) * model.v * m.body_mass(i);
%     end
%     v_com = sum(weighted_xvel(3,:));
% 
%     % Find d_com, adjust hip target
%     if (state == states.SWING_RIGHT)
%         d_com = model.com(3,1) - model.x(5,3);
%         target(7) = target(7) + params.cv * v_com + params.cd * d_com;
%     else
%         d_com = model.com(1) - model.x(8,3);
%         target(4) = target(4) + params.cv * v_com + params.cd * d_com;
%     end
%     
%     % Manually set hip torques
%     if (state == states.SWING_RIGHT)
%         swing_cart = model.q(3) + model.q(7);
%         t_body = params.kp * (target(3) - model.q(3)) - params.kd * model.v(3);
%         t_swing = params.kp * (target(7) - swing_cart) - params.kd * model.v(7);
%         t_stand = -1 * t_body - t_swing;
%         f(7) = t_swing;
%         f(4) = t_stand;
%     else
%         swing_cart = model.q(3) + model.q(4);
%         t_body = params.kp * (target(3) - model.q(3)) - params.kd * model.v(3);
%         t_swing = params.kp * (target(4) - swing_cart) - params.kd * model.v(4);
%         t_stand = -1 * t_body - t_swing;
%         f(4) = t_swing;
%         f(7) = t_stand;
%     end    
% end

% Get rotation matrix from specific index.  Used for converting to world
% coordinates rather than 
%function rot = get_rot_matrix(idx)
%    ximat = mj('get', 'ximat');
%    rot = reshape(ximat(idx, :), 3, 3);
%end
