function f = swing(state,qpos,qvel,xaxis,body_pref,thigh_pref,knee_pref,ankle_pref,kp,kd,cd,cv)

q_diff = zeros(9,1);
% thigh_pref += cd * d_com + cv * v_com

if (state == states.SWING_RIGHT)
    cart_body_angle = xaxis(3,1);    % ? get world angle
    cart_thigh_angle = xaxis(4,1);   % ? get world angle
    knee_angle = qpos(5);
    ankle_angle = qpos(6);
    
    % compute offsets
    body_diff = body_pref - cart_body_angle;
    body_vel = qvel(3);
    thigh_diff = thigh_pref - cart_thigh_angle;
    knee_diff = knee_pref - knee_angle;
    ankle_diff = ankle_pref - ankle_angle;
    q_diff(3:6) = [body_diff;thigh_diff;knee_diff;ankle_diff];
    
    % compute torques
    torque_body = kp * body_diff - kd * body_vel;
    torque_thigh = kp * thigh_diff - kd * thigh_vel;
    torque_stand = -1 * torque_body - torque_thigh;
    
    % compute remaining torques...
    % f = [0, 0, torque_body, torque_thigh, etc...]
else
    % reflect across body
end

end