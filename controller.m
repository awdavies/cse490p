function f = controller(state,m,J,target,model,params)

f = params.kp * (target - model.q) - params.kd * (model.v);
f(1:3) = zeros(1,3);

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
%         d_com = model.com(3,1) - model.x(8,3);
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