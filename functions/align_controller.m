function [u, ystar] = align_controller(q, v, x, k1, k2, k, b)
    global BALL_RADIUS
    global ARM_START_SITE ARM_END_SITE FINGER1_SITE FINGER2_SITE
    global ARM_UPPER_BODY ARM_LOWER_BODY FINGER1_1_BODY FINGER1_2_BODY ...
           FINGER2_1_BODY FINGER2_2_BODY BALL_BODY
    global ARM_UPPER_JOINT ARM_LOWER_JOINT FINGER1_1_JOINT FINGER1_2_JOINT ...
           FINGER2_1_JOINT FINGER2_2_JOINT BALL_X_JOINT BALL_Z_JOINT

%     % Push controller spring parameters
%     k1 = 50;
%     k2 = 10;
% 
%     % PD controller spring parameters
%     k = 10;
%     b = .5;

    % Make sure that the wrist stays aligned with the ball (i.e. the
    % two fingers stay equidistant from the vector between the wrist
    % and the ball).
    sites_xpos = mj('get', 'site_xpos');
    arm_vec = (sites_xpos(ARM_END_SITE+1, :) - x(ARM_LOWER_BODY+1, :))';
    alignment_vec = (x(BALL_BODY+1, :) - sites_xpos(ARM_END_SITE+1, :))';
    angle = acos(sum(arm_vec.*alignment_vec)/(norm(arm_vec)*norm(alignment_vec)));
    
    % Arm orientation correction
    arm_cross = cross(arm_vec, alignment_vec);
    if arm_cross(2) < 0
        angle = -angle;
    end
    
    % Approach along the distance between wrist and ball
    ystar = (x(BALL_BODY + 1, :)' - (alignment_vec * (BALL_RADIUS) / norm(alignment_vec)));
    
    % Return value
    u = zeros(length(q), 1);
    
    u_wrist = push_controller(ARM_END_SITE, ystar, k1, k2);
    % only set the arm joints
    u(ARM_UPPER_JOINT+1:ARM_LOWER_JOINT+1) = u_wrist(ARM_UPPER_JOINT+1:ARM_LOWER_JOINT+1);

    u_align = pd_controller(q(FINGER1_1_JOINT+1:FINGER1_2_JOINT+1), v(FINGER1_1_JOINT+1:FINGER1_2_JOINT+1), k, b, [angle; 0], [0; 0]);
    % only set finger joints
    u(FINGER1_1_JOINT+1:FINGER1_2_JOINT+1) = u_align;
    
    
    u_align = pd_controller(q(FINGER2_1_JOINT+1:FINGER2_2_JOINT+1), v(FINGER2_1_JOINT+1:FINGER2_2_JOINT+1), k, b, [angle; 0], [0; 0]);
    % only set finger joints
    u(FINGER2_1_JOINT+1:FINGER2_2_JOINT+1) = u_align;

end