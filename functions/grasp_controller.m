function u = grasp_controller(q, v, x, m, k1, k2)
    global FINGER_OFFSET
    global ARM_START_SITE ARM_END_SITE FINGER1_SITE FINGER2_SITE
    global ARM_UPPER_BODY ARM_LOWER_BODY FINGER1_1_BODY FINGER1_2_BODY ...
           FINGER2_1_BODY FINGER2_2_BODY BALL_BODY
    global ARM_UPPER_JOINT ARM_LOWER_JOINT FINGER1_1_JOINT FINGER1_2_JOINT ...
           FINGER2_1_JOINT FINGER2_2_JOINT BALL_X_JOINT BALL_Z_JOINT
    global ARM_UPPER_GEOM ARM_LOWER_GEOM FINGER1_1_GEOM FINGER1_2_GEOM ...
       FINGER2_1_GEOM FINGER2_2_GEOM BALL_GEOM

%     k1 = 10000;
%     k2 = .1;
   
    BALL_SIZE = m.geom_size(BALL_GEOM + 1, 1); % TODO use mj constant
    FINGER_SIZE = m.geom_size(FINGER1_1_GEOM + 1, 1);
    
    sites_xpos = mj('get', 'site_xpos');
   
    % Offset vector with respect to the far edge of the ball, where
    % we want fingerstips to end up during approach.
    FINGER_BALL_OFFSET_VEC = [(BALL_SIZE + FINGER_SIZE) 0 0]; 
   
    alignment_vec = (x(BALL_BODY+1, :) - sites_xpos(ARM_END_SITE+1, :))';
    alignment_theta = vec_angle(-alignment_vec);
    
    offset_vec1 = project_vec([FINGER_BALL_OFFSET_VEC(1) FINGER_BALL_OFFSET_VEC(3)],  alignment_theta + (pi/2));
    offset_vec2 = project_vec([FINGER_BALL_OFFSET_VEC(1) FINGER_BALL_OFFSET_VEC(3)],  alignment_theta - (pi/2));

    end_of_ball = x(BALL_BODY + 1, :) + (alignment_vec' * BALL_SIZE / (norm(alignment_vec)));

    ystar_finger1 = (end_of_ball + [offset_vec1(1) 0 offset_vec1(2)])';
    ystar_finger2 = (end_of_ball + [offset_vec2(1) 0 offset_vec2(2)])';

    ystar_wrist = (x(BALL_BODY + 1, :)' - (alignment_vec * (BALL_SIZE) / norm(alignment_vec)));
    
    %{
    ystar_finger1 = (x(BALL_BODY + 1, :) + [0 0 -BALL_SIZE])';
    ystar_finger2 = (x(BALL_BODY + 1, :) + [0 0 -BALL_SIZE])';
    ystar_hand =    (x(BALL_BODY + 1, :) + [0 0  BALL_SIZE])';
    %}
    
    

    % Get the controllers for both of the finger tips
    u_finger1 = push_controller(FINGER1_SITE, ystar_finger1, k1, k2);
    u_finger1(FINGER2_2_JOINT) = 0;

    u_finger2 = push_controller(FINGER2_SITE, ystar_finger2, k1, k2);
    u_finger2(FINGER1_2_JOINT) = 0; 

    u_hand = push_controller(ARM_END_SITE, ystar_wrist, 5*k1, k2);

    u = u_finger1 + u_finger2 + u_hand;
    u(1:ARM_UPPER_JOINT) = 0;
    u(BALL_X_JOINT + 1) = 0;
    u(BALL_Z_JOINT + 1) = 0;
end