function u = approach_controller(q, v, x, ystar, k1, k2)
    global ARM_START_SITE ARM_END_SITE FINGER1_SITE FINGER2_SITE
    global ARM_UPPER_BODY ARM_LOWER_BODY FINGER1_1_BODY FINGER1_2_BODY ...
           FINGER2_1_BODY FINGER2_2_BODY BALL_BODY
    
    % Push controller constants
%     k1 = 100;
%     k2 = 100;
    
    % PD controller constants
    k = 10;
    b = .5;
    
    u = zeros(length(q), 1);
    ret = push_controller(ARM_END_SITE, ystar, k1, k2);
    % Only apply forces to arm joints
    u(ARM_UPPER_BODY+1:ARM_LOWER_BODY+1) = ret(ARM_UPPER_BODY+1:ARM_LOWER_BODY+1);
    
    
%     % Apply force to fingers to rotate
%     % TODO calculate appropriate angles to orient fingers toward ball
%     goal_hand_joints = [pi/2; 0; pi/2; 0]; %ones(4,1) * pi/4;
%     
%     % Get controller for fingers to keep them at 
%     % a specified angle.
%     hand_q = q(end-5:end-2);
%     hand_v = v(end-5:end-2);
%     v_plan = [0; 0; 0; 0];
%     u_hand = pd_controller(hand_q, hand_v, k, b, goal_hand_joints, v_plan);
%     
%     % Pad the hand controller with zeroes for the arm and ball forces.
%     u_hand = [0; 0; u_hand; 0; 0];
%     
%     
%     u = u;% + u_hand;
end