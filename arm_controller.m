%{
Function to control the arm in the humanoid(Grasp).xml model.
arm_controller is initially called with the state 0, and is succesively
called with the state that was returned by the previous call. Each call
will return a controller u, that controls only joints that are part of the
arm.
%}

function [u, state] = arm_controller(state)
    m = mj('getmodel');
    
    % Site indices (0-based)
    global ARM_START_SITE ARM_END_SITE FINGER1_SITE FINGER2_SITE
    ARM_START_SITE = mj('getid', 'site', 'start_upper');
    ARM_END_SITE = mj('getid', 'site', 'end_lower');
    FINGER1_SITE = mj('getid', 'site', 'end_finger1');
    FINGER2_SITE = mj('getid', 'site', 'end_finger2');

    % Body indices (0-based)
    global ARM_UPPER_BODY ARM_LOWER_BODY FINGER1_1_BODY FINGER1_2_BODY ...
           FINGER2_1_BODY FINGER2_2_BODY BALL_BODY
    ARM_UPPER_BODY = mj('getid', 'body', 'upper');
    ARM_LOWER_BODY = mj('getid', 'body', 'lower');
    FINGER1_1_BODY = mj('getid', 'body', 'finger1_1');
    FINGER1_2_BODY = mj('getid', 'body', 'finger1_2');
    FINGER2_1_BODY = mj('getid', 'body', 'finger2_1');
    FINGER2_2_BODY = mj('getid', 'body', 'finger2_2');
    BALL_BODY = mj('getid', 'body', 'ball');

    % Geom indices (0-based)
    global ARM_UPPER_GEOM ARM_LOWER_GEOM FINGER1_1_GEOM FINGER1_2_GEOM ...
           FINGER2_1_GEOM FINGER2_2_GEOM BALL_GEOM
    ARM_UPPER_GEOM = mj('getid', 'geom', 'upper');
    ARM_LOWER_GEOM = mj('getid', 'geom', 'lower');
    FINGER1_1_GEOM = mj('getid', 'geom', 'finger1_1');
    FINGER1_2_GEOM = mj('getid', 'geom', 'finger1_2');
    FINGER2_1_GEOM = mj('getid', 'geom', 'finger2_1');
    FINGER2_2_GEOM = mj('getid', 'geom', 'finger2_2');
    BALL_GEOM = mj('getid', 'geom', 'ball');

    % Joint indices (0-based)
    global ARM_UPPER_JOINT ARM_LOWER_JOINT FINGER1_1_JOINT FINGER1_2_JOINT ...
           FINGER2_1_JOINT FINGER2_2_JOINT BALL_X_JOINT BALL_Z_JOINT
    ARM_UPPER_JOINT = mj('getid', 'joint', 'upper');
    ARM_LOWER_JOINT = mj('getid', 'joint', 'lower');
    FINGER1_1_JOINT = mj('getid', 'joint', 'finger1_1');
    FINGER1_2_JOINT = mj('getid', 'joint', 'finger1_2');
    FINGER2_1_JOINT = mj('getid', 'joint', 'finger2_1');
    FINGER2_2_JOINT = mj('getid', 'joint', 'finger2_2');
    BALL_X_JOINT = mj('getid', 'joint', 'ball_x');
    BALL_Z_JOINT = mj('getid', 'joint', 'ball_z');

    % Geometric constants
    global BALL_RADIUS FINGER_LENGTH TOTAL_LENGTH
    BALL_RADIUS = m.geom_size(BALL_GEOM+1, 1);
    FINGER_LENGTH = sum(m.geom_size(FINGER1_1_GEOM+1:FINGER1_2_GEOM+1, 2)*2);
    TOTAL_LENGTH = sum(m.geom_size(ARM_UPPER_GEOM+1:FINGER1_1_GEOM+1, 2)*2);

    % States
    ERROR = -1;     % ball not within reach
    APPROACH = 0;   % move hand toward target above ball
    GRASP = 2;      % close fingers around ball
    LIFT = 3;       % move ball/hand toward some target

    % State initialization
    mj('kinematics');
    mj('forward');
    [q,v,x,site_x,contact] = mj('get','qpos','qvel','xpos','site_xpos','contact');

    % DEBUG 
    % move ball closer to test approach/grasp on humanoid
    %q(BALL_X_JOINT+1) = degtorad(-25);
    % mj('set', 'qpos', q);

    switch state
        case APPROACH

            % Push controller spring parameters
            k1 = 50;
            k2 = 10;

            % PD controller spring parameters
            k = 10;
            b = .5;

            [u, ystar_approach] = align_controller(q, v, x, k1, k2, k, b);

        case GRASP

            k1 = 50;
            k2 = .3;

            u = grasp_controller(q, v, x, m, k1, k2);

        case LIFT

            k1 = 10;
            k2 = 5;
            u = lift_controller(k1, k2);
            u(FINGER1_1_JOINT+1 : FINGER2_2_JOINT+1) = 0;

            k1 = 50;
            k2 = .3;
            u_grasp = grasp_controller(q, v, x, m, k1, k2);
            u_grasp(1:FINGER1_1_JOINT) = 0;
            u = u + u_grasp;
    end


    % Calculate the total gravitational force and
    % counteract it.
    g = TotalGForce(m, 1:m.nbody);

    % Zero out counter-gravitational force on the non-arm bodies.
    g(1:ARM_UPPER_JOINT) = 0;

    % The ball doesn't need to counteract gravity -- if
    % if did, the initial contact force would send it
    % into space.
    g(BALL_X_JOINT+1:BALL_Z_JOINT+1) = 0;
    u = u - g;
    
    % Check for failure case:
    % If hand is not within two diameters of the ball CM: go to APPROACH
    hand_to_ball_dist = norm(site_x(ARM_END_SITE+1, :) - x(BALL_BODY+1, :));
    if (state ~= APPROACH && hand_to_ball_dist > BALL_RADIUS*4)
        state = APPROACH;
        disp('Backtrack to approach');
    end

    % Update states
    switch state
        case APPROACH

            finger_center = (site_x(FINGER1_SITE+1, :) + site_x(FINGER2_SITE+1, :)) / 2;
            hand_to_finger_dist = norm(site_x(ARM_END_SITE+1, :) - finger_center);
            if ((hand_to_finger_dist - hand_to_ball_dist) > BALL_RADIUS / 2)
                state = GRASP;
                disp('Switched to grasp');
            end

        case GRASP

            ball_contacts = [];

            for ii = 1:length(contact)
               if (contact(ii).obj1 == BALL_GEOM || contact(ii).obj2 == BALL_GEOM ...
                   && (contact(ii).obj1 ~= 0 && contact(ii).obj2 ~= 0))
                  ball_contacts = [ball_contacts contact(ii)]; 
               end
            end

            angles = zeros(length(ball_contacts), 1);
            for ii = 1:length(ball_contacts)
                ball_vec = ball_contacts(ii).pos - x(BALL_BODY + 1, :)';
                ident = [1; 0; 0];
                angles(ii) = acos(sum(ball_vec .* ident) / (norm(ident)*norm(ball_vec)));
                if (ball_vec(3) < 0)
                   angles(ii) = -angles(ii); 
                end
            end

            angles = angles + (pi / 2);

            for ii = 1:length(angles)
               if (angles(ii) < 0)
                  angles(ii) = angles(ii) + 2*pi; 
               end
            end

            angles = sort(angles);

            if (~isempty(angles))
                span = angles(end) - angles(1);
            else
                span = 0;
            end

            if (span > degtorad(210))
               state = LIFT; 
               fprintf('Switched to lifting!\n');
            end

        case LIFT

    end
end
