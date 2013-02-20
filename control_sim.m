%%% Constants

k1 = 44;
k2 = 22;

%%%%%%%%%%%%%

mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

% Grab initial position (so we can stand still).
f = zeros(9, 1);  % default external force (nothing).
mj('kinematics');
[q_stand, y_stand] = mj('get', 'qpos', 'geom_xpos');
for i = 1:1000
    mj('kinematics');
    [q,v,x] = mj('get','qpos','qvel','geom_xpos');

    %----------------
    % Compensate for any changes in q_stand by some amount.
    % only applies forces to joints.
    %----------------
    f(4:9) = k1 *  (q_stand(4:9) - q(4:9)) - k2 * v(4:9);
    mj('set','qfrc_external', f);
    mj('step', 15);
    mjplot;
end
