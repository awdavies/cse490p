%%% Constants

k1 = 75;
k2 = 10;
feet = 10;  % foot compensation factor.

%%%%%%%%%%%%%

mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

% Grab initial position (so we can stand still).
mj('kinematics');
[q_stand, y_stand] = mj('get', 'qpos', 'geom_xpos');
for i = 1:10000
    mj('kinematics');
    [q,v,x] = mj('get','qpos','qvel','geom_xpos');
    
    %----------------
    %
    %  Simple Angle Compensation.  Left and right foot
    %  are weighted to account for the upper body.
    %
    %----------------
    f = k1 *  (q_stand - q) - k2 * v;
    f(12) = k1 * 10 * (q_stand(12) - q(12)) - k2 * 10 * v(12);
    f(9) = k1 * 10 * (q_stand(9) - q(9)) - k2 * 10 * v(9);
    mj('set','qfrc_external', f);
    mj('step', 30);
    mjplot;
end
