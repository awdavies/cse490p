mj('clear');
mj('load', 'test.xml');
m = mj('getmodel');
mj('reset');

for i = 1:10000
    mj('kinematics');
    [q,v,x] = mj('get','qpos','qvel','geom_xpos');
    
    %----------
    f = randn(9,1)*200;
    %---------
    
    mj('set','qfrc_external', f);    
    mj('step', 35);
    mjplot;
end
