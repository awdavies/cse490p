% tuning parameters
kp = 1;
kd = 1;
cd = 1;
cv = 1;
body_pref = 0;
thigh_pref = 0;
knee_pref = 0;
ankle_pref = 0;

mj('clear');
mj('load', 'test.xml');
mj('reset');

state = states.SWING_RIGHT;
f = zeros(1,9);
for i = 1:10000
    mj('step1');
    [q,v,x,q_cart] = mj('get','qpos','qvel','geom_xpos','xaxis');
    
    % switch states if necessary
    change_state(state);
    
    if (state == states.SWING_RIGHT || state == states.SWING_LEFT)
        %f = swing(state,q,v,q_cart,body_pref,thigh_pref,knee_pref,ankle_pref,kp,kd,cd,cv)
    else
        %f = balance(state,...);
    end
    
    mj('set','qfrc_external',f);
    mj('step2');
end