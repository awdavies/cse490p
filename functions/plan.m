% Computes a plan for the position and velocity of the 2-link arm
% for every timestep.

function [q_plan, v_plan] = plan(qstar, q0, steps, dt)
    % Gaussian parameters
    T = steps * dt; % duration
    m = T/2;
    s = T/4;
    
    % distance between target and initial position (joint space)
    d = norm(qstar - q0);
    
    % times at which to compute qstar and vstar
    %steps = T/dt;
    t = (0:steps)*dt;
    
    % speed profile
    speed = exp(-.5*(t-m).^2/s.^2);
    speed = speed - min(speed);
    speed = speed * d / sum(speed*dt);
    
    pos = cumsum(speed*dt);
    
    diff = (qstar - q0) / d;
    q_plan = diff * pos;%qstar * ones(1, steps+1) + diff * pos;
    for i=1:steps
        v_plan(:,i) = (q_plan(:,i+1) - q_plan(:,i)) / dt;
    end
    
end