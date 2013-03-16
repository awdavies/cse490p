%{
Computes a controller to push a given site towards 
a goal ystar (in world coordinates). Here k1 and k2 
are constants where k1 is a pushing constant and k2
is a damping constant.
%}
function [ u ] = push_controller(site_index, ystar, k1, k2)

    % Compute the Jacobian and world coordinates
    % for this body.
    J = mj('jacsite', site_index);
    f = mj('get', 'site_xpos');
    f = f(site_index + 1, :).';
    
    % Get the joint velocities for the system.
    v = mj('get', 'qvel');
    
    u = k1 * (J.') * (ystar - f) - k2 * v;
end

