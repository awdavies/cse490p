% Returns the center of mass (Cartesian coordinates) as a 
% column vector for a group of consecutive body masses.
%
% Arguments:
%  m - model returned from m = mj('getmodel');
%  x0 - initial body positions, returned from mj('get', 'xpos');
%  i - index (mujoco 0-based) of first body mass
%  j - index (mujoco 0-based) of last body mass
function cm = center_of_mass(m, x0, i, j)
    % CM of each body as row vectors
    pos = x0(i+1:j+1, :); 
    
    % mass of each body as col vector
    mass = m.body_mass(i+1:j+1);
    
    cm = times(pos, mass * ones(1,3));
    cm = sum(cm, 1)' / sum(mass, 1);
end