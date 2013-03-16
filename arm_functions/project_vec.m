% project_vec(v, theta):
%   Returns the projection of 2D vector v onto the angle
%   theta.

function proj_v = project_vec(v, theta)
    proj_v = zeros(size(v));
    proj_v(1) = norm(v)*cos(theta);
    proj_v(2) = norm(v)*sin(theta);
end