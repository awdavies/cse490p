% vec_angle(vec1, vec2):
%   Returns the angle between 2D vectors vec1 and vec2.
%
% vec_angle(vec1):
%   Returns the base angle that 2D vector vec1 forms.
%   (e.g. vector [1 0] forms the angle pi/4)
function theta = vec_angle(varargin)
    if (nargin == 1)
        vec1 = varargin{1};
        vec2 = zeros(size(vec1));
        vec2(1) = 1;
        vec2(2) = 0;

    elseif (nargin == 2)
        vec1 = varargin{1};
        vec2 = varargin{2};
        
    else
       error('Invalid number of arguments to vec_angle: %d\n', nargin); 
    end
    
    vec_dot = sum(vec1.*vec2);
    vec_norm_product = norm(vec1)*norm(vec2);
    theta = acos(vec_dot/vec_norm_product);
end