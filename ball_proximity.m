function [] = ball_proximity(x)

global MODEL_WALK;

% If we're close enough, the stop.  If not, then keep walking.
% (the else ensures if we knock away the ball, then we're going to
% be fine).
if (x(20,1) - x(2,1) < pi/2)
    MODEL_WALK = 0;
else
    MODEL_WALK = 1;
end
