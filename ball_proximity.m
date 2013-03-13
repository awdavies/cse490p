function [] = ball_proximity(x)

global MODEL_WALK;

if (x(20,1) - x(2,1) < pi/2)
    MODEL_WALK = 0;
end