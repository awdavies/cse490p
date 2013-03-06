function [new_state,timer] = change_state(old_state, t, threshold, contact)

global MODEL_WALK;

% Default case
timer = t;
new_state = old_state;

% Switch state depending on the current state, and the change params.
% For this model, the finite state machine modeled is supposed to look
% this:
%
%   [stand_left]---- left foot strike --->  [swing_right]--, 
%                                                           `\
%   ^                                                        | 
%   |                                                        |
%                                                            |
% time > swing_time                                          |
%   	
%   |                                		time > swing_time 
%
% [swing_right]                                            |
%                                                         /
%   |                                                   _/
%   `----- right foot strike ----[stand_right]  <------` 
%

% Current stopping method
% 
% [swing_right] -> time > swing_time -> [begin_stop]
%                                             |
%                             [stop] <- foot strike
%
%
switch(old_state)
    % TODO: Remove hard coded foot numbers.
    case states.SWING_LEFT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_LEFT;
        end
    case states.SWING_RIGHT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_RIGHT;
            %Transition to final state 
            if MODEL_WALK == 0
                new_state = states.BEGIN_STOP;
            end
        end
    case states.STAND_LEFT
        for i = 1:length(contact)
            if (contact(i).obj2 == 10)
                  new_state = states.SWING_RIGHT;
                  timer = 0;
            end
        end
    case states.STAND_RIGHT
        for i = 1:length(contact)
            if (contact(i).obj2 == 7)
                  new_state = states.SWING_LEFT;
                  timer = 0;
            end
        end
    case states.BEGIN_STOP
        for i = 1:length(contact)
            if (contact(i).obj2 == 7)
                  new_state = states.STOP;
                  timer = 0;
            end
        end
    case states.STOP
        if MODEL_WALK == 1
            new_state = states.STAND_RIGHT;
        end
    otherwise
        disp('WARNING: Unrecognized stance in change state!');
    end
end
