function [new_state,timer] = change_state(old_state, t, model, m, J, threshold, contact)

global MODEL_WALK;

% Default case
timer = t;
new_state = old_state;

if MODEL_WALK == 0
  % Get vcom and biggest dcom.  front/back foot don't matter.
  front_foot = joints.RIGHT_FOOT_XZ;
  back_foot = joints.LEFT_FOOT_XZ;
  ff_dcom = front_foot - model.com(1);
  bf_dcom = back_foot - model.com(1);
  dcom = min(abs(ff_dcom), abs(bf_dcom));
end

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
% [swing_left]                                            |
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

%%%% WALKING STATES.
    % TODO: Remove hard coded foot numbers.
    case states.SWING_LEFT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_LEFT;
            %Transition to final state 
            if MODEL_WALK == 0
                new_state = states.BEGIN_STOP_LEFT;
            end
        end
    case states.SWING_RIGHT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_RIGHT;
            %Transition to final state 
            if MODEL_WALK == 0
                new_state = states.BEGIN_STOP_RIGHT;
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

%%%% STOPPING STATES.
    case states.BEGIN_STOP_RIGHT
        for i = 1:length(contact)
            if (contact(i).obj2 == 7)
                  new_state = states.STOP_RIGHT;
                  timer = 0;
            end
        end
    case states.BEGIN_STOP_LEFT
        for i = 1:length(contact)
            if (contact(i).obj2 == 10)
                  new_state = states.STOP_LEFT;
                  timer = 0;
            end
        end
    case states.STOP_RIGHT
        if MODEL_WALK == 1
            new_state = states.STAND_RIGHT;
        end

        if dcom < threshold.dcom
            new_state = states.SWING_LEFT;
        end
        vcom = abs(get_vcom(model, m, J));
        disp(vcom);
        if vcom < threshold.vcom
            new_state = states.STABLE_RIGHT;
        end
    case states.STOP_LEFT
        if MODEL_WALK == 1
            new_state = states.STAND_LEFT;
        end

        if dcom < threshold.dcom
            new_state = states.SWING_RIGHT;
        end
        vcom = abs(get_vcom(model, m, J));
        if vcom < threshold.vcom
            new_state = states.STABLE_LEFT;
        end
    case states.STABLE_RIGHT
        if MODEL_WALK == 1
            new_state = states.STAND_RIGHT;
        end
    case states.STABLE_LEFT
        if MODEL_WALK == 1
            new_state = states.STAND_LEFT;
        end
    otherwise
        disp('WARNING: Unrecognized stance in change state!');
    end
end
