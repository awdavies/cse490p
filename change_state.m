function [new_state,timer] = change_state(old_state, t, threshold, contact)

% Default case
timer = t;
new_state = old_state;

switch(old_state)
    % TODO: Remove hard coded foot numbers.
    case states.SWING_LEFT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_RIGHT;
        end
    case states.SWING_RIGHT
        if t >= threshold.swing
            timer = 0;
            new_state = states.STAND_LEFT;
        end
    case states.STAND_LEFT
        for i = 1:length(contact)
            if (contact(i).obj2 == 8)
                  new_state = states.SWING_LEFT;
                  timer = 0;
            end
        end
    case states.STAND_RIGHT
        for i = 1:length(contact)
            if (contact(i).obj2 == 5)
                  new_state = states.SWING_RIGHT;
                  timer = 0;
            end
        end
    otherwise
        disp('WARNING: Unrecognized stance in change state!');
    end
end
