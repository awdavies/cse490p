function [new_state,timer] = change_state(old_state, t, threshold, contact)

% Default case
timer = t;
new_state = old_state;

switch(old_state)
    case states.STAND_LEFT
        if (t >= threshold.stand)
            new_state = states.SWING_RIGHT;
            timer = 0;
        end
    case states.SWING_RIGHT
        for i = 1:length(contact)
            if (contact(i).obj2 == 8)
                if (norm(contact(i).force) >= threshold.force)
                    new_state = states.STAND_RIGHT;
                    timer = 0;
                end
            elseif (t >= threshold.swing)
                new_state = states.STAND_RIGHT;
                timer = 0;
            end
        end                    
    case states.STAND_RIGHT
        if (t >= threshold.stand)
            new_state = states.SWING_LEFT;
            timer = 0;
        end
    case states.SWING_LEFT
       for i = 1:length(contact)
            if (contact(i).obj2 == 5)
                if (norm(contact(i).force) >= threshold.force)
                    new_state = states.STAND_LEFT;
                    timer = 0;
                end
            elseif (t >= threshold.swing)
                new_state = states.STAND_LEFT;
                timer = 0;
            end
        end
    otherwise
        new_state = old_state;
        timer = t;
end