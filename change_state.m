function new_state = change_state(old_state)
switch(old_state)
    case states.STAND_LEFT
        % if (balance_achieved), new_state = states.SWING_RIGHT
    case states.SWING_RIGHT
        % if (right_contact), new_state = states.STAND_RIGHT
    case states.STAND_RIGHT
        % if (balance_achieved), new_state = states.SWING_LEFT
    case states.SWING_LEFT
        % if (left_contact), new_state = states.STAND_LEFT
    otherwise
        new_state = old_state;
end