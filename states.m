classdef states
    enumeration
        STAND_LEFT, SWING_RIGHT, STAND_RIGHT, SWING_LEFT,
    end

    properties (Constant)
        % Target swing angles
            SWING_RIGHT_TARGET = [0.0000 
              0.0000
                -0.0000 
                  -0.5498 
                    1.4769 
                      -0.5236 
                        0.0000 
                          0.0000 
                            0.0000];
  
            SWING_LEFT_TARGET = [0.0000
              0.0000
                -0.0000 
                        0.0000 
                          0.0000 
                            0.0000
                  -0.5498 
                    1.4769 
                      -0.5236];

            % Target stand angles
            STAND_LEFT_TARGET = [0.0000 
              0.0000
                0.0000
                  0.0017 
                    0.0016 
                      -0.1936 
                        0.6283 
                          0.0000 
                            0.0000];

            STAND_RIGHT_TARGET = [0.0000 
              0.0000
                0.0000 
                        0.6283 
                          0.0000 
                            0.0000
                  0.0017 
                    0.0016 
                      -0.1936];
    end
    
    
    methods
        function target = get_target(s)
            if (s == states.SWING_RIGHT)
                target = states.SWING_RIGHT_TARGET;
            elseif (s == states.SWING_LEFT)
                target = states.SWING_LEFT_TARGET;
            elseif (s == states.STAND_RIGHT)
                target = states.STAND_RIGHT_TARGET;
            else
               	target = states.STAND_LEFT_TARGET;
            end
        end
    end
end
