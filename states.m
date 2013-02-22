classdef states
    enumeration
        STAND_LEFT, SWING_RIGHT, STAND_RIGHT, SWING_LEFT,
    end
    
    
    methods
        function target = get_target(s)
            % Target swing angles
            SWING_RIGHT_TARGET =  [0.0000;
                                  0.0000;
                                  0.0000;
                                  -1.2043;
                                  1.2291;
                                  0.2775;
                                  -0.3398;
                                  0.0147;
                                  -0.0186];

            SWING_LEFT_TARGET =   [0.0000; 
                                  0.0000; 
                                  0.0000; 
                                  -0.3398;
                                  0.0147; 
                                  -0.0186; 
                                  -1.2043; 
                                  1.2291; 
                                  0.2775];

            % Target stand angles
            STAND_RIGHT_TARGET = [0.0000;
                                  0.0000; 
                                  0.0000; 
                                  0.0000; 
                                  0.0000; 
                                  0.0000; 
                                  0.7330; 
                                  -0.0008;
                                  -0.0012];

            STAND_LEFT_TARGET = [0.0000; 
                                  0.0000; 
                                  0.0000; 
                                  0.7330; 
                                  -0.0008; 
                                  -0.0012;
                                  0.0000; 
                                  0.0000; 
                          0.0000];
            
            if (s == states.SWING_RIGHT)
                target = SWING_RIGHT_TARGET;
            elseif (s == states.SWING_LEFT)
                target = SWING_LEFT_TARGET;
            elseif (s == states.STAND_RIGHT)
                target = STAND_RIGHT_TARGET;
            else
               	target = STAND_LEFT_TARGET;
            end
        end
    end
end