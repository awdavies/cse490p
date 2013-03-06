classdef states
    enumeration
        STAND_LEFT, SWING_RIGHT, STAND_RIGHT, SWING_LEFT, BEGIN_STOP, STOP
    end

    properties (Constant)
        
            % Target stand angles
            STAND_TARGET = [0.0000 
                            0.0000
                            0.0000 
                            -0.3142 
                              0.2873 
                              0.0262 
                              0.1309 
                              -0.0101 
                              -0.0838 
                            0
                            0
                            0
                            0
                            0
                            0
                            0.4163
                            0.1963 
                            0
                            0];
                        
           SLOW_DOWN_TARGET = [0.0000 
                            0.0000
                            0.0000 
                           -0.6212 
                            0.4064 
                            0.0890 
                            0.2618 
                           -0.0349 
                           -0.2094 
                            0
                            0
                            0
                            0
                            0
                            0
                            0.4163
                            0.1963 
                            0
                            0];
            
            % Target swing angles
            SWING_RIGHT_TARGET = [0.0000 
              0.0000
                0.0000 
                  -0.5498 
                    1.4769 
                      -0.5236 
                        0.0000 
                          0.0000 
                            0.0000
                            0
                            0
                            0
                            0
                            0
                            0
                      0.4163
                      0.1963 
                            0
                            0];
  
            SWING_LEFT_TARGET = [0.0000
              0.0000
                -0.0000 
                        0.0000 
                          0.0000 
                            0.0000
                  -0.5498 
                    1.4769 
                      -0.5236
                      0
                      0
                      0
                      0
                      0
                      0
                      0.4163
                      0.1963 
                      0
                      0];

            % Target stand angles
            STAND_LEFT_TARGET = [0.0000 
              0.0000
                0.0000
                  0.0017 
                    0.0016 
                      -0.1936 
                        0.6283 
                          0.0000 
                            0.0000
                            0
                            0
                            0
                            0
                            0
                            0
                      0.4163
                      0.1963 
                            0
                            0];

            STAND_RIGHT_TARGET = [0.0000 
              0.0000
                0.0000 
                        0.6283 
                          0.0000 
                            0.0000
                  0.0017 
                    0.0016 
                      -0.1936
                      0
                      0
                      0
                      0
                      0
                      0
                      0.4163
                      0.1963 
                      0
                      0];
    end
    
    
    methods
        function target = get_target(s)
            if (s == states.STOP)
                target = states.STAND_TARGET;
            elseif (s == states.BEGIN_STOP)
                target = states.SLOW_DOWN_TARGET;
            elseif (s == states.SWING_RIGHT)
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
