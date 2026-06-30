classdef TargetType
    % TargetType  Допустимые типы радиолокационных целей.
    %
    % Строковые константы используются вместо enumeration, т.к. имя False
    % конфликтует с встроенной функцией false в MATLAB.

    properties (Constant)
        False       = "False"
        Ground      = "Ground"
        AirplaneUAV = "AirplaneUAV"
        Quadcopter  = "Quadcopter"
    end

    methods (Static)
        function values = allValues()
            values = [
                TargetType.False
                TargetType.Ground
                TargetType.AirplaneUAV
                TargetType.Quadcopter
            ];
        end

        function tf = isValid(value)
            tf = any(strcmp(value, TargetType.allValues()));
        end
    end
end
