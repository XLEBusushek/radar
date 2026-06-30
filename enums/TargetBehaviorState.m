classdef TargetBehaviorState
    % TargetBehaviorState  Допустимые состояния поведения цели.

    enumeration
        FlyStraight
        TurnLeft
        TurnRight
        Climb
        Descend
        Hover
        SlowDown
        SpeedUp
        Hidden
    end
end
