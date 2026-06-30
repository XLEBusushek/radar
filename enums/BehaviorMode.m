classdef BehaviorMode
    % BehaviorMode  Долгосрочные поведенческие режимы цели.

    enumeration
        Cruise
        TurnToWaypoint
        ChangeAltitude
        HoverObserve
        FollowRoad
        Patrol
        AvoidBoundary
        HideLowAltitude
        Loiter
    end
end
