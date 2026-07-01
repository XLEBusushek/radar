classdef BehaviorMode
    % BehaviorMode  Долгосрочные поведенческие режимы цели.

    enumeration
        Cruise
        TurnToWaypoint
        ChangeAltitude
        HoverObserve
        MoveToPoint
        AltitudeAdjust
        FollowRoad
        ApproachIntersection
        TurnAtIntersection
        CruiseAfterTurn
        Patrol
        LongCruise
        WideTurn
        AltitudeCorrection
        AvoidBoundary
        HideLowAltitude
        Loiter
    end
end
