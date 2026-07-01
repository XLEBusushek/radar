classdef MissionType
    % MissionType  Долгосрочные типы миссий цели.

    enumeration
        PatrolRoute
        FollowRoadRoute
        InspectArea
        MoveBetweenZones
        LoiterArea
        ReturnToArea
        Idle
    end
end
