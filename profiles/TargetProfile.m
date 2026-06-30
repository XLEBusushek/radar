classdef TargetProfile
    % TargetProfile  Физические параметры и ограничения типа цели.

    properties
        SpeedMin (1, 1) double
        SpeedMax (1, 1) double
        RCSMin (1, 1) double
        RCSMax (1, 1) double
        AltitudeMin (1, 1) double
        AltitudeMax (1, 1) double
        MaxTurnRate (1, 1) double   % deg/s
        MaxPitchRate (1, 1) double  % deg/s
        MaxAcceleration (1, 1) double % m/s^2
        MaxDeceleration (1, 1) double % m/s^2
        SpeedResponseTime (1, 1) double % s
        HoverSpeedMin (1, 1) double = 0
        CruiseSpeedMin (1, 1) double = NaN
        CanHover (1, 1) logical
        CanClimb (1, 1) logical
        CanDescend (1, 1) logical
    end
end
