classdef MissionTypeCatalog
    % MissionTypeCatalog  Допустимые типы миссий по типу цели.

    methods (Static)
        function holdTime = minimumHoldTime(targetType)
            switch char(targetType)
                case char(TargetType.False)
                    holdTime = 10;
                case char(TargetType.Ground)
                    holdTime = 20;
                case char(TargetType.AirplaneUAV)
                    holdTime = 40;
                case char(TargetType.Quadcopter)
                    holdTime = 15;
                otherwise
                    holdTime = 10;
            end
        end

        function modes = allowedTypes(targetType)
            switch char(targetType)
                case char(TargetType.False)
                    modes = [MissionType.MoveBetweenZones, MissionType.LoiterArea, MissionType.ReturnToArea, MissionType.Idle];
                case char(TargetType.Ground)
                    modes = [MissionType.FollowRoadRoute, MissionType.ReturnToArea, MissionType.Idle];
                case char(TargetType.AirplaneUAV)
                    modes = [MissionType.PatrolRoute, MissionType.LoiterArea, MissionType.ReturnToArea, MissionType.Idle];
                case char(TargetType.Quadcopter)
                    modes = [MissionType.InspectArea, MissionType.LoiterArea, MissionType.ReturnToArea, MissionType.Idle];
                otherwise
                    modes = MissionType.Idle;
            end
        end
    end
end
