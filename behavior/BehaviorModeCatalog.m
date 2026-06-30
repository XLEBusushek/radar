classdef BehaviorModeCatalog
    % BehaviorModeCatalog  Допустимые режимы поведения по типу цели.

    methods (Static)
        function modes = allowedModes(targetType)
            switch char(targetType)
                case char(TargetType.False)
                    modes = [
                        BehaviorMode.Cruise
                        BehaviorMode.TurnToWaypoint
                        BehaviorMode.ChangeAltitude
                        BehaviorMode.HideLowAltitude
                        BehaviorMode.AvoidBoundary
                    ];
                case char(TargetType.Ground)
                    modes = [
                        BehaviorMode.FollowRoad
                        BehaviorMode.ApproachIntersection
                        BehaviorMode.TurnAtIntersection
                        BehaviorMode.CruiseAfterTurn
                        BehaviorMode.TurnToWaypoint
                        BehaviorMode.AvoidBoundary
                    ];
                case char(TargetType.AirplaneUAV)
                    modes = [
                        BehaviorMode.Cruise
                        BehaviorMode.LongCruise
                        BehaviorMode.WideTurn
                        BehaviorMode.AltitudeCorrection
                        BehaviorMode.Patrol
                        BehaviorMode.TurnToWaypoint
                        BehaviorMode.ChangeAltitude
                        BehaviorMode.AvoidBoundary
                        BehaviorMode.Loiter
                    ];
                case char(TargetType.Quadcopter)
                    modes = [
                        BehaviorMode.Cruise
                        BehaviorMode.TurnToWaypoint
                        BehaviorMode.ChangeAltitude
                        BehaviorMode.HoverObserve
                        BehaviorMode.AvoidBoundary
                        BehaviorMode.Loiter
                    ];
                otherwise
                    modes = BehaviorMode.Cruise;
            end
        end

        function tf = isAllowed(targetType, mode)
            tf = any(BehaviorModeCatalog.allowedModes(targetType) == mode);
        end
    end
end
