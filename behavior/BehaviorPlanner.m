classdef BehaviorPlanner
    % BehaviorPlanner  Долгосрочное планирование поведения цели.

    methods (Static)
        function [target, command] = plan(target, environment, dt)
            arguments
                target (1, 1) RadarTargetModel
                environment (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            target = target.tickBehaviorTime(dt);

            boundaryCommand = BehaviorPlanner.buildAvoidBoundaryCommand(target, environment);
            if ~isempty(boundaryCommand)
                target = target.setBehaviorCommand(boundaryCommand);
                command = boundaryCommand;
                return;
            end

            if target.isBehaviorCommandActive() && ~BehaviorPlanner.shouldRefreshCommand(target)
                command = target.BehaviorCommand;
                return;
            end

            command = BehaviorPlanner.generateCommand(target, environment);
            if target.Type == TargetType.Quadcopter
                target = BehaviorPlanner.updateQuadcopterContext(target, command);
            end
            target = target.setBehaviorCommand(command);
        end
    end

    methods (Static, Access = private)
        function command = generateCommand(target, environment)
            switch target.Type
                case TargetType.False
                    command = BehaviorPlanner.planBird(target, environment);
                case TargetType.Ground
                    command = BehaviorPlanner.planGround(target, environment);
                case TargetType.AirplaneUAV
                    command = BehaviorPlanner.planAirplane(target, environment);
                case TargetType.Quadcopter
                    command = BehaviorPlanner.planQuadcopter(target, environment);
                otherwise
                    command = BehaviorPlanner.defaultCruiseCommand(target);
            end
        end

        function command = planBird(target, environment)
            profile = TargetProfileRegistry.getProfile(target.Type);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            if target.Position(3) < 12 && rand() < 0.35
                command = BehaviorCommand.create( ...
                    'BehaviorMode', BehaviorMode.HideLowAltitude, ...
                    'DesiredPosition', target.Position, ...
                    'DesiredHeading', target.Heading, ...
                    'DesiredSpeed', profile.SpeedMin + 0.5 * rand() * (profile.SpeedMax - profile.SpeedMin), ...
                    'DesiredAltitude', min(15, altitudeLimits(1) + 5 + 10 * rand()), ...
                    'DesiredVerticalSpeed', -1, ...
                    'HoldTime', 3 + 4 * rand(), ...
                    'Priority', 2, ...
                    'Reason', 'Bird low altitude concealment');
                return;
            end

            segmentLength = 20 + 30 * rand();
            heading = target.Heading + deg2rad(-35 + 70 * rand());
            waypoint = target.Position;
            waypoint(1) = target.Position(1) + segmentLength * cos(heading);
            waypoint(2) = target.Position(2) + segmentLength * sin(heading);

            if rand() < 0.35
                desiredAltitude = altitudeLimits(1) + rand() * min(15, altitudeLimits(2) - altitudeLimits(1));
            else
                desiredAltitude = max(5, min(40, 5 + 35 * rand()));
            end
            waypoint(3) = desiredAltitude;

            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorMode.TurnToWaypoint, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1)), ...
                'DesiredSpeed', profile.SpeedMin + rand() * (profile.SpeedMax - profile.SpeedMin), ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', 3 + 4 * rand(), ...
                'Priority', 1, ...
                'Reason', 'Bird short chaotic segment');
        end

        function command = planGround(target, environment)
            profile = TargetProfileRegistry.getProfile(target.Type);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            if ~isfield(target.MotionContext, 'RoadHeading')
                roadHeading = RoadNetwork.nearestHeading(target.Heading);
            else
                roadHeading = target.MotionContext.RoadHeading;
            end

            segmentLength = 80 + 120 * rand();
            baseAltitude = target.Position(3);
            if isfield(target.MotionContext, 'BaseAltitude')
                baseAltitude = target.MotionContext.BaseAltitude;
            end
            roadAltitude = min(max(baseAltitude, altitudeLimits(1)), altitudeLimits(2));

            waypoint = RoadNetwork.nextRoadWaypoint(target.Position, roadHeading, segmentLength, roadAltitude);
            waypoint(1) = min(max(waypoint(1), environment.XLimits(1)), environment.XLimits(2));
            waypoint(2) = min(max(waypoint(2), environment.YLimits(1)), environment.YLimits(2));

            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorMode.FollowRoad, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', roadHeading, ...
                'DesiredSpeed', profile.SpeedMin + rand() * (profile.SpeedMax - profile.SpeedMin), ...
                'DesiredAltitude', roadAltitude, ...
                'HoldTime', 5 + 5 * rand(), ...
                'Priority', 1, ...
                'Reason', 'Ground vehicle following road segment');
        end

        function command = planAirplane(target, environment)
            profile = TargetProfileRegistry.getProfile(target.Type);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            if rand() < 0.75
                mode = BehaviorMode.Patrol;
                reason = 'Airplane UAV long patrol segment';
                holdTime = 15 + 10 * rand();
            else
                mode = BehaviorMode.Cruise;
                reason = 'Airplane UAV cruise segment';
                holdTime = 12 + 8 * rand();
            end

            segmentLength = 200 + 400 * rand();
            heading = target.Heading + deg2rad(-20 + 40 * rand());
            waypoint = target.Position;
            waypoint(1) = target.Position(1) + segmentLength * cos(heading);
            waypoint(2) = target.Position(2) + segmentLength * sin(heading);
            desiredAltitude = max(80, min(300, 80 + 220 * rand()));
            desiredAltitude = min(max(desiredAltitude, altitudeLimits(1)), altitudeLimits(2));
            waypoint(3) = desiredAltitude;

            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1)), ...
                'DesiredSpeed', profile.SpeedMin + rand() * (profile.SpeedMax - profile.SpeedMin), ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', holdTime, ...
                'Priority', 1, ...
                'Reason', reason);
        end

        function command = planQuadcopter(target, environment)
            profile = TargetProfileRegistry.getProfile(target.Type);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            if isfield(target.MotionContext, 'HoverAfterWaypoint') && ...
                    target.MotionContext.HoverAfterWaypoint && ...
                    BehaviorPlanner.hasReachedWaypoint(target)
                command = BehaviorCommand.create( ...
                    'BehaviorMode', BehaviorMode.HoverObserve, ...
                    'DesiredPosition', target.Position, ...
                    'DesiredHeading', target.Heading, ...
                    'DesiredSpeed', 0.2 + 0.8 * rand(), ...
                    'DesiredAltitude', target.Position(3), ...
                    'HoldTime', 3 + 9 * rand(), ...
                    'Priority', 2, ...
                    'Reason', 'Quadcopter observation hover');
                return;
            end

            segmentLength = 30 + 120 * rand();
            heading = target.Heading + deg2rad(-50 + 100 * rand());
            waypoint = target.Position;
            waypoint(1) = target.Position(1) + segmentLength * cos(heading);
            waypoint(2) = target.Position(2) + segmentLength * sin(heading);
            desiredAltitude = max(20, min(150, 20 + 130 * rand()));
            desiredAltitude = min(max(desiredAltitude, altitudeLimits(1)), altitudeLimits(2));
            waypoint(3) = desiredAltitude;

            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorMode.TurnToWaypoint, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1)), ...
                'DesiredSpeed', profile.CruiseSpeedMin + rand() * (profile.SpeedMax - profile.CruiseSpeedMin), ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', 4 + 6 * rand(), ...
                'Priority', 1, ...
                'Reason', 'Quadcopter mission transit');
        end

        function target = updateQuadcopterContext(target, command)
            if command.BehaviorMode == BehaviorMode.TurnToWaypoint
                target.MotionContext.HoverAfterWaypoint = rand() < 0.5;
            elseif command.BehaviorMode == BehaviorMode.HoverObserve
                target.MotionContext.HoverAfterWaypoint = false;
            end
        end

        function tf = shouldRefreshCommand(target)
            if target.BehaviorTime >= target.BehaviorHoldTime
                tf = true;
                return;
            end

            transitModes = [
                BehaviorMode.TurnToWaypoint
                BehaviorMode.FollowRoad
                BehaviorMode.Patrol
                BehaviorMode.Cruise
                BehaviorMode.Loiter
            ];

            if BehaviorPlanner.hasReachedWaypoint(target) && ...
                    any(target.BehaviorMode == transitModes)
                tf = true;
                return;
            end

            tf = false;
        end

        function command = buildAvoidBoundaryCommand(target, environment)
            command = [];
            marginRatio = 0.10;
            position = target.Position;

            xSpan = diff(environment.XLimits);
            ySpan = diff(environment.YLimits);
            xMargin = marginRatio * xSpan;
            yMargin = marginRatio * ySpan;

            nearBoundary = ...
                (position(1) - environment.XLimits(1)) < xMargin || ...
                (environment.XLimits(2) - position(1)) < xMargin || ...
                (position(2) - environment.YLimits(1)) < yMargin || ...
                (environment.YLimits(2) - position(2)) < yMargin;

            if ~nearBoundary
                return;
            end

            center = [
                mean(environment.XLimits), ...
                mean(environment.YLimits), ...
                position(3)
            ];
            inwardHeading = atan2(center(2) - position(2), center(1) - position(1));
            retreatDistance = 0.15 * min(xSpan, ySpan);
            desiredPosition = position + retreatDistance * [cos(inwardHeading), sin(inwardHeading), 0];
            desiredPosition(3) = position(3);

            profile = TargetProfileRegistry.getProfile(target.Type);
            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorMode.AvoidBoundary, ...
                'DesiredPosition', desiredPosition, ...
                'DesiredHeading', inwardHeading, ...
                'DesiredSpeed', target.Speed, ...
                'DesiredAltitude', position(3), ...
                'HoldTime', 5, ...
                'Priority', 10, ...
                'Reason', 'Avoid simulation boundary');
        end

        function command = defaultCruiseCommand(target)
            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorMode.Cruise, ...
                'DesiredPosition', target.Position, ...
                'DesiredHeading', target.Heading, ...
                'DesiredSpeed', target.Speed, ...
                'DesiredAltitude', target.Position(3), ...
                'HoldTime', 5, ...
                'Priority', 1, ...
                'Reason', 'Default cruise');
        end

        function tf = hasReachedWaypoint(target)
            if isempty(target.TargetWaypoint) || any(isnan(target.TargetWaypoint))
                tf = false;
                return;
            end

            distanceXY = norm(target.TargetWaypoint(1:2) - target.Position(1:2));
            tf = distanceXY < 15;
        end
    end
end
