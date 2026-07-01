classdef BehaviorPlanner
    % BehaviorPlanner  Долгосрочное планирование поведения цели.

    methods (Static)
        function [target, command] = plan(target, environment, missionCommand, dt)
            arguments
                target (1, 1) RadarTargetModel
                environment (1, 1) struct
                missionCommand (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            target = target.tickBehaviorTime(dt);

            boundaryCommand = BehaviorPlanner.buildAvoidBoundaryCommand(target, environment);
            if ~isempty(boundaryCommand)
                target = target.setBehaviorCommand(boundaryCommand);
                command = boundaryCommand;
                return;
            end

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.Status == MissionStatus.Executing
                shouldRefreshMissionCommand = true;
                if target.isBehaviorCommandActive() && ...
                        BehaviorPlanner.isMissionAlignedBehavior(target.BehaviorCommand, missionCommand)
                    if target.Type == TargetType.Ground && ...
                            missionCommand.MissionType == MissionType.FollowRoadRoute
                        refreshedCommand = BehaviorPlanner.commandFromMission( ...
                            target, missionCommand, environment);
                        if refreshedCommand.BehaviorMode == target.BehaviorCommand.BehaviorMode && ...
                                strcmp(refreshedCommand.Reason, target.BehaviorCommand.Reason)
                            shouldRefreshMissionCommand = false;
                        end
                    elseif target.Type == TargetType.AirplaneUAV && ...
                            missionCommand.MissionType == MissionType.PatrolRoute
                        refreshedCommand = BehaviorPlanner.commandFromMission( ...
                            target, missionCommand, environment);
                        if refreshedCommand.BehaviorMode == target.BehaviorCommand.BehaviorMode && ...
                                strcmp(refreshedCommand.Reason, target.BehaviorCommand.Reason)
                            shouldRefreshMissionCommand = false;
                        end
                    elseif target.Type == TargetType.Quadcopter && ...
                            missionCommand.MissionType == MissionType.InspectArea
                        refreshedCommand = BehaviorPlanner.commandFromMission( ...
                            target, missionCommand, environment);
                        if refreshedCommand.BehaviorMode == target.BehaviorCommand.BehaviorMode && ...
                                isfield(refreshedCommand, 'InspectionPhase') && ...
                                isfield(target.BehaviorCommand, 'InspectionPhase') && ...
                                refreshedCommand.InspectionPhase == target.BehaviorCommand.InspectionPhase && ...
                                strcmp(refreshedCommand.Reason, target.BehaviorCommand.Reason)
                            shouldRefreshMissionCommand = false;
                        end
                    elseif target.Type == TargetType.False && ...
                            missionCommand.MissionType == MissionType.MoveBetweenZones
                        refreshedCommand = BehaviorPlanner.commandFromMission( ...
                            target, missionCommand, environment);
                        if refreshedCommand.BehaviorMode == target.BehaviorCommand.BehaviorMode && ...
                                isfield(refreshedCommand, 'BirdPhase') && ...
                                isfield(target.BehaviorCommand, 'BirdPhase') && ...
                                refreshedCommand.BirdPhase == target.BehaviorCommand.BirdPhase && ...
                                strcmp(refreshedCommand.Reason, target.BehaviorCommand.Reason)
                            shouldRefreshMissionCommand = false;
                        end
                    else
                        shouldRefreshMissionCommand = false;
                    end
                end

                if ~shouldRefreshMissionCommand
                    command = target.BehaviorCommand;
                    return;
                end

                command = BehaviorPlanner.commandFromMission(target, missionCommand, environment);
                target = target.setBehaviorCommand(command);
                return;
            end

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.Status == MissionStatus.Paused
                if target.isBehaviorCommandActive() && ...
                        strcmp(target.BehaviorCommand.Reason, 'Mission paused')
                    command = target.BehaviorCommand;
                    return;
                end
                command = BehaviorPlanner.safeIdleCommand(target, 'Mission paused');
                target = target.setBehaviorCommand(command);
                return;
            end

            if MissionCommand.isActive(missionCommand) && ...
                    (missionCommand.Status == MissionStatus.Created || ...
                    missionCommand.Status == MissionStatus.Planning || ...
                    MissionStateMachine.isTerminal(missionCommand))
                if target.isBehaviorCommandActive() && ...
                        strcmp(target.BehaviorCommand.Reason, 'Mission not executing')
                    command = target.BehaviorCommand;
                    return;
                end
                command = BehaviorPlanner.safeIdleCommand(target, 'Mission not executing');
                target = target.setBehaviorCommand(command);
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
        function command = safeIdleCommand(target, reason)
            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorPlanner.safeIdleMode(target.Type), ...
                'DesiredPosition', [nan, nan, nan], ...
                'DesiredHeading', target.Heading, ...
                'DesiredSpeed', target.Speed, ...
                'DesiredAltitude', target.Position(3), ...
                'HoldTime', 8, ...
                'Priority', 1, ...
                'Reason', reason);
        end

        function mode = safeIdleMode(targetType)
            switch char(targetType)
                case char(TargetType.Ground)
                    mode = BehaviorMode.CruiseAfterTurn;
                otherwise
                    mode = BehaviorMode.Cruise;
            end
        end

        function command = groundMissionCommand(target, missionCommand, environment)
            waypoint = missionCommand.CurrentWaypoint;
            context = target.MotionContext;

            if ~isfield(context, 'CruiseSpeed')
                stream = RandStream('mt19937ar', 'Seed', round(target.ID + 17));
                context.CruiseSpeed = 15 + 10 * stream.rand();
                context.ApproachSpeed = 8 + 4 * stream.rand();
                context.TurnSpeed = 5 + 3 * stream.rand();
            end

            distToWaypoint = norm(target.Position(1:2) - waypoint(1:2));
            desiredHeading = atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1));

            headingError = abs(MotionKinematics.wrapAngle(desiredHeading - target.Heading));
            graph = RoadGraph.fromEnvironment(environment);
            distToIntersection = RoadGraph.distanceToNearestIntersection(graph, target.Position);
            cornerAngle = BehaviorPlanner.cornerTurnAngle(missionCommand.MissionRoute, ...
                missionCommand.CurrentWaypointIndex);

            if (distToIntersection < 80 && distToWaypoint < 90 && cornerAngle > deg2rad(8))
                mode = BehaviorMode.ApproachIntersection;
                desiredSpeed = context.ApproachSpeed;
                reason = 'Ground mission approach intersection';
            elseif distToIntersection < 25 && (cornerAngle > deg2rad(12) || headingError > deg2rad(12)) && ...
                    target.Speed <= context.ApproachSpeed + 2
                mode = BehaviorMode.TurnAtIntersection;
                desiredSpeed = context.TurnSpeed;
                reason = 'Ground mission turn at intersection';
            else
                mode = BehaviorMode.FollowRoad;
                desiredSpeed = context.CruiseSpeed;
                reason = 'Ground mission drive';
            end

            terrainHeight = environment.Terrain.Height(waypoint(1), waypoint(2));
            desiredAltitude = terrainHeight + 0.8;

            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', max(6, min(12, missionCommand.MissionHoldTime)), ...
                'Priority', missionCommand.MissionPriority, ...
                'Reason', ['Mission: ' reason]);
        end

        function command = airplaneMissionCommand(target, missionCommand, environment)
            waypoint = missionCommand.CurrentWaypoint;
            turnStartDistance = 120;
            if isfield(missionCommand, 'TurnStartDistance')
                turnStartDistance = missionCommand.TurnStartDistance;
            end

            distToWaypoint = norm(target.Position(1:2) - waypoint(1:2));
            steerWaypoint = waypoint;
            if distToWaypoint < turnStartDistance && all(isfinite(missionCommand.NextWaypoint))
                exitDelta = missionCommand.NextWaypoint(1:2) - waypoint(1:2);
                if norm(exitDelta) > 1
                    exitDir = exitDelta / norm(exitDelta);
                    leadDistance = min(turnStartDistance, 120);
                    steerWaypoint = [ ...
                        waypoint(1:2) + leadDistance * exitDir, ...
                        waypoint(3)];
                end
                mode = BehaviorMode.WideTurn;
                reason = 'Airplane mission wide turn';
            else
                mode = BehaviorMode.LongCruise;
                reason = 'Airplane mission long cruise';
            end

            desiredHeading = atan2( ...
                steerWaypoint(2) - target.Position(2), ...
                steerWaypoint(1) - target.Position(1));

            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', steerWaypoint, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', missionCommand.DesiredMissionSpeed, ...
                'DesiredAltitude', missionCommand.DesiredMissionAltitude, ...
                'HoldTime', max(10, min(20, missionCommand.MissionHoldTime)), ...
                'Priority', missionCommand.MissionPriority, ...
                'Reason', ['Mission: ' reason]);
        end

        function command = quadcopterMissionCommand(target, missionCommand, environment)
            phase = missionCommand.InspectionPhase;
            holdTime = max(10, min(20, missionCommand.MissionHoldTime));

            switch phase
                case InspectionPhase.MoveToPoint
                    mode = BehaviorMode.MoveToPoint;
                    waypoint = missionCommand.CurrentWaypoint;
                    desiredSpeed = missionCommand.DesiredMissionSpeed;
                    desiredAltitude = waypoint(3);
                    reason = 'Quadcopter mission move to point';

                case InspectionPhase.MoveToNextPoint
                    mode = BehaviorMode.MoveToPoint;
                    waypoint = missionCommand.CurrentWaypoint;
                    desiredSpeed = missionCommand.DesiredMissionSpeed;
                    desiredAltitude = waypoint(3);
                    reason = 'Quadcopter mission move to next point';

                case InspectionPhase.HoverObserve
                    mode = BehaviorMode.HoverObserve;
                    waypoint = missionCommand.InspectionHoverPosition;
                    desiredSpeed = missionCommand.InspectionHoverSpeed;
                    desiredAltitude = missionCommand.InspectionHoverAltitude;
                    holdTime = max(3, missionCommand.InspectionHoverTime);
                    reason = 'Quadcopter mission hover observe';

                case InspectionPhase.AltitudeAdjust
                    mode = BehaviorMode.AltitudeAdjust;
                    waypoint = target.Position;
                    desiredSpeed = min(4, missionCommand.DesiredMissionSpeed);
                    desiredAltitude = missionCommand.InspectionPhaseAltitude;
                    reason = 'Quadcopter mission altitude adjust';

                otherwise
                    mode = BehaviorMode.MoveToPoint;
                    waypoint = missionCommand.CurrentWaypoint;
                    desiredSpeed = missionCommand.DesiredMissionSpeed;
                    desiredAltitude = waypoint(3);
                    reason = 'Quadcopter mission move to point';
            end

            if ~all(isfinite(waypoint))
                waypoint = target.Position;
            end

            desiredHeading = atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1));
            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', holdTime, ...
                'Priority', missionCommand.MissionPriority, ...
                'Reason', ['Mission: ' reason]);
            command.InspectionPhase = phase;
        end

        function command = birdMissionCommand(target, missionCommand, environment)
            phase = missionCommand.BirdPhase;
            holdTime = max(4, min(10, missionCommand.MissionHoldTime));

            switch phase
                case BirdPhase.LowAltitudeHide
                    mode = BehaviorMode.HideLowAltitude;
                    waypoint = target.Position;
                    desiredSpeed = min(10, max(5, missionCommand.DesiredMissionSpeed * 0.65));
                    desiredAltitude = min(10, max(3, missionCommand.CurrentWaypoint(3)));
                    if ~isfinite(desiredAltitude)
                        desiredAltitude = min(10, max(3, target.Position(3)));
                    end
                    reason = 'Bird mission low altitude hide';

                otherwise
                    mode = BehaviorMode.TurnToWaypoint;
                    waypoint = missionCommand.CurrentWaypoint;
                    desiredSpeed = missionCommand.DesiredMissionSpeed;
                    desiredAltitude = waypoint(3);
                    reason = 'Bird mission move to waypoint';
            end

            if ~all(isfinite(waypoint))
                waypoint = target.Position;
            end

            desiredHeading = atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1));
            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', holdTime, ...
                'Priority', missionCommand.MissionPriority, ...
                'Reason', ['Mission: ' reason]);
            command.BirdPhase = phase;
        end

        function command = commandFromMission(target, missionCommand, environment)
            if nargin < 3
                environment = struct();
            end

            if target.Type == TargetType.Ground && ...
                    missionCommand.MissionType == MissionType.FollowRoadRoute && ...
                    isstruct(environment) && isfield(environment, 'Terrain')
                command = BehaviorPlanner.groundMissionCommand(target, missionCommand, environment);
                return;
            end

            if target.Type == TargetType.AirplaneUAV && ...
                    missionCommand.MissionType == MissionType.PatrolRoute
                command = BehaviorPlanner.airplaneMissionCommand(target, missionCommand, environment);
                return;
            end

            if target.Type == TargetType.Quadcopter && ...
                    missionCommand.MissionType == MissionType.InspectArea
                command = BehaviorPlanner.quadcopterMissionCommand(target, missionCommand, environment);
                return;
            end

            if target.Type == TargetType.False && ...
                    missionCommand.MissionType == MissionType.MoveBetweenZones && ...
                    isfield(missionCommand, 'BirdPhase')
                command = BehaviorPlanner.birdMissionCommand(target, missionCommand, environment);
                return;
            end

            waypoint = missionCommand.CurrentWaypoint;
            desiredHeading = atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1));

            command = BehaviorCommand.create( ...
                'BehaviorMode', BehaviorPlanner.missionBehaviorMode(missionCommand.MissionType), ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', missionCommand.DesiredMissionSpeed, ...
                'DesiredAltitude', missionCommand.DesiredMissionAltitude, ...
                'HoldTime', min(8, missionCommand.MissionHoldTime), ...
                'Priority', missionCommand.MissionPriority, ...
                'Reason', ['Mission: ' char(missionCommand.MissionReason)]);
        end

        function mode = missionBehaviorMode(missionType)
            switch missionType
                case MissionType.FollowRoadRoute
                    mode = BehaviorMode.FollowRoad;
                case MissionType.PatrolRoute
                    mode = BehaviorMode.LongCruise;
                case MissionType.InspectArea
                    mode = BehaviorMode.MoveToPoint;
                case MissionType.MoveBetweenZones
                    mode = BehaviorMode.TurnToWaypoint;
                case MissionType.ReturnToArea
                    mode = BehaviorMode.AvoidBoundary;
                case MissionType.LoiterArea
                    mode = BehaviorMode.Loiter;
                otherwise
                    mode = BehaviorMode.Cruise;
            end
        end

        function tf = isMissionAlignedBehavior(behaviorCommand, missionCommand)
            if ~contains(behaviorCommand.Reason, 'Mission:')
                tf = false;
                return;
            end

            if isfield(missionCommand, 'BirdPhase') && isfield(behaviorCommand, 'BirdPhase')
                tf = behaviorCommand.BirdPhase == missionCommand.BirdPhase;
                if missionCommand.BirdPhase == BirdPhase.MoveToWaypoint
                    tf = tf && all(isfinite(behaviorCommand.DesiredPosition)) && ...
                        norm(behaviorCommand.DesiredPosition - missionCommand.CurrentWaypoint) < 1e-3;
                end
                return;
            end

            tf = all(isfinite(behaviorCommand.DesiredPosition)) && ...
                norm(behaviorCommand.DesiredPosition - missionCommand.CurrentWaypoint) < 1e-3;
        end

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
            missionCommand = target.MissionCommand;

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.MissionType == MissionType.MoveBetweenZones && ...
                    missionCommand.Status == MissionStatus.Executing && ...
                    isfield(missionCommand, 'BirdPhase')
                command = BehaviorPlanner.birdMissionCommand(target, missionCommand, environment);
                return;
            end

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
            missionCommand = target.MissionCommand;

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.MissionType == MissionType.FollowRoadRoute && ...
                    missionCommand.Status == MissionStatus.Executing && ...
                    isfield(environment, 'Terrain')
                command = BehaviorPlanner.groundMissionCommand(target, missionCommand, environment);
                return;
            end

            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            context = target.MotionContext;
            if ~isfield(context, 'CruiseSpeed')
                stream = RandStream('mt19937ar', 'Seed', round(target.ID + 17));
                context.CruiseSpeed = 15 + 10 * stream.rand();
                context.ApproachSpeed = 8 + 4 * stream.rand();
                context.TurnSpeed = 5 + 3 * stream.rand();
            end
            if ~isfield(context, 'LaneOffset')
                context.LaneOffset = 2;
            end
            if ~isfield(context, 'GroundPhase')
                context.GroundPhase = 'Drive';
            end

            roadInfo = Environment.findNearestRoad(environment, target.Position);
            roadHeading = roadInfo.Heading;
            segment = environment.RoadNetwork.Segments(roadInfo.SegmentIndex, :);
            endPoint = segment(3:4);
            distRemaining = norm(endPoint - roadInfo.Point(1:2));
            headingError = abs(MotionKinematics.wrapAngle(roadHeading - target.Heading));
            graph = RoadGraph.fromEnvironment(environment);
            distToIntersection = RoadGraph.distanceToNearestIntersection(graph, target.Position);
            cornerAngle = 0;
            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.MissionType == MissionType.FollowRoadRoute
                cornerAngle = BehaviorPlanner.cornerTurnAngle( ...
                    missionCommand.MissionRoute, missionCommand.CurrentWaypointIndex);
            end

            if (strcmp(context.GroundPhase, 'ApproachIntersection') && ...
                    target.Speed > context.ApproachSpeed + 2) || ...
                    (distToIntersection < 80 && distRemaining < 90 && cornerAngle > deg2rad(8))
                mode = BehaviorMode.ApproachIntersection;
                desiredSpeed = context.ApproachSpeed;
                reason = 'Ground vehicle approaching intersection';
                holdTime = 5 + 4 * rand();
            elseif distToIntersection < 25 && (cornerAngle > deg2rad(12) || headingError > deg2rad(12)) && ...
                    target.Speed <= context.ApproachSpeed + 2
                mode = BehaviorMode.TurnAtIntersection;
                desiredSpeed = context.TurnSpeed;
                reason = 'Ground vehicle turning at intersection';
                holdTime = 4 + 3 * rand();
            elseif strcmp(context.GroundPhase, 'Accelerate')
                mode = BehaviorMode.CruiseAfterTurn;
                desiredSpeed = context.CruiseSpeed;
                reason = 'Ground vehicle accelerating after turn';
                holdTime = 6 + 4 * rand();
            else
                mode = BehaviorMode.FollowRoad;
                desiredSpeed = context.CruiseSpeed;
                reason = 'Ground vehicle following road segment';
                holdTime = 8 + 7 * rand();
            end

            lookAhead = min(60, max(15, distRemaining));
            if lookAhead > distRemaining
                lookAhead = max(distRemaining * 0.5, 5);
            end
            waypoint2d = roadInfo.Point(1:2) + lookAhead * [cos(roadHeading), sin(roadHeading)];
            waypoint = RoadGraph.pointToWaypoint(waypoint2d, roadHeading, environment, context.LaneOffset);
            roadSnap = Environment.findNearestRoad(environment, [waypoint(1:2), target.Position(3)]);
            waypoint(1:2) = RoadGraph.applyLaneOffset( ...
                roadSnap.Point(1:2), roadSnap.Heading, context.LaneOffset);
            waypoint(3) = environment.Terrain.Height(waypoint(1), waypoint(2)) + 0.8;
            waypoint(1) = min(max(waypoint(1), environment.XLimits(1)), environment.XLimits(2));
            waypoint(2) = min(max(waypoint(2), environment.YLimits(1)), environment.YLimits(2));

            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', roadHeading, ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', waypoint(3), ...
                'HoldTime', holdTime, ...
                'Priority', 1, ...
                'Reason', reason);
        end

        function command = planAirplane(target, environment)
            profile = TargetProfileRegistry.getProfile(target.Type);
            missionCommand = target.MissionCommand;

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.MissionType == MissionType.PatrolRoute && ...
                    missionCommand.Status == MissionStatus.Executing
                command = BehaviorPlanner.airplaneMissionCommand(target, missionCommand, environment);
                return;
            end

            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);
            desiredSpeed = 14 + 4 * rand();

            roll = rand();
            if roll < 0.70
                mode = BehaviorMode.LongCruise;
                reason = 'Airplane UAV long cruise segment';
                holdTime = 20 + 20 * rand();
            elseif roll < 0.85
                mode = BehaviorMode.WideTurn;
                reason = 'Airplane UAV wide turn';
                holdTime = 18 + 12 * rand();
            elseif roll < 0.95
                mode = BehaviorMode.Patrol;
                reason = 'Airplane UAV patrol segment';
                holdTime = 20 + 15 * rand();
            else
                mode = BehaviorMode.AltitudeCorrection;
                reason = 'Airplane UAV altitude correction';
                holdTime = 25 + 15 * rand();
            end

            segmentLength = 250 + 350 * rand();
            heading = target.Heading + deg2rad(-12 + 24 * rand());
            waypoint = target.Position;
            waypoint(1) = target.Position(1) + segmentLength * cos(heading);
            waypoint(2) = target.Position(2) + segmentLength * sin(heading);

            if mode == BehaviorMode.AltitudeCorrection
                desiredAltitude = target.Position(3) + (-30 + 60 * rand());
            else
                desiredAltitude = target.Position(3);
            end
            desiredAltitude = max(80, min(300, desiredAltitude));
            desiredAltitude = min(max(desiredAltitude, altitudeLimits(1)), altitudeLimits(2));
            waypoint(3) = desiredAltitude;

            command = BehaviorCommand.create( ...
                'BehaviorMode', mode, ...
                'DesiredPosition', waypoint, ...
                'DesiredHeading', atan2(waypoint(2) - target.Position(2), waypoint(1) - target.Position(1)), ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', desiredAltitude, ...
                'HoldTime', holdTime, ...
                'Priority', 1, ...
                'Reason', reason);
        end

        function command = planQuadcopter(target, environment)
            missionCommand = target.MissionCommand;

            if MissionCommand.isActive(missionCommand) && ...
                    missionCommand.MissionType == MissionType.InspectArea && ...
                    missionCommand.Status == MissionStatus.Executing
                command = BehaviorPlanner.quadcopterMissionCommand(target, missionCommand, environment);
                return;
            end

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
                BehaviorMode.ApproachIntersection
                BehaviorMode.TurnAtIntersection
                BehaviorMode.CruiseAfterTurn
                BehaviorMode.Patrol
                BehaviorMode.Cruise
                BehaviorMode.LongCruise
                BehaviorMode.WideTurn
                BehaviorMode.MoveToPoint
                BehaviorMode.Loiter
            ];

            if target.Type == TargetType.Ground && isfield(target.MotionContext, 'SegmentLength')
                if target.MotionContext.DistanceOnRoad >= target.MotionContext.SegmentLength
                    tf = true;
                    return;
                end
            end

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

            if MissionCommand.isActive(target.MissionCommand) && ...
                    target.MissionCommand.MissionType == MissionType.ReturnToArea && ...
                    ~MissionStateMachine.isTerminal(target.MissionCommand)
                return;
            end

            if MissionCommand.isActive(target.MissionCommand) && ...
                    target.MissionCommand.MissionType == MissionType.InspectArea && ...
                    ismember(target.MissionCommand.Status, [MissionStatus.Executing, MissionStatus.Paused])
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

        function turnAngle = cornerTurnAngle(route, waypointIndex)
            turnAngle = 0;
            if isempty(route) || size(route, 1) < 3
                return;
            end

            waypointIndex = max(2, min(waypointIndex, size(route, 1) - 1));
            incoming = route(waypointIndex, 1:2) - route(waypointIndex - 1, 1:2);
            outgoing = route(waypointIndex + 1, 1:2) - route(waypointIndex, 1:2);

            if norm(incoming) < 0.5 || norm(outgoing) < 0.5
                return;
            end

            incomingHeading = atan2(incoming(2), incoming(1));
            outgoingHeading = atan2(outgoing(2), outgoing(1));
            turnAngle = abs(MotionKinematics.wrapAngle(outgoingHeading - incomingHeading));
        end
    end
end
