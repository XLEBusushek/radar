classdef GroundMotionModel < MotionModelBase
    % GroundMotionModel  Движение наземной цели вдоль RoadNetwork Environment.

    methods (Static)
        function target = update(target, decision, behaviorCommand, profile, environment, dt)
            headingAtStart = target.Heading;
            speedAtStart = target.Speed;

            target = GroundMotionModel.ensureContext(target, environment);
            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = GroundMotionModel.updateGroundPhase(target, behaviorCommand, environment);
            target = GroundMotionModel.updateRoadHeading(target, behaviorCommand, environment);
            target = GroundMotionModel.alignToRoad(target, profile, dt);
            target = GroundMotionModel.applyRoadSpeed(target, behaviorCommand, profile, dt);

            target.Pitch = 0;
            target = MotionKinematics.clampSpeed(target, profile, decision.NextState);
            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = GroundMotionModel.applyGroundAltitude(target, behaviorCommand, profile, environment, dt);

            target = MotionKinematics.updateVelocity(target);

            displacement = target.Speed * dt;
            target.Position(1) = target.Position(1) + displacement * cos(target.Heading);
            target.Position(2) = target.Position(2) + displacement * sin(target.Heading);
            target.MotionContext.DistanceOnRoad = target.MotionContext.DistanceOnRoad + displacement;

            target = GroundMotionModel.snapToRoad(target, environment);
            target.Position(1) = min(max(target.Position(1), environment.XLimits(1)), environment.XLimits(2));
            target.Position(2) = min(max(target.Position(2), environment.YLimits(1)), environment.YLimits(2));
        end
    end

    methods (Static, Access = private)
        function target = ensureContext(target, environment)
            if ~isfield(target.MotionContext, 'LaneOffset')
                stream = RandStream('mt19937ar', 'Seed', round(target.ID));
                target.MotionContext.LaneOffset = RoadGraph.pickLaneOffset(stream);
                target.MotionContext.RoadHeading = target.Heading;
                target.MotionContext.DistanceOnRoad = 0;
                target.MotionContext.GroundPhase = 'Drive';
                target = GroundMotionModel.initSegmentSpeeds(target);
            end

            if ~isfield(target.MotionContext, 'GroundPhase')
                target.MotionContext.GroundPhase = 'Drive';
            end

            if isfield(environment, 'Terrain') && ~isempty(environment.Terrain)
                terrainHeight = environment.Terrain.Height(target.Position(1), target.Position(2));
                target.MotionContext.BaseAltitude = terrainHeight + 0.8;
            end
        end

        function target = initSegmentSpeeds(target)
            stream = RandStream('mt19937ar', 'Seed', round(target.ID + 17));
            context = target.MotionContext;
            context.CruiseSpeed = 15 + 10 * stream.rand();
            context.ApproachSpeed = 8 + 4 * stream.rand();
            context.TurnSpeed = 5 + 3 * stream.rand();
            target.MotionContext = context;
        end

        function target = updateGroundPhase(target, behaviorCommand, environment)
            context = target.MotionContext;

            if BehaviorCommand.isActive(behaviorCommand)
                switch behaviorCommand.BehaviorMode
                    case BehaviorMode.ApproachIntersection
                        context.GroundPhase = 'ApproachIntersection';
                    case BehaviorMode.TurnAtIntersection
                        context.GroundPhase = 'Turn';
                    case BehaviorMode.CruiseAfterTurn
                        context.GroundPhase = 'Accelerate';
                    case BehaviorMode.FollowRoad
                        if strcmp(context.GroundPhase, 'Accelerate') && ...
                                target.Speed >= context.CruiseSpeed - 1.0
                            context.GroundPhase = 'Drive';
                        elseif strcmp(context.GroundPhase, 'Turn')
                            headingError = abs(MotionKinematics.wrapAngle( ...
                                behaviorCommand.DesiredHeading - target.Heading));
                            if headingError < deg2rad(5)
                                context.GroundPhase = 'Accelerate';
                            end
                        elseif ~ismember(context.GroundPhase, ...
                                {'ApproachIntersection', 'Turn', 'Accelerate'})
                            context.GroundPhase = 'Drive';
                        end
                end
            end

            target.MotionContext = context;
        end

        function target = applyRoadSpeed(target, behaviorCommand, profile, dt)
            context = target.MotionContext;

            switch context.GroundPhase
                case 'Drive'
                    desiredSpeed = context.CruiseSpeed;
                case 'ApproachIntersection'
                    desiredSpeed = context.ApproachSpeed;
                    decelStep = profile.MaxDeceleration * 1.8 * dt;
                    if target.Speed > desiredSpeed
                        target.Speed = max(desiredSpeed, target.Speed - decelStep);
                    end
                case 'Turn'
                    desiredSpeed = context.TurnSpeed;
                case 'Accelerate'
                    desiredSpeed = context.CruiseSpeed;
                otherwise
                    desiredSpeed = context.CruiseSpeed;
            end

            if BehaviorCommand.isActive(behaviorCommand) && isfinite(behaviorCommand.DesiredSpeed)
                desiredSpeed = behaviorCommand.DesiredSpeed;
            end

            target.Speed = MotionKinematics.applySmoothSpeed(target.Speed, desiredSpeed, profile, dt);
            target.MotionContext = context;
        end

        function target = updateRoadHeading(target, behaviorCommand, ~)
            context = target.MotionContext;

            if BehaviorCommand.isActive(behaviorCommand) && all(isfinite(behaviorCommand.DesiredPosition))
                deltaXY = behaviorCommand.DesiredPosition(1:2) - target.Position(1:2);
                if norm(deltaXY) > 3
                    context.RoadHeading = atan2(deltaXY(2), deltaXY(1));
                end
            end

            target.MotionContext = context;
        end

        function target = alignToRoad(target, profile, dt)
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            target.Heading = MotionKinematics.rotateToward( ...
                target.Heading, target.MotionContext.RoadHeading, maxTurnStep);
        end

        function target = applyGroundAltitude(target, behaviorCommand, profile, environment, dt)
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);
            terrainHeight = environment.Terrain.Height(target.Position(1), target.Position(2));
            desiredAltitude = terrainHeight + 0.8;

            if BehaviorCommand.isActive(behaviorCommand) && isfinite(behaviorCommand.DesiredAltitude)
                desiredAltitude = behaviorCommand.DesiredAltitude;
            end

            desiredAltitude = min(max(desiredAltitude, terrainHeight + 0.5), terrainHeight + 1.1);
            desiredAltitude = min(max(desiredAltitude, altitudeLimits(1)), altitudeLimits(2));
            altitudeError = desiredAltitude - target.Position(3);
            altitudeStep = sign(altitudeError) * min(abs(altitudeError), 4 * dt);
            target.Position(3) = target.Position(3) + altitudeStep;
            heightNoise = 0;
            if isfield(target.NaturalMotionState, 'RoadHeightNoise')
                heightNoise = target.NaturalMotionState.RoadHeightNoise;
            end
            target.Position(3) = min(max(terrainHeight + 0.8 + heightNoise, terrainHeight + 0.5), ...
                terrainHeight + 1.1);
            target.MotionContext.BaseAltitude = terrainHeight + 0.8;
        end

        function target = snapToRoad(target, environment)
            if ~isfield(environment, 'RoadNetwork')
                return;
            end

            roadInfo = Environment.findNearestRoad(environment, target.Position);
            baseLaneOffset = target.MotionContext.LaneOffset;
            laneOffsetNoise = 0;
            if isfield(target.NaturalMotionState, 'LaneOffsetNoise')
                laneOffsetNoise = target.NaturalMotionState.LaneOffsetNoise;
            end
            laneOffset = RoadGraph.clampFinalLaneOffset(baseLaneOffset, laneOffsetNoise);
            heading = roadInfo.Heading;
            snappedXY = RoadGraph.applyLaneOffset(roadInfo.Point(1:2), heading, laneOffset);
            blendRatio = 0.92;
            if roadInfo.Distance > 4
                blendRatio = 0.96;
            end
            target.Position(1:2) = (1 - blendRatio) * target.Position(1:2) + blendRatio * snappedXY;
            target.MotionContext.CurrentRoadSegmentIndex = roadInfo.SegmentIndex;
        end

        function tf = isNearIntersection(target, environment)
            graph = RoadGraph.fromEnvironment(environment);
            dist = RoadGraph.distanceToNearestIntersection(graph, target.Position);
            tf = dist < 40;
        end
    end
end
