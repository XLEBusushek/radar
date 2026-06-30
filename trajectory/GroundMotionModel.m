classdef GroundMotionModel < MotionModelBase
    % GroundMotionModel  Движение наземной цели вдоль дорожной сетки.

    methods (Static)
        function target = update(target, decision, behaviorCommand, profile, environment, dt)
            headingAtStart = target.Heading;
            speedAtStart = target.Speed;

            target = GroundMotionModel.ensureContext(target);
            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);
            target = GroundMotionModel.updateRoadHeading(target, decision.NextState, behaviorCommand, environment);
            target = GroundMotionModel.alignToRoad(target, profile, dt);

            if target.Speed < profile.SpeedMin
                target.Speed = profile.SpeedMin;
            end

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

            target.Position(1) = min(max(target.Position(1), environment.XLimits(1)), environment.XLimits(2));
            target.Position(2) = min(max(target.Position(2), environment.YLimits(1)), environment.YLimits(2));
        end
    end

    methods (Static, Access = private)
        function target = ensureContext(target)
            if ~isfield(target.MotionContext, 'BaseAltitude')
                target.MotionContext.BaseAltitude = target.Position(3);
                target.MotionContext.RoadHeading = RoadNetwork.nearestHeading(target.Heading);
                target.MotionContext.DistanceOnRoad = 0;
                target.MotionContext.SegmentLength = 80 + 120 * rand();
            end
        end

        function target = updateRoadHeading(target, behaviorState, behaviorCommand, environment)
            context = target.MotionContext;

            if BehaviorCommand.isActive(behaviorCommand) && all(isfinite(behaviorCommand.DesiredPosition))
                deltaXY = behaviorCommand.DesiredPosition(1:2) - target.Position(1:2);
                if norm(deltaXY) > 5
                    context.RoadHeading = RoadNetwork.nearestHeading(atan2(deltaXY(2), deltaXY(1)));
                end
            end

            shouldTurn = behaviorState == TargetBehaviorState.TurnLeft || ...
                behaviorState == TargetBehaviorState.TurnRight || ...
                context.DistanceOnRoad >= context.SegmentLength || ...
                GroundMotionModel.isNearBoundary(target.Position, environment);

            if shouldTurn
                turnLeft = behaviorState == TargetBehaviorState.TurnLeft;
                if behaviorState ~= TargetBehaviorState.TurnLeft && behaviorState ~= TargetBehaviorState.TurnRight
                    turnLeft = rand() > 0.5;
                end

                context.RoadHeading = RoadNetwork.turnHeading(context.RoadHeading, turnLeft);
                context.DistanceOnRoad = 0;
                context.SegmentLength = 80 + 120 * rand();
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
            baseAltitude = target.MotionContext.BaseAltitude;

            if BehaviorCommand.isActive(behaviorCommand) && isfinite(behaviorCommand.DesiredAltitude)
                desiredAltitude = behaviorCommand.DesiredAltitude;
            else
                desiredAltitude = baseAltitude;
            end

            desiredAltitude = min(max(desiredAltitude, baseAltitude - 3), baseAltitude + 3);
            desiredAltitude = min(max(desiredAltitude, altitudeLimits(1)), altitudeLimits(2));
            altitudeError = desiredAltitude - target.Position(3);
            altitudeStep = sign(altitudeError) * min(abs(altitudeError), 3 * dt);
            target.Position(3) = target.Position(3) + altitudeStep;
            target.Position(3) = min(max(target.Position(3), altitudeLimits(1)), altitudeLimits(2));
        end

        function tf = isNearBoundary(position, environment)
            marginRatio = 0.08;
            xMargin = marginRatio * diff(environment.XLimits);
            yMargin = marginRatio * diff(environment.YLimits);

            tf = (position(1) - environment.XLimits(1)) < xMargin || ...
                (environment.XLimits(2) - position(1)) < xMargin || ...
                (position(2) - environment.YLimits(1)) < yMargin || ...
                (environment.YLimits(2) - position(2)) < yMargin;
        end
    end
end
