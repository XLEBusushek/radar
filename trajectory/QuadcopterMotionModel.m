classdef QuadcopterMotionModel < MotionModelBase
    % QuadcopterMotionModel  Маневренное движение квадрокоптера.

    methods (Static)
        function target = update(target, decision, behaviorCommand, profile, environment, dt)
            headingAtStart = target.Heading;
            pitchAtStart = target.Pitch;
            speedAtStart = target.Speed;

            target = QuadcopterMotionModel.ensureContext(target);
            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target.Speed = speedAtStart;
            target = QuadcopterMotionModel.applyDesiredDynamics( ...
                target, decision.NextState, behaviorCommand, profile, dt);
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt, false);

            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            target.Pitch = MotionKinematics.rotateToward( ...
                pitchAtStart, target.Pitch, maxPitchStep);
            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target.Heading = MotionKinematics.rotateToward( ...
                headingAtStart, target.Heading, maxTurnStep);
            target = MotionKinematics.clampSpeed(target, profile, decision.NextState, false);

            if BehaviorCommand.isActive(behaviorCommand) && ...
                    behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                target.Speed = min(target.Speed, max(1.0, behaviorCommand.DesiredSpeed + 0.05));
            end

            target = QuadcopterMotionModel.enforceSpeedStep(target, speedAtStart, profile, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            positionBefore = target.Position;
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            if BehaviorCommand.isActive(behaviorCommand) && ...
                    behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                target.Position(1:2) = positionBefore(1:2);
                target = QuadcopterMotionModel.applyHoverGuidance( ...
                    target, behaviorCommand, dt);
                target = MotionKinematics.updateVelocity(target);
            end
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
        end
    end

    methods (Static, Access = private)
        function target = ensureContext(target)
            if ~isfield(target.MotionContext, 'DesiredSpeed')
                target.MotionContext.DesiredSpeed = target.Speed;
                target.MotionContext.SmoothedDesiredSpeed = target.Speed;
            end
        end

        function target = applyDesiredDynamics(target, behaviorState, behaviorCommand, profile, dt)
            context = target.MotionContext;
            maxAccelStep = profile.MaxAcceleration * dt;
            maxDecelStep = profile.MaxDeceleration * dt;

            if BehaviorCommand.isActive(behaviorCommand) && isfinite(behaviorCommand.DesiredSpeed)
                context.CommandDesiredSpeed = behaviorCommand.DesiredSpeed;
            end

            switch behaviorState
                case TargetBehaviorState.Hover
                    context.DesiredSpeed = min(1.0, max(profile.HoverSpeedMin, 0.2));
                    target.Pitch = MotionKinematics.rotateToward( ...
                        target.Pitch, 0, deg2rad(profile.MaxPitchRate) * dt);
                case TargetBehaviorState.SpeedUp
                    context.DesiredSpeed = min(profile.SpeedMax, context.DesiredSpeed + maxAccelStep);
                case TargetBehaviorState.SlowDown
                    context.DesiredSpeed = max(profile.CruiseSpeedMin, context.DesiredSpeed - maxDecelStep);
                case TargetBehaviorState.Climb
                    context.DesiredSpeed = min(profile.SpeedMax, context.DesiredSpeed + 0.5 * maxAccelStep);
                case TargetBehaviorState.Descend
                    context.DesiredSpeed = max(profile.CruiseSpeedMin, context.DesiredSpeed - 0.5 * maxDecelStep);
                otherwise
                    if isfield(context, 'CommandDesiredSpeed')
                        commandSpeed = context.CommandDesiredSpeed;
                        if commandSpeed >= target.Speed
                            context.DesiredSpeed = MotionKinematics.moveToward( ...
                                target.Speed, commandSpeed, maxAccelStep);
                        else
                            context.DesiredSpeed = MotionKinematics.moveToward( ...
                                target.Speed, commandSpeed, maxDecelStep);
                        end
                    else
                        cruiseTarget = min(profile.SpeedMax, max(profile.CruiseSpeedMin, target.Speed));
                        context.DesiredSpeed = MotionKinematics.moveToward( ...
                            context.DesiredSpeed, cruiseTarget, maxAccelStep);
                    end
            end

            if BehaviorCommand.isActive(behaviorCommand) && ...
                    behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                hoverTarget = min(1.0, max(profile.HoverSpeedMin, behaviorCommand.DesiredSpeed));
                context.DesiredSpeed = MotionKinematics.moveToward( ...
                    target.Speed, hoverTarget, maxDecelStep);
            end

            target.Speed = MotionKinematics.applySmoothSpeed(target.Speed, context.DesiredSpeed, profile, dt);
            target.MotionContext = context;
        end

        function target = enforceSpeedStep(target, speedAtStart, profile, dt)
            maxUp = profile.MaxAcceleration * dt;
            maxDown = profile.MaxDeceleration * dt;
            speedDelta = target.Speed - speedAtStart;
            speedDelta = max(-maxDown, min(maxUp, speedDelta));
            target.Speed = speedAtStart + speedDelta;
        end

        function target = applyHoverGuidance(target, behaviorCommand, dt)
            if all(isfinite(behaviorCommand.DesiredPosition))
                deltaXY = behaviorCommand.DesiredPosition(1:2) - target.Position(1:2);
                maxStep = 0.6 * dt;
                stepNorm = norm(deltaXY);
                if stepNorm > 1e-6
                    step = min(maxStep, stepNorm);
                    target.Position(1:2) = target.Position(1:2) + deltaXY / stepNorm * step;
                end
            end

            if isfinite(behaviorCommand.DesiredAltitude)
                altitudeError = behaviorCommand.DesiredAltitude - target.Position(3);
                altitudeStep = sign(altitudeError) * min(abs(altitudeError), 0.35 * dt);
                target.Position(3) = target.Position(3) + altitudeStep;
            end
        end
    end
end
