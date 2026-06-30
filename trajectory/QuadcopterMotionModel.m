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
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);

            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            target.Heading = MotionKinematics.rotateToward( ...
                headingAtStart, target.Heading, maxTurnStep);
            target.Pitch = MotionKinematics.rotateToward( ...
                pitchAtStart, target.Pitch, maxPitchStep);

            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = MotionKinematics.clampSpeed(target, profile, decision.NextState, false);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
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
                if ~isfield(context, 'CommandDesiredSpeed') || ...
                        context.CommandDesiredSpeed ~= behaviorCommand.DesiredSpeed
                    context.CommandDesiredSpeed = behaviorCommand.DesiredSpeed;
                end
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
                        context.DesiredSpeed = context.CommandDesiredSpeed;
                    else
                        cruiseTarget = min(profile.SpeedMax, max(profile.CruiseSpeedMin, target.Speed));
                        context.DesiredSpeed = MotionKinematics.moveToward( ...
                            context.DesiredSpeed, cruiseTarget, maxAccelStep);
                    end
            end

            if BehaviorCommand.isActive(behaviorCommand) && ...
                    behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                context.DesiredSpeed = min(1.0, max(profile.HoverSpeedMin, behaviorCommand.DesiredSpeed));
            end

            target.Speed = MotionKinematics.applySmoothSpeed(target.Speed, context.DesiredSpeed, profile, dt);
            target.MotionContext = context;
        end
    end
end
