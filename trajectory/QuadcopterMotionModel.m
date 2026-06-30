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
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);
            target = QuadcopterMotionModel.applyDesiredDynamics( ...
                target, decision.NextState, behaviorCommand, profile, dt);

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
            end
        end

        function target = applyDesiredDynamics(target, behaviorState, behaviorCommand, profile, dt)
            maxAccel = profile.MaxAcceleration * dt;
            context = target.MotionContext;

            if BehaviorCommand.isActive(behaviorCommand) && isfinite(behaviorCommand.DesiredSpeed)
                context.DesiredSpeed = behaviorCommand.DesiredSpeed;
            end

            switch behaviorState
                case TargetBehaviorState.Hover
                    context.DesiredSpeed = min(1.0, max(profile.HoverSpeedMin, 0.2));
                    target.Pitch = MotionKinematics.rotateToward( ...
                        target.Pitch, 0, deg2rad(profile.MaxPitchRate) * dt);
                case TargetBehaviorState.SpeedUp
                    context.DesiredSpeed = min(profile.SpeedMax, context.DesiredSpeed + maxAccel);
                case TargetBehaviorState.SlowDown
                    context.DesiredSpeed = max(profile.CruiseSpeedMin, context.DesiredSpeed - maxAccel);
                case TargetBehaviorState.Climb
                    context.DesiredSpeed = min(profile.SpeedMax, context.DesiredSpeed + 0.5 * maxAccel);
                case TargetBehaviorState.Descend
                    context.DesiredSpeed = max(profile.CruiseSpeedMin, context.DesiredSpeed - 0.5 * maxAccel);
                otherwise
                    cruiseTarget = min(profile.SpeedMax, max(profile.CruiseSpeedMin, target.Speed));
                    context.DesiredSpeed = MotionKinematics.moveToward( ...
                        context.DesiredSpeed, cruiseTarget, maxAccel);
            end

            if BehaviorCommand.isActive(behaviorCommand) && ...
                    behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                context.DesiredSpeed = min(1.0, max(profile.HoverSpeedMin, behaviorCommand.DesiredSpeed));
            end

            target.Speed = MotionKinematics.moveToward(target.Speed, context.DesiredSpeed, maxAccel);
            target.MotionContext = context;
        end
    end
end
