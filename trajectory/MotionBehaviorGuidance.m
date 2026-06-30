classdef MotionBehaviorGuidance
    % MotionBehaviorGuidance  Мягкое следование целевым параметрам BehaviorCommand.

    methods (Static)
        function target = apply(target, behaviorCommand, profile, environment, dt)
            if ~BehaviorCommand.isActive(behaviorCommand)
                return;
            end

            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            desiredHeading = MotionBehaviorGuidance.resolveDesiredHeading( ...
                target, behaviorCommand);
            if isfinite(desiredHeading)
                target.Heading = MotionKinematics.rotateToward( ...
                    target.Heading, desiredHeading, maxTurnStep);
            end

            if isfinite(behaviorCommand.DesiredAltitude)
                target = MotionBehaviorGuidance.applyAltitudeGuidance( ...
                    target, behaviorCommand, profile, environment, dt, maxPitchStep);
            end

            if isfinite(behaviorCommand.DesiredSpeed)
                desiredSpeed = MotionBehaviorGuidance.resolveDesiredSpeed( ...
                    target, behaviorCommand, profile, dt);
                target = MotionKinematics.applySmoothSpeedToTarget( ...
                    target, desiredSpeed, profile, dt);
                if isstruct(target.MotionContext) && ...
                        isfield(target.MotionContext, 'DesiredSpeed')
                    target.MotionContext.DesiredSpeed = desiredSpeed;
                end
            end
        end
    end

    methods (Static, Access = private)
        function desiredHeading = resolveDesiredHeading(target, behaviorCommand)
            desiredHeading = behaviorCommand.DesiredHeading;

            if all(isfinite(behaviorCommand.DesiredPosition))
                deltaXY = behaviorCommand.DesiredPosition(1:2) - target.Position(1:2);
                if norm(deltaXY) > 2
                    desiredHeading = atan2(deltaXY(2), deltaXY(1));
                end
            end
        end

        function desiredSpeed = resolveDesiredSpeed(target, behaviorCommand, profile, dt)
            if ~isfield(target.MotionContext, 'SmoothedDesiredSpeed')
                target.MotionContext.SmoothedDesiredSpeed = target.Speed;
            end

            commandSpeed = behaviorCommand.DesiredSpeed;
            responseStep = abs(commandSpeed - target.MotionContext.SmoothedDesiredSpeed) * ...
                dt / max(profile.SpeedResponseTime, dt);
            target.MotionContext.SmoothedDesiredSpeed = MotionKinematics.moveToward( ...
                target.MotionContext.SmoothedDesiredSpeed, commandSpeed, max(responseStep, 0.01));
            desiredSpeed = target.MotionContext.SmoothedDesiredSpeed;
        end

        function target = applyAltitudeGuidance(target, behaviorCommand, profile, environment, dt, maxPitchStep)
            altitudeError = behaviorCommand.DesiredAltitude - target.Position(3);
            tolerance = 2;

            if abs(altitudeError) <= tolerance
                target.Pitch = MotionKinematics.rotateToward(target.Pitch, 0, maxPitchStep);
                return;
            end

            if target.Type == TargetType.Ground
                maxDelta = 3 * dt;
                altitudeStep = sign(altitudeError) * min(abs(altitudeError), maxDelta);
                target.Position(3) = target.Position(3) + altitudeStep;
                target.Pitch = 0;
                return;
            end

            desiredPitch = sign(altitudeError) * min(deg2rad(12), abs(altitudeError) / 40);
            target.Pitch = MotionKinematics.rotateToward( ...
                target.Pitch, desiredPitch, maxPitchStep);
        end
    end
end
