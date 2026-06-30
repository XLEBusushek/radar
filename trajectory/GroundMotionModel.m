classdef GroundMotionModel < MotionModelBase
    % GroundMotionModel  Упрощенная кинематическая Bicycle Model для наземной цели.

    methods (Static)
        function target = update(target, decision, profile, environment, dt)
            headingAtStart = target.Heading;
            speedAtStart = target.Speed;

            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);

            if target.Speed < profile.SpeedMin
                target.Speed = profile.SpeedMin;
            end

            target.Pitch = 0;
            target = MotionKinematics.clampSpeed(target, profile);
            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);

            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;
            desiredAltitude = min(max(target.Position(3), altitudeLimits(1)), altitudeLimits(2));
            altitudeError = desiredAltitude - target.Position(3);

            if abs(altitudeError) > 0.01
                climbRate = speedAtStart * sin(maxPitchStep);
                target.Position(3) = target.Position(3) + sign(altitudeError) * min(abs(altitudeError), climbRate * dt);
            end

            target.Position(3) = min(max(target.Position(3), altitudeLimits(1)), altitudeLimits(2));
            target = MotionKinematics.updateVelocity(target);

            displacement = target.Speed * dt;
            target.Position(1) = target.Position(1) + displacement * cos(target.Heading);
            target.Position(2) = target.Position(2) + displacement * sin(target.Heading);

            target.Position(1) = min(max(target.Position(1), environment.XLimits(1)), environment.XLimits(2));
            target.Position(2) = min(max(target.Position(2), environment.YLimits(1)), environment.YLimits(2));
        end
    end
end
