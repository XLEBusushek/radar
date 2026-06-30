classdef BirdMotionModel < MotionModelBase
    % BirdMotionModel  Модель движения для типа False (птица).

    methods (Static)
        function target = update(target, decision, profile, environment, dt)
            headingAtStart = target.Heading;
            speedAtStart = target.Speed;

            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.clampSpeed(target, profile);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
        end
    end
end
