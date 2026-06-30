classdef AirplaneMotionModel < MotionModelBase
    % AirplaneMotionModel  Модель движения самолета / БПЛА с инерцией.

    methods (Static)
        function target = update(target, decision, profile, environment, dt)
            headingAtStart = target.Heading;
            pitchAtStart = target.Pitch;
            speedAtStart = target.Speed;

            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);

            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            target.Heading = MotionKinematics.rotateToward( ...
                headingAtStart, target.Heading, maxTurnStep);
            target.Pitch = MotionKinematics.rotateToward( ...
                pitchAtStart, target.Pitch, maxPitchStep);

            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = MotionKinematics.clampSpeed(target, profile);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
        end
    end
end
