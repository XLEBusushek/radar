classdef (Abstract) MotionModelBase
    % MotionModelBase  Базовый интерфейс модели движения цели.

    methods (Static, Abstract)
        target = update(target, decision, behaviorCommand, profile, environment, dt)
    end

    methods (Static, Access = protected)
        function target = applyCommonMotion(target, decision, behaviorCommand, profile, environment, dt)
            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);
            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.clampSpeed(target, profile);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
        end
    end
end
