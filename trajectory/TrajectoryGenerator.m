classdef TrajectoryGenerator
    % TrajectoryGenerator  Исполнение решений DecisionEngine в физическом движении.

    methods (Static)
        function target = updateMotion(target, decision, environment, dt)
            arguments
                target (1, 1) RadarTargetModel
                decision (1, 1) struct
                environment (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            TrajectoryGenerator.validateDecision(decision);

            target = target.applyDecision(decision);

            if decision.NextState ~= TargetBehaviorState.Hidden
                target.IsHidden = false;
            end

            profile = TargetProfileRegistry.getProfile(target.Type);
            motionUpdate = MotionModelRegistry.getModel(target.Type);
            target = motionUpdate(target, decision, profile, environment, dt);

            target.StateTime = target.StateTime + dt;
            target = target.saveHistory();
        end
    end

    methods (Static, Access = private)
        function validateDecision(decision)
            if ~isfield(decision, 'NextState')
                error('TrajectoryGenerator:InvalidDecision', ...
                    'Decision must contain field NextState.');
            end
        end
    end
end
