classdef AirplaneMotionModel < MotionModelBase
    % AirplaneMotionModel  Инерционное полетное движение БПЛА.

    methods (Static)
        function target = update(target, decision, behaviorCommand, profile, environment, dt)
            headingAtStart = target.Heading;
            pitchAtStart = target.Pitch;
            speedAtStart = target.Speed;

            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);
            target = AirplaneMotionModel.applyInertialFlight(target, decision.NextState, profile, dt);

            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;
            turnScale = AirplaneMotionModel.resolveTurnScale(decision.NextState, behaviorCommand);

            target.Heading = MotionKinematics.rotateToward( ...
                headingAtStart, target.Heading, maxTurnStep * turnScale);
            target.Pitch = MotionKinematics.rotateToward( ...
                pitchAtStart, target.Pitch, maxPitchStep);

            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);
            target = MotionKinematics.clampSpeed(target, profile, decision.NextState);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
        end
    end

    methods (Static, Access = private)
        function turnScale = resolveTurnScale(behaviorState, behaviorCommand)
            isPatrolMission = BehaviorCommand.isActive(behaviorCommand) && ...
                (strcmp(behaviorCommand.Reason, 'Mission: Airplane mission wide turn') || ...
                strcmp(behaviorCommand.Reason, 'Mission: Airplane mission long cruise'));

            if isPatrolMission
                if behaviorCommand.BehaviorMode == BehaviorMode.WideTurn
                    turnScale = 1.0;
                else
                    turnScale = 0.9;
                end

                if behaviorState == TargetBehaviorState.TurnLeft || ...
                        behaviorState == TargetBehaviorState.TurnRight
                    turnScale = max(turnScale, 0.85);
                end
                return;
            end

            if behaviorState == TargetBehaviorState.TurnLeft || ...
                    behaviorState == TargetBehaviorState.TurnRight
                turnScale = 0.85;
            else
                turnScale = 0.25;
            end
        end

        function target = applyInertialFlight(target, behaviorState, profile, dt)
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            if behaviorState == TargetBehaviorState.FlyStraight
                target.Pitch = MotionKinematics.rotateToward(target.Pitch, 0, maxPitchStep * 0.8);
            elseif behaviorState == TargetBehaviorState.Climb || behaviorState == TargetBehaviorState.Descend
                return;
            else
                target.Pitch = MotionKinematics.rotateToward(target.Pitch, 0, maxPitchStep * 0.35);
            end
        end
    end
end
