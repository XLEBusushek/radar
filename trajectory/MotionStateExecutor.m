classdef MotionStateExecutor
    % MotionStateExecutor  Преобразование состояния поведения в изменение параметров.

    methods (Static)
        function target = applyState(target, behaviorState, profile, dt)
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;
            maxSpeedStep = profile.MaxAcceleration * dt;

            switch behaviorState
                case TargetBehaviorState.FlyStraight
                    % Минимальные плавные изменения — без дополнительных команд.

                case TargetBehaviorState.TurnLeft
                    target.Heading = MotionKinematics.wrapAngle(target.Heading + maxTurnStep);

                case TargetBehaviorState.TurnRight
                    target.Heading = MotionKinematics.wrapAngle(target.Heading - maxTurnStep);

                case TargetBehaviorState.Climb
                    if profile.CanClimb
                        target.Pitch = min(target.Pitch + maxPitchStep, pi / 2);
                    end

                case TargetBehaviorState.Descend
                    if profile.CanDescend
                        target.Pitch = max(target.Pitch - maxPitchStep, -pi / 2);
                    end

                case TargetBehaviorState.SpeedUp
                    target.Speed = target.Speed + maxSpeedStep;

                case TargetBehaviorState.SlowDown
                    target.Speed = target.Speed - maxSpeedStep;

                case TargetBehaviorState.Hover
                    if profile.CanHover
                        hoverSpeed = max(profile.SpeedMin * 0.1, 0.05);
                        if target.Speed > hoverSpeed
                            target.Speed = max(target.Speed - maxSpeedStep, hoverSpeed);
                        end
                        target.Pitch = MotionKinematics.rotateToward(target.Pitch, 0, maxPitchStep);
                    end

                case TargetBehaviorState.Hidden
                    target.IsHidden = true;

                otherwise
                    error('MotionStateExecutor:UnknownState', ...
                        'Unknown behavior state: %s', string(behaviorState));
            end

            target = MotionKinematics.clampSpeed(target, profile);
        end
    end
end
