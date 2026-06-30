classdef MotionStateExecutor
    % MotionStateExecutor  Преобразование состояния поведения в изменение параметров.

    methods (Static)
        function target = applyState(target, behaviorState, profile, dt)
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

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

                case {TargetBehaviorState.SpeedUp, TargetBehaviorState.SlowDown}
                    % Скорость меняется только через плавную динамику в motion models.

                case TargetBehaviorState.Hover
                    if profile.CanHover
                        hoverSpeedTarget = min(1.0, max(profile.HoverSpeedMin, 0.5));
                        target = MotionKinematics.applySmoothSpeedToTarget( ...
                            target, hoverSpeedTarget, profile, dt);
                        target.Pitch = MotionKinematics.rotateToward(target.Pitch, 0, maxPitchStep);
                    end

                case TargetBehaviorState.Hidden
                    target.IsHidden = true;

                otherwise
                    error('MotionStateExecutor:UnknownState', ...
                        'Unknown behavior state: %s', string(behaviorState));
            end

            target = MotionKinematics.clampSpeed(target, profile, behaviorState);
        end
    end
end
