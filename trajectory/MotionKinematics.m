classdef MotionKinematics
    % MotionKinematics  Общие кинематические операции для моделей движения.

    methods (Static)
        function target = updateVelocity(target)
            target.Velocity = [
                target.Speed * cos(target.Pitch) * cos(target.Heading), ...
                target.Speed * cos(target.Pitch) * sin(target.Heading), ...
                target.Speed * sin(target.Pitch)
            ];
        end

        function newSpeed = applySmoothSpeed(currentSpeed, desiredSpeed, profile, dt)
            speedError = desiredSpeed - currentSpeed;
            if speedError > 0
                maxDelta = profile.MaxAcceleration * dt;
            else
                maxDelta = profile.MaxDeceleration * dt;
            end
            deltaSpeed = max(-maxDelta, min(maxDelta, speedError));
            newSpeed = currentSpeed + deltaSpeed;
        end

        function target = applySmoothSpeedToTarget(target, desiredSpeed, profile, dt)
            target.Speed = MotionKinematics.applySmoothSpeed( ...
                target.Speed, desiredSpeed, profile, dt);
        end

        function target = clampSpeed(target, profile, behaviorState, allowRaiseMin) %#ok<INUSD>
            % clampSpeed  Аварийная защита от выхода за абсолютные пределы скорости.
            maxSpeed = profile.SpeedMax;
            minSpeed = 0;

            if nargin >= 3 && behaviorState == TargetBehaviorState.Hover && profile.CanHover
                maxSpeed = min(maxSpeed, 1.0);
                minSpeed = profile.HoverSpeedMin;
            end

            if target.Speed > maxSpeed
                target.Speed = maxSpeed;
            elseif target.Speed < minSpeed
                target.Speed = minSpeed;
            end
        end

        function value = moveToward(currentValue, targetValue, maxStep)
            delta = targetValue - currentValue;
            delta = max(-maxStep, min(maxStep, delta));
            value = currentValue + delta;
        end

        function target = integratePosition(target, environment, dt)
            newPosition = target.Position + target.Velocity * dt;
            newPosition(1) = MotionKinematics.limitAxis( ...
                newPosition(1), target.Position(1), environment.XLimits);
            newPosition(2) = MotionKinematics.limitAxis( ...
                newPosition(2), target.Position(2), environment.YLimits);
            newPosition(3) = MotionKinematics.limitAxis( ...
                newPosition(3), target.Position(3), environment.ZLimits);
            target.Position = newPosition;
        end

        function target = enforceAltitudeProfile(target, profile, environment)
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);
            altitude = target.Position(3);

            if altitude > altitudeLimits(2)
                target.Position(3) = altitudeLimits(2);
                if target.Pitch > 0
                    target.Pitch = 0;
                end
            elseif altitude < altitudeLimits(1)
                target.Position(3) = altitudeLimits(1);
                if target.Pitch < 0
                    target.Pitch = 0;
                end
            end
        end

        function target = applyBoundarySteering(target, profile, environment, dt)
            position = target.Position;
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            marginRatio = 0.10;

            xSpan = diff(environment.XLimits);
            ySpan = diff(environment.YLimits);
            xMargin = marginRatio * xSpan;
            yMargin = marginRatio * ySpan;

            desiredHeading = target.Heading;
            needsCorrection = false;

            if (position(1) - environment.XLimits(1)) < xMargin
                desiredHeading = 0;
                needsCorrection = true;
            elseif (environment.XLimits(2) - position(1)) < xMargin
                desiredHeading = pi;
                needsCorrection = true;
            end

            if (position(2) - environment.YLimits(1)) < yMargin
                desiredHeading = pi / 2;
                needsCorrection = true;
            elseif (environment.YLimits(2) - position(2)) < yMargin
                desiredHeading = -pi / 2;
                needsCorrection = true;
            end

            if needsCorrection
                target.Heading = MotionKinematics.rotateToward( ...
                    target.Heading, desiredHeading, maxTurnStep);
            end

            target.Heading = MotionKinematics.wrapAngle(target.Heading);
        end

        function target = enforceRateLimits(target, initialHeading, initialSpeed, profile, dt)
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;

            target.Heading = MotionKinematics.rotateToward( ...
                initialHeading, target.Heading, maxTurnStep);

            speedDelta = target.Speed - initialSpeed;
            if speedDelta > 0
                maxSpeedStep = profile.MaxAcceleration * dt;
            else
                maxSpeedStep = profile.MaxDeceleration * dt;
            end
            speedDelta = max(-maxSpeedStep, min(maxSpeedStep, speedDelta));
            target.Speed = initialSpeed + speedDelta;
        end

        function angle = wrapAngle(angle)
            angle = mod(angle + pi, 2 * pi) - pi;
        end

        function angle = rotateToward(currentAngle, targetAngle, maxStep)
            delta = MotionKinematics.wrapAngle(targetAngle - currentAngle);
            delta = max(-maxStep, min(maxStep, delta));
            angle = MotionKinematics.wrapAngle(currentAngle + delta);
        end

        function limits = areaLimits(environment)
            limits = struct( ...
                'XLimits', environment.XLimits, ...
                'YLimits', environment.YLimits, ...
                'ZLimits', environment.ZLimits);
        end
    end

    methods (Static, Access = private)
        function limitedValue = limitAxis(newValue, currentValue, axisLimits)
            if newValue < axisLimits(1)
                limitedValue = axisLimits(1);
            elseif newValue > axisLimits(2)
                limitedValue = axisLimits(2);
            else
                limitedValue = newValue;
            end
        end
    end
end
