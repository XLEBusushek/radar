classdef ProbabilityModifiers
    % ProbabilityModifiers  Динамическая корректировка вероятностей перехода.

    methods (Static)
        function probabilities = apply(probabilities, context, behaviorCommand)
            if nargin < 3
                behaviorCommand = BehaviorCommand.empty();
            end

            probabilities = ProbabilityModifiers.applyValidMask(probabilities, context);
            probabilities = ProbabilityModifiers.applyBehaviorCoefficients(probabilities, context);
            probabilities = ProbabilityModifiers.applyBoundaryProximity(probabilities, context);
            probabilities = ProbabilityModifiers.applyAltitudeLimits(probabilities, context);
            probabilities = ProbabilityModifiers.applyRecentTurn(probabilities, context);
            probabilities = ProbabilityModifiers.applyBehaviorCommand(probabilities, context, behaviorCommand);
            probabilities = ProbabilityModifiers.normalize(probabilities, context.ValidMask);
        end

        function probabilities = applyValidMask(probabilities, context)
            probabilities(~context.ValidMask) = 0;
        end

        function probabilities = applyBehaviorCoefficients(probabilities, context)
            coeffs = context.Target.BehaviorCoefficients;
            idx = context.StateIndices;

            turnScale = 1 + 0.8 * coeffs.Maneuverability;
            straightScale = 1 + 0.8 * coeffs.Inertia;
            turnAttenuation = max(0.2, 1 - 0.4 * coeffs.Inertia);
            straightAttenuation = max(0.2, 1 - 0.4 * coeffs.Maneuverability);

            probabilities(idx.TurnLeft) = probabilities(idx.TurnLeft) * turnScale * turnAttenuation;
            probabilities(idx.TurnRight) = probabilities(idx.TurnRight) * turnScale * turnAttenuation;
            probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * straightScale * straightAttenuation;

            probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * (1 + 0.6 * coeffs.Aggressiveness);
            probabilities(idx.SlowDown) = probabilities(idx.SlowDown) * (1 + 0.6 * (1 - coeffs.Aggressiveness));

            alt = context.Target.Position(3);
            preferredAlt = context.Environment.MinAltitude + ...
                coeffs.AltitudePreference * (context.Environment.MaxAltitude - context.Environment.MinAltitude);

            if alt < preferredAlt - 50
                probabilities(idx.Climb) = probabilities(idx.Climb) * (1 + 0.7 * coeffs.AltitudePreference);
            elseif alt > preferredAlt + 50
                probabilities(idx.Descend) = probabilities(idx.Descend) * (1 + 0.7 * coeffs.AltitudePreference);
            end

            uniform = zeros(size(probabilities));
            uniform(context.ValidMask) = 1 / sum(context.ValidMask);
            blendFactor = coeffs.Randomness * 0.75;
            probabilities = (1 - blendFactor) * probabilities + blendFactor * uniform;
        end

        function probabilities = applyBoundaryProximity(probabilities, context)
            position = context.Target.Position;
            environment = context.Environment;
            heading = context.Target.Heading;
            idx = context.StateIndices;

            marginRatio = 0.10;
            thresholdX = marginRatio * diff(environment.XLimits);
            thresholdY = marginRatio * diff(environment.YLimits);

            nearWest = (position(1) - environment.XLimits(1)) < thresholdX;
            nearEast = (environment.XLimits(2) - position(1)) < thresholdX;
            nearSouth = (position(2) - environment.YLimits(1)) < thresholdY;
            nearNorth = (environment.YLimits(2) - position(2)) < thresholdY;

            movingTowardBoundary = ...
                (nearWest && cos(heading) < -0.2) || ...
                (nearEast && cos(heading) > 0.2) || ...
                (nearSouth && sin(heading) < -0.2) || ...
                (nearNorth && sin(heading) > 0.2);

            if movingTowardBoundary || nearWest || nearEast || nearSouth || nearNorth
                probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 0.55;
                probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * 0.70;
                probabilities(idx.TurnLeft) = probabilities(idx.TurnLeft) * 1.45;
                probabilities(idx.TurnRight) = probabilities(idx.TurnRight) * 1.45;
            end
        end

        function probabilities = applyAltitudeLimits(probabilities, context)
            altitude = context.Target.Position(3);
            environment = context.Environment;
            idx = context.StateIndices;
            tolerance = 5;

            if altitude >= environment.MaxAltitude - tolerance
                probabilities(idx.Climb) = 0;
                probabilities(idx.Descend) = probabilities(idx.Descend) * 1.6;
            end

            if altitude <= environment.MinAltitude + tolerance
                probabilities(idx.Descend) = 0;
                probabilities(idx.Climb) = probabilities(idx.Climb) * 1.6;
            end
        end

        function probabilities = applyRecentTurn(probabilities, context)
            currentState = context.Target.CurrentState;
            stateTime = context.Target.StateTime;
            idx = context.StateIndices;

            isTurnState = currentState == TargetBehaviorState.TurnLeft || ...
                currentState == TargetBehaviorState.TurnRight;

            if isTurnState && stateTime <= 4
                probabilities(idx.TurnLeft) = probabilities(idx.TurnLeft) * 0.45;
                probabilities(idx.TurnRight) = probabilities(idx.TurnRight) * 0.45;
                probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 1.35;
            end
        end

        function probabilities = normalize(probabilities, validMask)
            probabilities(~validMask) = 0;
            probabilities = max(probabilities, 0);
            total = sum(probabilities);

            if total > 0
                probabilities = probabilities / total;
                return;
            end

            validIndices = find(validMask);
            probabilities(validIndices) = 1 / numel(validIndices);
        end

        function probabilities = applyBehaviorCommand(probabilities, context, behaviorCommand)
            if ~BehaviorCommand.isActive(behaviorCommand)
                return;
            end

            idx = context.StateIndices;
            target = context.Target;
            environment = context.Environment;

            desiredHeading = behaviorCommand.DesiredHeading;
            if all(isfinite(behaviorCommand.DesiredPosition))
                deltaXY = behaviorCommand.DesiredPosition(1:2) - target.Position(1:2);
                if norm(deltaXY) > 2
                    desiredHeading = atan2(deltaXY(2), deltaXY(1));
                end
            end

            if isfinite(desiredHeading)
                headingError = MotionKinematics.wrapAngle(desiredHeading - target.Heading);
                if abs(headingError) > deg2rad(8)
                    if headingError > 0
                        probabilities(idx.TurnLeft) = probabilities(idx.TurnLeft) * 2.2;
                    else
                        probabilities(idx.TurnRight) = probabilities(idx.TurnRight) * 2.2;
                    end
                    probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 0.7;
                else
                    probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 1.4;
                end
            end

            if isfinite(behaviorCommand.DesiredAltitude)
                altitudeError = behaviorCommand.DesiredAltitude - target.Position(3);
                if altitudeError > 5
                    probabilities(idx.Climb) = probabilities(idx.Climb) * 2.0;
                elseif altitudeError < -5
                    probabilities(idx.Descend) = probabilities(idx.Descend) * 2.0;
                end
            end

            if isfinite(behaviorCommand.DesiredSpeed)
                speedError = behaviorCommand.DesiredSpeed - target.Speed;
                if speedError > 0.5
                    probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * 2.0;
                    probabilities(idx.SlowDown) = probabilities(idx.SlowDown) * 0.6;
                elseif speedError < -0.5
                    probabilities(idx.SlowDown) = probabilities(idx.SlowDown) * 2.0;
                    probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * 0.6;
                end
            end

            if behaviorCommand.BehaviorMode == BehaviorMode.HoverObserve
                probabilities(idx.Hover) = probabilities(idx.Hover) * 2.5;
                probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * 0.5;
                probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 1.2;
            end

            if behaviorCommand.BehaviorMode == BehaviorMode.HideLowAltitude
                probabilities(idx.Hidden) = probabilities(idx.Hidden) * 2.5;
                probabilities(idx.Descend) = probabilities(idx.Descend) * 1.5;
            end

            if behaviorCommand.BehaviorMode == BehaviorMode.AvoidBoundary
                position = target.Position;
                heading = target.Heading;
                marginRatio = 0.10;
                thresholdX = marginRatio * diff(environment.XLimits);
                thresholdY = marginRatio * diff(environment.YLimits);

                nearWest = (position(1) - environment.XLimits(1)) < thresholdX;
                nearEast = (environment.XLimits(2) - position(1)) < thresholdX;
                nearSouth = (position(2) - environment.YLimits(1)) < thresholdY;
                nearNorth = (environment.YLimits(2) - position(2)) < thresholdY;

                movingOutward = ...
                    (nearWest && cos(heading) < -0.1) || ...
                    (nearEast && cos(heading) > 0.1) || ...
                    (nearSouth && sin(heading) < -0.1) || ...
                    (nearNorth && sin(heading) > 0.1);

                if movingOutward
                    probabilities(idx.SpeedUp) = probabilities(idx.SpeedUp) * 0.35;
                    probabilities(idx.FlyStraight) = probabilities(idx.FlyStraight) * 0.45;
                end

                probabilities(idx.TurnLeft) = probabilities(idx.TurnLeft) * 1.8;
                probabilities(idx.TurnRight) = probabilities(idx.TurnRight) * 1.8;
            end
        end
    end
end
