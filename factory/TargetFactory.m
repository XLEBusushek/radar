classdef TargetFactory
    % TargetFactory  Создание целей с параметрами из TargetProfileRegistry.

    methods (Static)
        function target = createRandom(targetType, environment)
            arguments
                targetType (1, 1) string
                environment (1, 1) struct
            end

            profile = TargetProfileRegistry.getProfile(targetType);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

            position = [
                TargetFactory.randomInRange(environment.XLimits(1), environment.XLimits(2)), ...
                TargetFactory.randomInRange(environment.YLimits(1), environment.YLimits(2)), ...
                TargetFactory.randomInRange(altitudeLimits(1), altitudeLimits(2))
            ];

            heading = TargetFactory.randomInRange(-pi, pi);
            speedMin = TargetFactory.effectiveSpeedMin(profile);
            speed = TargetFactory.randomInRange(speedMin, profile.SpeedMax);
            rcs = TargetFactory.randomInRange(profile.RCSMin, profile.RCSMax);
            coefficients = BehaviorCoefficients.createRandom();

            target = RadarTargetModel( ...
                targetType, position, heading, speed, rcs, ...
                'BehaviorCoefficients', coefficients);
            target = TargetFactory.initializeMotionContext(target, environment, profile);
        end

        function speedMin = effectiveSpeedMin(profile, behaviorState)
            if nargin < 2
                behaviorState = TargetBehaviorState.FlyStraight;
            end

            if behaviorState == TargetBehaviorState.Hover && profile.CanHover
                speedMin = profile.HoverSpeedMin;
                return;
            end

            if ~isnan(profile.CruiseSpeedMin)
                speedMin = profile.CruiseSpeedMin;
            else
                speedMin = profile.SpeedMin;
            end
        end

        function target = initializeMotionContext(target, environment, profile)
            if nargin < 3
                profile = TargetProfileRegistry.getProfile(target.Type);
            end

            switch char(target.Type)
                case char(TargetType.Ground)
                    target.Heading = RoadNetwork.nearestHeading(target.Heading);
                    target.MotionContext.BaseAltitude = target.Position(3);
                    target.MotionContext.RoadHeading = target.Heading;
                    target.MotionContext.DistanceOnRoad = 0;
                    target.MotionContext.SegmentLength = 80 + 120 * rand();
                case char(TargetType.False)
                    target.MotionContext.StraightDistance = 0;
                    target.MotionContext.NextTurnDistance = 20 + 30 * rand();
                    target.MotionContext.PitchPhase = 2 * pi * rand();
                case char(TargetType.AirplaneUAV)
                    target.MotionContext.LastHeading = target.Heading;
                case char(TargetType.Quadcopter)
                    target.MotionContext.DesiredSpeed = target.Speed;
                    target.MotionContext.HoverAfterWaypoint = false;
            end
        end

        function limits = resolveAltitudeLimits(profile, environment)
            altitudeMin = max(profile.AltitudeMin, environment.MinAltitude);
            altitudeMax = min(profile.AltitudeMax, environment.MaxAltitude);

            if altitudeMax < altitudeMin
                altitudeMax = altitudeMin;
            end

            limits = [altitudeMin, altitudeMax];
        end
    end

    methods (Static, Access = private)
        function value = randomInRange(minValue, maxValue)
            value = minValue + (maxValue - minValue) * rand();
        end
    end
end
