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
            speed = TargetFactory.randomInRange(profile.SpeedMin, profile.SpeedMax);
            rcs = TargetFactory.randomInRange(profile.RCSMin, profile.RCSMax);
            coefficients = BehaviorCoefficients.createRandom();

            target = RadarTargetModel( ...
                targetType, position, heading, speed, rcs, ...
                'BehaviorCoefficients', coefficients);
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
