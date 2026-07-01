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
            target.MotionContext.SmoothedDesiredSpeed = target.Speed;
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
                    stream = RandStream('mt19937ar', 'Seed', round(target.ID + 17));
                    target.MotionContext.LaneOffset = RoadGraph.pickLaneOffset(stream);
                    target.MotionContext.DistanceOnRoad = 0;
                    target.MotionContext.CruiseSpeed = 15 + 10 * stream.rand();
                    target.MotionContext.ApproachSpeed = 8 + 4 * stream.rand();
                    target.MotionContext.TurnSpeed = 5 + 3 * stream.rand();
                    target.MotionContext.GroundPhase = 'Drive';
                    target = TargetFactory.snapGroundToRoad(target, environment);
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

        function target = snapGroundToRoad(target, environment)
            if ~isfield(environment, 'RoadNetwork') || isempty(environment.RoadNetwork.Segments)
                target.Heading = RoadNetwork.nearestHeading(target.Heading);
                target.MotionContext.RoadHeading = target.Heading;
                target.MotionContext.BaseAltitude = target.Position(3);
                return;
            end

            roadInfo = Environment.findNearestRoad(environment, target.Position);
            laneOffset = target.MotionContext.LaneOffset;
            snappedXY = RoadGraph.applyLaneOffset(roadInfo.Point(1:2), roadInfo.Heading, laneOffset);
            terrainHeight = environment.Terrain.Height(snappedXY(1), snappedXY(2));

            target.Position = [snappedXY, terrainHeight + 0.8];
            target.Heading = roadInfo.Heading;
            target.MotionContext.RoadHeading = roadInfo.Heading;
            target.MotionContext.BaseAltitude = terrainHeight + 0.8;
            target.MotionContext.CurrentRoadSegmentIndex = roadInfo.SegmentIndex;
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
