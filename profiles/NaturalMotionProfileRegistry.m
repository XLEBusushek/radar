classdef NaturalMotionProfileRegistry
    % NaturalMotionProfileRegistry  Профили естественных микровозмущений.

    methods (Static)
        function profile = getProfile(targetType)
            if ~TargetType.isValid(targetType)
                error('NaturalMotionProfileRegistry:InvalidType', ...
                    'Unknown target type: %s', targetType);
            end

            profileMap = NaturalMotionProfileRegistry.profileMap();
            profile = profileMap(char(targetType));
        end

        function profile = birdMissionProfile()
            profile = NaturalMotionProfileRegistry.buildBirdMissionProfile();
        end
    end

    methods (Static, Access = private)
        function profileMap = profileMap()
            persistent cachedProfileMap;

            if isempty(cachedProfileMap)
                cachedProfileMap = NaturalMotionProfileRegistry.buildProfileMap();
            end

            profileMap = cachedProfileMap;
        end

        function profileMap = buildProfileMap()
            profileMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

            profileMap(char(TargetType.False)) = NaturalMotionProfileRegistry.baseProfile( ...
                'HeadingSigma', 0.025, 'SpeedSigma', 0.18, 'AltitudeSigma', 0.25, ...
                'PositionSigma', 0.6, 'MaxHeadingNoise', 0.12, 'MaxSpeedNoise', 0.9, ...
                'MaxAltitudeNoise', 2.5, 'MaxPositionNoise', 3.5);
            profileMap(char(TargetType.Ground)) = NaturalMotionProfileRegistry.groundProfile();
            profileMap(char(TargetType.AirplaneUAV)) = NaturalMotionProfileRegistry.airplaneProfile();
            profileMap(char(TargetType.Quadcopter)) = NaturalMotionProfileRegistry.quadcopterProfile();
        end

        function profile = buildBirdMissionProfile()
            profile = NaturalMotionProfileRegistry.baseProfile( ...
                'HeadingSigma', 0.018, 'HeadingTau', 9.0, ...
                'SpeedSigma', 0.08, 'SpeedTau', 7.0, ...
                'AltitudeSigma', 0.10, 'AltitudeTau', 10.0, ...
                'PositionSigma', 0.12, 'PositionTau', 11.0, ...
                'MaxHeadingNoise', deg2rad(8.0), ...
                'MaxSpeedNoise', 0.7, ...
                'MaxAltitudeNoise', 2.0, ...
                'MaxPositionNoise', 0);
            profile.MaxWindDrift = 2.0;
            profile.WindDriftSigma = 0.10;
            profile.WindDriftTau = 13.0;
            profile.SpeedMin = 5.0;
            profile.SpeedMax = 15.0;
            profile.LowHideAltitudeMax = 15.0;
        end

        function profile = quadcopterProfile()
            profile = NaturalMotionProfileRegistry.baseProfile( ...
                'HeadingSigma', 0.012, 'HeadingTau', 10.0, ...
                'SpeedSigma', 0.05, 'SpeedTau', 8.0, ...
                'AltitudeSigma', 0.08, 'AltitudeTau', 12.0, ...
                'PositionSigma', 0.12, 'PositionTau', 14.0, ...
                'MaxHeadingNoise', deg2rad(5.0), ...
                'MaxSpeedNoise', 0.5, ...
                'MaxAltitudeNoise', 1.0, ...
                'MaxPositionNoise', 0);
            profile.MaxHoverDrift = 1.5;
            profile.MaxHoverAltitudeNoise = 1.0;
            profile.MaxHoverSpeedNoise = 0.15;
            profile.MaxMoveSpeedNoise = 0.5;
            profile.HoverDriftSigma = 0.08;
            profile.HoverDriftTau = 16.0;
            profile.MoveSpeedMin = 5.0;
            profile.MoveSpeedMax = 12.0;
            profile.HoverSpeedMax = 1.0;
        end

        function profile = airplaneProfile()
            profile = NaturalMotionProfileRegistry.baseProfile( ...
                'HeadingSigma', 0.004, 'HeadingTau', 22.0, ...
                'SpeedSigma', 0.035, 'SpeedTau', 18.0, ...
                'AltitudeSigma', 0.12, 'AltitudeTau', 28.0, ...
                'PositionSigma', 0, 'PositionTau', 8.0, ...
                'MaxHeadingNoise', deg2rad(3.0), ...
                'MaxSpeedNoise', 0.5, ...
                'MaxAltitudeNoise', 8.0, ...
                'MaxPositionNoise', 0);
            profile.WaypointToleranceMin = 50;
            profile.WaypointToleranceMax = 80;
            profile.MissionAltitudeSpread = 20;
            profile.MissionAltitudeOffsetMax = 40;
        end

        function profile = groundProfile()
            profile = NaturalMotionProfileRegistry.baseProfile( ...
                'HeadingSigma', 0.006, 'SpeedSigma', 0.06, 'AltitudeSigma', 0.02, ...
                'PositionSigma', 0.10, 'MaxHeadingNoise', 0.03, 'MaxSpeedNoise', 0.5, ...
                'MaxAltitudeNoise', 0.2, 'MaxPositionNoise', 0.5);
            profile.MaxLaneOffsetNoise = 0.7;
            profile.MaxRoadHeightNoise = 0.2;
            profile.LaneOffsetSigma = 0.05;
            profile.LaneOffsetTau = 14.0;
            profile.RoadHeightSigma = 0.03;
            profile.RoadHeightTau = 11.0;
            profile.RoadWidth = 6.0;
            profile.GroundRideHeight = 0.8;
            profile.GroundRideHeightTolerance = 0.3;
        end

        function profile = baseProfile(varargin)
            parser = inputParser;
            parser.addParameter('HeadingSigma', 0.02);
            parser.addParameter('HeadingTau', 5.0);
            parser.addParameter('SpeedSigma', 0.15);
            parser.addParameter('SpeedTau', 6.0);
            parser.addParameter('AltitudeSigma', 0.20);
            parser.addParameter('AltitudeTau', 7.0);
            parser.addParameter('PositionSigma', 0.50);
            parser.addParameter('PositionTau', 8.0);
            parser.addParameter('MaxHeadingNoise', 0.10);
            parser.addParameter('MaxSpeedNoise', 0.80);
            parser.addParameter('MaxAltitudeNoise', 2.0);
            parser.addParameter('MaxPositionNoise', 3.0);
            parser.parse(varargin{:});

            profile = struct( ...
                'HeadingSigma', parser.Results.HeadingSigma, ...
                'HeadingTau', parser.Results.HeadingTau, ...
                'SpeedSigma', parser.Results.SpeedSigma, ...
                'SpeedTau', parser.Results.SpeedTau, ...
                'AltitudeSigma', parser.Results.AltitudeSigma, ...
                'AltitudeTau', parser.Results.AltitudeTau, ...
                'PositionSigma', parser.Results.PositionSigma, ...
                'PositionTau', parser.Results.PositionTau, ...
                'MaxHeadingNoise', parser.Results.MaxHeadingNoise, ...
                'MaxSpeedNoise', parser.Results.MaxSpeedNoise, ...
                'MaxAltitudeNoise', parser.Results.MaxAltitudeNoise, ...
                'MaxPositionNoise', parser.Results.MaxPositionNoise);
        end
    end
end
