classdef TargetProfileRegistry
    % TargetProfileRegistry  Централизованное хранение профилей типов целей.

    methods (Static)
        function profile = getProfile(targetType)
            if ~TargetType.isValid(targetType)
                error('TargetProfileRegistry:InvalidType', ...
                    'Unknown target type: %s', targetType);
            end

            profileMap = TargetProfileRegistry.profileMap();
            profile = profileMap(char(targetType));
        end
    end

    methods (Static, Access = private)
        function profileMap = profileMap()
            persistent cachedProfileMap;

            if isempty(cachedProfileMap)
                cachedProfileMap = TargetProfileRegistry.buildProfileMap();
            end

            profileMap = cachedProfileMap;
        end

        function profileMap = buildProfileMap()
            profileMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

            profileMap(char(TargetType.False)) = TargetProfileRegistry.falseProfile();
            profileMap(char(TargetType.Ground)) = TargetProfileRegistry.groundProfile();
            profileMap(char(TargetType.AirplaneUAV)) = TargetProfileRegistry.airplaneProfile();
            profileMap(char(TargetType.Quadcopter)) = TargetProfileRegistry.quadcopterProfile();
        end

        function profile = falseProfile()
            profile = TargetProfile();
            profile.SpeedMin = 5;
            profile.SpeedMax = 15;
            profile.RCSMin = 0.001;
            profile.RCSMax = 0.03;
            profile.AltitudeMin = 0;
            profile.AltitudeMax = 40;
            profile.MaxTurnRate = 20;
            profile.MaxPitchRate = 10;
            profile.MaxAcceleration = 2;
            profile.CanHover = false;
            profile.CanClimb = true;
            profile.CanDescend = true;
        end

        function profile = groundProfile()
            profile = TargetProfile();
            profile.SpeedMin = 5;
            profile.SpeedMax = 30;
            profile.RCSMin = 5;
            profile.RCSMax = 30;
            profile.AltitudeMin = 0;
            profile.AltitudeMax = 30;
            profile.MaxTurnRate = 5;
            profile.MaxPitchRate = 3;
            profile.MaxAcceleration = 3;
            profile.CanHover = false;
            profile.CanClimb = false;
            profile.CanDescend = false;
        end

        function profile = airplaneProfile()
            profile = TargetProfile();
            profile.SpeedMin = 10;
            profile.SpeedMax = 20;
            profile.RCSMin = 0.01;
            profile.RCSMax = 0.1;
            profile.AltitudeMin = 0;
            profile.AltitudeMax = 5000;
            profile.MaxTurnRate = 3;
            profile.MaxPitchRate = 2;
            profile.MaxAcceleration = 1;
            profile.CanHover = false;
            profile.CanClimb = true;
            profile.CanDescend = true;
        end

        function profile = quadcopterProfile()
            profile = TargetProfile();
            profile.SpeedMin = 5;
            profile.SpeedMax = 12;
            profile.RCSMin = 0.01;
            profile.RCSMax = 0.1;
            profile.AltitudeMin = 0;
            profile.AltitudeMax = 500;
            profile.MaxTurnRate = 25;
            profile.MaxPitchRate = 15;
            profile.MaxAcceleration = 4;
            profile.HoverSpeedMin = 0;
            profile.CruiseSpeedMin = 5;
            profile.CanHover = true;
            profile.CanClimb = true;
            profile.CanDescend = true;
        end
    end
end
