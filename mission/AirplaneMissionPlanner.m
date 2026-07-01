classdef AirplaneMissionPlanner
    % AirplaneMissionPlanner  Патрульные миссии самолетных БПЛА по Environment.PatrolZones.

    methods (Static)
        function command = createMission(target, environment)
            stream = RandStream('mt19937ar', 'Seed', round(environment.RandomSeed + target.ID));
            [zone, zoneIndex] = PatrolRouteBuilder.selectPatrolZone(environment, target.Position, stream);

            if zoneIndex < 1 || ~isfield(zone, 'PreferredAltitude')
                command = AirplaneMissionPlanner.fallbackMission(target, environment);
                return;
            end

            altitudeOffset = -20 + 40 * stream.rand();
            missionAltitude = zone.PreferredAltitude + altitudeOffset;
            profile = TargetProfileRegistry.getProfile(target.Type);
            altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);
            missionAltitude = min(max(missionAltitude, altitudeLimits(1)), altitudeLimits(2));

            [route, ~] = PatrolRouteBuilder.buildPatrolRoute(zone, environment, stream, missionAltitude);
            route = PatrolRouteBuilder.alignRouteToPosition(route, target.Position);
            if size(route, 1) < 4
                command = AirplaneMissionPlanner.fallbackMission(target, environment);
                return;
            end

            holdTime = MissionTypeCatalog.minimumHoldTime(target.Type);
            cruiseSpeed = 14 + 4 * stream.rand();
            turnStartDistance = 80 + 70 * stream.rand();

            command = MissionCommand.create( ...
                'MissionType', MissionType.PatrolRoute, ...
                'MissionRoute', route, ...
                'DesiredMissionSpeed', cruiseSpeed, ...
                'DesiredMissionAltitude', missionAltitude, ...
                'MissionHoldTime', holdTime, ...
                'MissionPriority', 1, ...
                'MissionReason', 'Airplane patrol zone route', ...
                'MissionStartTime', target.MissionTime);

            command.PatrolZoneIndex = zoneIndex;
            command.TurnStartDistance = turnStartDistance;
            command.WaypointTolerance = 50 + 30 * stream.rand();
            command.IsCyclic = true;
            command.PatrolZone = zone;
        end
    end

    methods (Static, Access = private)
        function command = fallbackMission(target, environment)
            zoneInfo = Environment.findNearestPatrolZone(environment, target.Position);

            if zoneInfo.Index > 0
                stream = RandStream('mt19937ar', 'Seed', round(environment.RandomSeed + target.ID + 97));
                zone = zoneInfo.Zone;
                missionAltitude = zone.PreferredAltitude;
                [route, ~] = PatrolRouteBuilder.buildPatrolRoute(zone, environment, stream, missionAltitude);
                route = PatrolRouteBuilder.alignRouteToPosition(route, target.Position);
                command = MissionCommand.create( ...
                    'MissionType', MissionType.PatrolRoute, ...
                    'MissionRoute', route, ...
                    'DesiredMissionSpeed', 16, ...
                    'DesiredMissionAltitude', missionAltitude, ...
                    'MissionHoldTime', MissionTypeCatalog.minimumHoldTime(target.Type), ...
                    'MissionPriority', 1, ...
                    'MissionReason', 'Airplane patrol zone route', ...
                    'MissionStartTime', target.MissionTime);
                command.PatrolZoneIndex = zoneInfo.Index;
                command.TurnStartDistance = 120;
                command.WaypointTolerance = 65;
                command.IsCyclic = true;
                return;
            end

            command = MissionCommand.create( ...
                'MissionType', MissionType.PatrolRoute, ...
                'MissionRoute', target.Position, ...
                'DesiredMissionSpeed', 16, ...
                'DesiredMissionAltitude', target.Position(3), ...
                'MissionHoldTime', MissionTypeCatalog.minimumHoldTime(target.Type), ...
                'MissionPriority', 1, ...
                'MissionReason', 'Airplane patrol fallback', ...
                'MissionStartTime', target.MissionTime);
            command.IsCyclic = true;
            command.TurnStartDistance = 120;
            command.WaypointTolerance = 65;
        end
    end
end
