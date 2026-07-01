classdef BirdMissionPlanner
    % BirdMissionPlanner  Missions for birds using Environment.TreeZones.

    methods (Static)
        function command = createMission(target, environment)
            stream = RandStream('mt19937ar', 'Seed', round(environment.RandomSeed + target.ID));
            [~, zoneIndex] = TreeZoneRouteBuilder.selectTreeZone( ...
                environment, target.Position, stream);

            if zoneIndex < 1
                command = BirdMissionPlanner.fallbackMission(target, environment);
                return;
            end

            [route, ~] = TreeZoneRouteBuilder.buildBirdRoute( ...
                environment, stream, target.Position);
            if size(route, 1) < 3
                command = BirdMissionPlanner.fallbackMission(target, environment);
                return;
            end

            holdTime = MissionTypeCatalog.minimumHoldTime(target.Type);
            legCount = max(size(route, 1) - 1, 1);
            segmentSpeeds = 5 + 10 * stream.rand(legCount, 1);

            command = MissionCommand.create( ...
                'MissionType', MissionType.MoveBetweenZones, ...
                'MissionRoute', route, ...
                'DesiredMissionSpeed', segmentSpeeds(1), ...
                'DesiredMissionAltitude', route(1, 3), ...
                'MissionHoldTime', holdTime, ...
                'MissionPriority', 1, ...
                'MissionReason', 'Bird move between tree zones', ...
                'MissionStartTime', target.MissionTime);

            command = BirdMissionPlanner.attachBirdFields(command, zoneIndex, stream, route, segmentSpeeds);
        end
    end

    methods (Static, Access = private)
        function command = attachBirdFields(command, zoneIndex, stream, route, segmentSpeeds)
            command.BirdTreeZoneIndex = zoneIndex;
            command.BirdPhase = BirdPhase.MoveToWaypoint;
            command.BirdSegmentSpeeds = segmentSpeeds;
            command.BirdLowAltitudeHideFlags = false(size(route, 1), 1);

            for pointIdx = 1:size(route, 1)
                if route(pointIdx, 3) <= 15.5
                    command.BirdLowAltitudeHideFlags(pointIdx) = stream.rand() < 0.80;
                end
            end

            if ~any(command.BirdLowAltitudeHideFlags)
                [~, lowIdx] = min(route(:, 3));
                command.BirdLowAltitudeHideFlags(lowIdx) = true;
            end

            command.BirdHideDuration = 0;
            command.BirdHideElapsed = 0;
        end

        function command = fallbackMission(target, environment)
            zoneInfo = Environment.findNearestTreeZone(environment, target.Position);

            if zoneInfo.Index > 0
                stream = RandStream('mt19937ar', 'Seed', round(environment.RandomSeed + target.ID + 29));
                [route, ~] = TreeZoneRouteBuilder.buildBirdRoute( ...
                    environment, stream, target.Position);
                legCount = max(size(route, 1) - 1, 1);
                segmentSpeeds = 5 + 10 * stream.rand(legCount, 1);
                command = MissionCommand.create( ...
                    'MissionType', MissionType.MoveBetweenZones, ...
                    'MissionRoute', route, ...
                    'DesiredMissionSpeed', segmentSpeeds(1), ...
                    'DesiredMissionAltitude', route(1, 3), ...
                    'MissionHoldTime', MissionTypeCatalog.minimumHoldTime(target.Type), ...
                    'MissionPriority', 1, ...
                    'MissionReason', 'Bird move between tree zones', ...
                    'MissionStartTime', target.MissionTime);
                command = BirdMissionPlanner.attachBirdFields( ...
                    command, zoneInfo.Index, stream, route, segmentSpeeds);
                return;
            end

            route = target.Position;
            stream = RandStream('mt19937ar', 'Seed', 1);
            command = MissionCommand.create( ...
                'MissionType', MissionType.MoveBetweenZones, ...
                'MissionRoute', route, ...
                'DesiredMissionSpeed', 8, ...
                'DesiredMissionAltitude', target.Position(3), ...
                'MissionHoldTime', MissionTypeCatalog.minimumHoldTime(target.Type), ...
                'MissionPriority', 1, ...
                'MissionReason', 'Bird tree zone fallback', ...
                'MissionStartTime', target.MissionTime);
            command = BirdMissionPlanner.attachBirdFields( ...
                command, 0, stream, route, 8);
        end
    end
end
