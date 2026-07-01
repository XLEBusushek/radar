classdef TreeZoneRouteBuilder
    % TreeZoneRouteBuilder  Bird routes from Environment.TreeZones.

    methods (Static)
        function [zone, zoneIndex] = selectTreeZone(environment, position, stream)
            zones = environment.TreeZones;
            zoneIndex = 0;
            zone = struct('Center', position(1:2), 'Radius', 0, 'Height', 15);

            if isempty(zones)
                return;
            end

            insideCandidates = [];
            for zoneIdx = 1:numel(zones)
                if TreeZoneRouteBuilder.isInsideZone(position(1:2), zones(zoneIdx))
                    insideCandidates(end + 1) = zoneIdx; %#ok<AGROW>
                end
            end

            if ~isempty(insideCandidates)
                zoneIndex = insideCandidates(stream.randi(numel(insideCandidates)));
            else
                zoneInfo = Environment.findNearestTreeZone(environment, position);
                zoneIndex = zoneInfo.Index;
            end

            if zoneIndex < 1
                return;
            end

            zone = zones(zoneIndex);
        end

        function [route, routeInfo] = buildBirdRoute(environment, stream, startPosition)
            arguments
                environment (1, 1) struct
                stream (1, 1) RandStream
                startPosition (1, 3) double
            end

            zones = environment.TreeZones;
            if isempty(zones)
                route = startPosition;
                routeInfo = TreeZoneRouteBuilder.emptyRouteInfo(route);
                return;
            end

            numPoints = 3 + stream.randi(5);
            [~, zoneIndex] = TreeZoneRouteBuilder.selectTreeZone(environment, startPosition, stream);
            route = zeros(numPoints, 3);
            nearTreeFlags = false(numPoints, 1);

            for pointIdx = 1:numPoints
                if pointIdx == 1
                    zone = zones(zoneIndex);
                    insideZone = stream.rand() < 0.35;
                    route(pointIdx, 1:2) = TreeZoneRouteBuilder.samplePointForZone( ...
                        zone, stream, insideZone);
                    nearTreeFlags(pointIdx) = insideZone || stream.rand() < 0.55;
                else
                    isLongHop = stream.rand() < 0.28;
                    if isLongHop
                        zoneIndex = TreeZoneRouteBuilder.pickDifferentZone(zoneIndex, zones, stream);
                        segmentLength = 80 + 70 * stream.rand();
                    else
                        segmentLength = 20 + 30 * stream.rand();
                    end

                    zone = zones(zoneIndex);
                    insideZone = stream.rand() < 0.30;
                    targetXY = TreeZoneRouteBuilder.samplePointForZone(zone, stream, insideZone);
                    prevXY = route(pointIdx - 1, 1:2);
                    delta = targetXY - prevXY;
                    if norm(delta) < 1
                        heading = 2 * pi * stream.rand();
                    else
                        heading = atan2(delta(2), delta(1));
                    end
                    route(pointIdx, 1:2) = prevXY + segmentLength * [cos(heading), sin(heading)];
                    nearTreeFlags(pointIdx) = insideZone || ...
                        TreeZoneRouteBuilder.isNearTreeZone(route(pointIdx, 1:2), zones);
                end

                route(pointIdx, 3) = TreeZoneRouteBuilder.sampleAltitude( ...
                    route(pointIdx, 1:2), environment, stream, nearTreeFlags(pointIdx));

                if pointIdx < numPoints && stream.rand() < 0.32
                    zoneIndex = TreeZoneRouteBuilder.pickNeighborZone( ...
                        route(pointIdx, 1:2), zoneIndex, zones, stream);
                end
            end

            route = TreeZoneRouteBuilder.clampPointsToEnvironment(route, environment);
            route = TreeZoneRouteBuilder.alignRouteToPosition(route, startPosition);
            segmentLengths = TreeZoneRouteBuilder.allSegmentLengths(route);

            routeInfo = struct( ...
                'WaypointCount', size(route, 1), ...
                'MinSegmentLength', TreeZoneRouteBuilder.minSegmentLength(route), ...
                'MaxSegmentLength', TreeZoneRouteBuilder.maxSegmentLength(route), ...
                'MeanSegmentLength', TreeZoneRouteBuilder.meanSegmentLength(route), ...
                'ShortSegmentCount', sum(segmentLengths >= 19.5 & segmentLengths <= 50.5), ...
                'LongSegmentCount', sum(segmentLengths >= 79.5), ...
                'LowAltitudeWaypointCount', sum(route(:, 3) <= 15.5), ...
                'SegmentLengths', segmentLengths);
        end

        function route = alignRouteToPosition(route, position)
            if isempty(route)
                return;
            end

            distances = vecnorm(route(:, 1:2) - position(1:2), 2, 2);
            [~, startIdx] = min(distances);
            route = [route(startIdx:end, :); route(1:startIdx - 1, :)];
        end

        function lengthValue = minSegmentLength(route)
            lengths = TreeZoneRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = min(lengths);
            end
        end

        function lengthValue = maxSegmentLength(route)
            lengths = TreeZoneRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = max(lengths);
            end
        end

        function lengthValue = meanSegmentLength(route)
            lengths = TreeZoneRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = mean(lengths);
            end
        end

        function lengths = allSegmentLengths(route)
            if size(route, 1) < 2
                lengths = zeros(0, 1);
                return;
            end

            lengths = zeros(size(route, 1) - 1, 1);
            for segmentIdx = 1:numel(lengths)
                lengths(segmentIdx) = norm(route(segmentIdx + 1, 1:2) - route(segmentIdx, 1:2));
            end
        end
    end

    methods (Static, Access = private)
        function routeInfo = emptyRouteInfo(route)
            routeInfo = struct( ...
                'WaypointCount', size(route, 1), ...
                'MinSegmentLength', 0, ...
                'MaxSegmentLength', 0, ...
                'MeanSegmentLength', 0, ...
                'ShortSegmentCount', 0, ...
                'LongSegmentCount', 0, ...
                'LowAltitudeWaypointCount', 0, ...
                'SegmentLengths', zeros(0, 1));
        end

        function altitude = sampleAltitude(point2d, environment, stream, nearTrees)
            zoneInfo = Environment.findNearestTreeZone(environment, [point2d, 0]);

            if nearTrees || (zoneInfo.Index > 0 && zoneInfo.Distance <= zoneInfo.Zone.Radius * 1.15)
                altitude = 0 + 15 * stream.rand();
            else
                preferredAltitude = 20;
                if zoneInfo.Index > 0
                    preferredAltitude = zoneInfo.Zone.Height;
                end
                altitude = preferredAltitude + (-8 + 16 * stream.rand());
            end

            altitude = min(max(altitude, 0), 40);
            altitude = min(max(altitude, environment.MinAltitude + 1), environment.MaxAltitude - 1);
        end

        function point2d = samplePointForZone(zone, stream, insideZone)
            if insideZone
                point2d = TreeZoneRouteBuilder.randomPointInZone(zone, stream);
                return;
            end

            angle = 2 * pi * stream.rand();
            radius = zone.Radius * (0.85 + 0.35 * stream.rand());
            point2d = zone.Center + radius * [cos(angle), sin(angle)];
        end

        function point2d = randomPointInZone(zone, stream)
            angle = 2 * pi * stream.rand();
            radius = zone.Radius * sqrt(stream.rand());
            point2d = zone.Center + radius * [cos(angle), sin(angle)];
        end

        function zoneIndex = pickDifferentZone(currentIndex, zones, stream)
            candidates = setdiff(1:numel(zones), currentIndex);
            if isempty(candidates)
                zoneIndex = currentIndex;
                return;
            end
            zoneIndex = candidates(stream.randi(numel(candidates)));
        end

        function zoneIndex = pickNeighborZone(point2d, currentIndex, zones, stream)
            distances = inf(numel(zones), 1);
            for zoneIdx = 1:numel(zones)
                if zoneIdx == currentIndex
                    continue;
                end
                distances(zoneIdx) = norm(point2d - zones(zoneIdx).Center);
            end

            [sortedDist, sortedIdx] = sort(distances);
            nearestCount = min(3, sum(isfinite(sortedDist)));
            if nearestCount < 1
                zoneIndex = currentIndex;
                return;
            end

            pickIdx = sortedIdx(1:nearestCount);
            pickIdx = pickIdx(stream.randi(numel(pickIdx)));
            zoneIndex = pickIdx;
        end

        function tf = isNearTreeZone(point2d, zones)
            tf = false;
            for zoneIdx = 1:numel(zones)
                zone = zones(zoneIdx);
                if norm(point2d - zone.Center) <= zone.Radius * 1.2
                    tf = true;
                    return;
                end
            end
        end

        function points = clampPointsToEnvironment(points, environment)
            points(:, 1) = min(max(points(:, 1), environment.XLimits(1)), environment.XLimits(2));
            points(:, 2) = min(max(points(:, 2), environment.YLimits(1)), environment.YLimits(2));
        end

        function tf = isInsideZone(point2d, zone)
            tf = norm(point2d - zone.Center) <= zone.Radius;
        end
    end
end
