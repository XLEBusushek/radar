classdef InspectionRouteBuilder
    % InspectionRouteBuilder  Маршруты инспекции из Environment.InspectionZones.

    methods (Static)
        function [zone, zoneIndex] = selectInspectionZone(environment, position, stream)
            zones = environment.InspectionZones;
            zoneIndex = 0;
            zone = struct('Center', position(1:2), 'Radius', 0, 'Importance', 1);

            if isempty(zones)
                return;
            end

            insideCandidates = [];
            for zoneIdx = 1:numel(zones)
                if InspectionRouteBuilder.isInsideZone(position(1:2), zones(zoneIdx))
                    insideCandidates(end + 1) = zoneIdx; %#ok<AGROW>
                end
            end

            if ~isempty(insideCandidates)
                zoneIndex = insideCandidates(stream.randi(numel(insideCandidates)));
            else
                zoneInfo = Environment.findNearestInspectionZone(environment, position);
                zoneIndex = zoneInfo.Index;
            end

            if zoneIndex < 1
                return;
            end

            zone = zones(zoneIndex);
        end

        function [route, routeInfo] = buildInspectionRoute(zone, environment, stream, startPosition)
            arguments
                zone (1, 1) struct
                environment (1, 1) struct
                stream (1, 1) RandStream
                startPosition (1, 3) double
            end

            numPoints = 3 + stream.randi(3);
            altitudes = InspectionRouteBuilder.sampleAltitudes(stream, environment, numPoints);
            route = zeros(numPoints, 3);
            route(1, 1:2) = InspectionRouteBuilder.randomPointInZone(zone, stream);
            route(1, 3) = altitudes(1);

            for pointIdx = 2:numPoints
                segmentLength = 30 + 120 * stream.rand();
                heading = 2 * pi * stream.rand();
                nextXY = route(pointIdx - 1, 1:2) + segmentLength * [cos(heading), sin(heading)];
                nextXY = InspectionRouteBuilder.clampToZone(nextXY, zone);
                route(pointIdx, 1:2) = nextXY;
                route(pointIdx, 3) = altitudes(pointIdx);
            end

            route = InspectionRouteBuilder.ensureSegmentLengths(route, 30, 150);
            route = InspectionRouteBuilder.clampPointsToEnvironment(route, environment);
            route = InspectionRouteBuilder.alignRouteToPosition(route, startPosition);

            routeInfo = struct( ...
                'WaypointCount', size(route, 1), ...
                'MinSegmentLength', InspectionRouteBuilder.minSegmentLength(route), ...
                'MaxSegmentLength', InspectionRouteBuilder.maxSegmentLength(route), ...
                'MeanSegmentLength', InspectionRouteBuilder.meanSegmentLength(route));
        end

        function route = alignRouteToPosition(route, position)
            if isempty(route)
                return;
            end

            distances = vecnorm(route(:, 1:2) - position(1:2), 2, 2);
            [~, startIdx] = min(distances);
            route = [route(startIdx:end, :); route(1:startIdx - 1, :)];
        end

        function lengthValue = segmentLength(route, segmentIndex)
            startPoint = route(segmentIndex, 1:2);
            endPoint = route(mod(segmentIndex, size(route, 1)) + 1, 1:2);
            lengthValue = norm(endPoint - startPoint);
        end

        function lengthValue = minSegmentLength(route)
            lengths = InspectionRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = min(lengths);
            end
        end

        function lengthValue = maxSegmentLength(route)
            lengths = InspectionRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = max(lengths);
            end
        end

        function lengthValue = meanSegmentLength(route)
            lengths = InspectionRouteBuilder.allSegmentLengths(route);
            if isempty(lengths)
                lengthValue = 0;
            else
                lengthValue = mean(lengths);
            end
        end
    end

    methods (Static, Access = private)
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

        function altitude = sampleAltitude(stream, environment)
            altitude = 40 + 80 * stream.rand();
            altitude = min(max(altitude, 40), 120);
            altitude = min(max(altitude, environment.MinAltitude + 5), environment.MaxAltitude - 5);
        end

        function altitudes = sampleAltitudes(stream, environment, numPoints)
            altitudes = 40 + 80 * stream.rand(numPoints, 1);
            altitudes = min(max(altitudes, 40), 120);
            altitudes = min(max(altitudes, environment.MinAltitude + 5), environment.MaxAltitude - 5);

            if max(altitudes) - min(altitudes) < 15 && numPoints > 1
                altitudes = linspace(50, min(110, environment.MaxAltitude - 5), numPoints)';
            end
        end

        function point2d = randomPointInZone(zone, stream)
            angle = 2 * pi * stream.rand();
            radius = zone.Radius * sqrt(stream.rand());
            point2d = zone.Center + radius * [cos(angle), sin(angle)];
        end

        function point2d = clampToZone(point2d, zone)
            delta = point2d - zone.Center;
            distance = norm(delta);
            if distance > zone.Radius && distance > 0
                point2d = zone.Center + zone.Radius * delta / distance;
            end
        end

        function route = ensureSegmentLengths(route, minLength, maxLength)
            if size(route, 1) < 2
                return;
            end

            adjusted = route(1, :);
            for pointIdx = 2:size(route, 1)
                prevPoint = adjusted(end, 1:2);
                currentPoint = route(pointIdx, 1:2);
                delta = currentPoint - prevPoint;
                distance = norm(delta);
                if distance < minLength
                    if distance < 1
                        heading = 0;
                    else
                        heading = atan2(delta(2), delta(1));
                    end
                    currentPoint = prevPoint + minLength * [cos(heading), sin(heading)];
                elseif distance > maxLength
                    currentPoint = prevPoint + maxLength * delta / distance;
                end
                adjusted(end + 1, :) = [currentPoint, route(pointIdx, 3)]; %#ok<AGROW>
            end

            route = adjusted;
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
