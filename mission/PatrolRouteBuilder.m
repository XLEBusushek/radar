classdef PatrolRouteBuilder
    % PatrolRouteBuilder  Построение патрульных маршрутов из Environment.PatrolZones.

    methods (Static)
        function [zone, zoneIndex] = selectPatrolZone(environment, position, stream)
            zones = environment.PatrolZones;
            zoneIndex = 0;
            zone = struct('Polygon', zeros(0, 2), 'PreferredAltitude', position(3), 'Priority', 1);

            if isempty(zones)
                return;
            end

            insideCandidates = [];
            for zoneIdx = 1:numel(zones)
                if PatrolRouteBuilder.isInsidePolygon(position(1:2), zones(zoneIdx).Polygon)
                    insideCandidates(end + 1) = zoneIdx; %#ok<AGROW>
                end
            end

            if ~isempty(insideCandidates)
                zoneIndex = insideCandidates(stream.randi(numel(insideCandidates)));
            else
                zoneInfo = Environment.findNearestPatrolZone(environment, position);
                zoneIndex = zoneInfo.Index;
            end

            if zoneIndex < 1
                return;
            end

            zone = zones(zoneIndex);
        end

        function [route, routeInfo] = buildPatrolRoute(zone, environment, stream, preferredAltitude)
            arguments
                zone (1, 1) struct
                environment (1, 1) struct
                stream (1, 1) RandStream
                preferredAltitude (1, 1) double
            end

            numPoints = 4 + stream.randi(3);
            shapeRoll = stream.rand();
            polygon = PatrolRouteBuilder.scalePolygonToMinEdge(zone.Polygon, 250);

            if shapeRoll < 0.45
                waypoints2d = PatrolRouteBuilder.rectangleRoute(polygon, numPoints);
            elseif shapeRoll < 0.8
                waypoints2d = PatrolRouteBuilder.racetrackRoute(polygon, numPoints);
            else
                waypoints2d = PatrolRouteBuilder.polygonRoute(polygon, numPoints);
            end

            waypoints2d = PatrolRouteBuilder.clampPointsToEnvironment(waypoints2d, environment);
            route = [waypoints2d, repmat(preferredAltitude, size(waypoints2d, 1), 1)];
            route = PatrolRouteBuilder.ensureSegmentLengths(route, 250, 700);

            routeInfo = struct( ...
                'WaypointCount', size(route, 1), ...
                'MinSegmentLength', PatrolRouteBuilder.minSegmentLength(route), ...
                'MeanSegmentLength', PatrolRouteBuilder.meanSegmentLength(route));
        end

        function lengthValue = segmentLength(route, segmentIndex)
            startPoint = route(segmentIndex, 1:2);
            endPoint = route(mod(segmentIndex, size(route, 1)) + 1, 1:2);
            lengthValue = norm(endPoint - startPoint);
        end

        function lengthValue = minSegmentLength(route)
            if size(route, 1) < 2
                lengthValue = 0;
                return;
            end

            lengths = zeros(size(route, 1), 1);
            for segmentIdx = 1:size(route, 1)
                lengths(segmentIdx) = PatrolRouteBuilder.segmentLength(route, segmentIdx);
            end
            lengthValue = min(lengths);
        end

        function lengthValue = meanSegmentLength(route)
            if size(route, 1) < 2
                lengthValue = 0;
                return;
            end

            lengths = zeros(size(route, 1), 1);
            for segmentIdx = 1:size(route, 1)
                lengths(segmentIdx) = PatrolRouteBuilder.segmentLength(route, segmentIdx);
            end
            lengthValue = mean(lengths);
        end

        function route = alignRouteToPosition(route, position)
            if isempty(route)
                return;
            end

            distances = vecnorm(route(:, 1:2) - position(1:2), 2, 2);
            [~, startIdx] = min(distances);
            route = [route(startIdx:end, :); route(1:startIdx-1, :)];
        end
    end

    methods (Static, Access = private)
        function polygon = scalePolygonToMinEdge(polygon, minEdge)
            for iterIdx = 1:25
                edgeLengths = PatrolRouteBuilder.polygonEdgeLengths(polygon);
                if min(edgeLengths) >= minEdge
                    return;
                end

                centroid = mean(polygon, 1);
                scaleFactor = minEdge / max(min(edgeLengths), 1) * 1.05;
                for vertexIdx = 1:size(polygon, 1)
                    polygon(vertexIdx, :) = centroid + scaleFactor * (polygon(vertexIdx, :) - centroid);
                end
            end
        end

        function edgeLengths = polygonEdgeLengths(polygon)
            vertexCount = size(polygon, 1);
            edgeLengths = zeros(vertexCount, 1);
            for vertexIdx = 1:vertexCount
                nextIdx = mod(vertexIdx, vertexCount) + 1;
                edgeLengths(vertexIdx) = norm(polygon(nextIdx, :) - polygon(vertexIdx, :));
            end
        end

        function waypoints = rectangleRoute(polygon, numPoints)
            corners = PatrolRouteBuilder.orderPolygonVertices(polygon);
            waypoints = PatrolRouteBuilder.samplePolygonWaypoints(corners, numPoints);
        end

        function waypoints = racetrackRoute(polygon, numPoints)
            corners = PatrolRouteBuilder.orderPolygonVertices(polygon);
            centroid = mean(corners, 1);
            [~, longAxisIdx] = max(vecnorm(corners - centroid, 2, 2));
            axisVector = corners(longAxisIdx, :) - centroid;
            axisHeading = atan2(axisVector(2), axisVector(1));
            rotation = [cos(-axisHeading), -sin(-axisHeading); sin(-axisHeading), cos(-axisHeading)];
            rotated = (corners - centroid) * rotation';

            minXY = min(rotated(:, 1:2), [], 1);
            maxXY = max(rotated(:, 1:2), [], 1);
            spanX = maxXY(1) - minXY(1);
            spanY = maxXY(2) - minXY(2);
            insetY = max(spanY * 0.2, 80);

            trackPoints = [
                minXY(1) + 0.1 * spanX, minXY(2) + insetY
                maxXY(1) - 0.1 * spanX, minXY(2) + insetY
                maxXY(1) - 0.1 * spanX, maxXY(2) - insetY
                minXY(1) + 0.1 * spanX, maxXY(2) - insetY
            ];
            trackPoints = trackPoints * [cos(axisHeading), -sin(axisHeading); sin(axisHeading), cos(axisHeading)];
            trackPoints = trackPoints + centroid;
            waypoints = PatrolRouteBuilder.samplePolygonWaypoints(trackPoints, numPoints);
        end

        function waypoints = polygonRoute(polygon, numPoints)
            corners = PatrolRouteBuilder.orderPolygonVertices(polygon);
            centroid = mean(corners, 1);
            expanded = centroid + 1.15 * (corners - centroid);
            waypoints = PatrolRouteBuilder.samplePolygonWaypoints(expanded, numPoints);
        end

        function ordered = orderPolygonVertices(polygon)
            centroid = mean(polygon, 1);
            angles = atan2(polygon(:, 2) - centroid(2), polygon(:, 1) - centroid(1));
            [~, order] = sort(angles);
            ordered = polygon(order, :);
        end

        function waypoints = samplePolygonWaypoints(polygon, numPoints)
            if numPoints <= size(polygon, 1)
                indices = round(linspace(1, size(polygon, 1), numPoints));
                waypoints = polygon(indices, :);
                return;
            end

            perimeterPoints = PatrolRouteBuilder.polygonPerimeterPoints(polygon);
            sampleIdx = round(linspace(1, size(perimeterPoints, 1), numPoints));
            waypoints = perimeterPoints(sampleIdx, :);
        end

        function perimeterPoints = polygonPerimeterPoints(polygon)
            perimeterPoints = zeros(0, 2);
            vertexCount = size(polygon, 1);

            for vertexIdx = 1:vertexCount
                nextIdx = mod(vertexIdx, vertexCount) + 1;
                startPoint = polygon(vertexIdx, :);
                endPoint = polygon(nextIdx, :);
                segmentLength = norm(endPoint - startPoint);
                numSamples = max(2, ceil(segmentLength / 120));
                for sampleIdx = 1:numSamples
                    t = (sampleIdx - 1) / numSamples;
                    perimeterPoints(end + 1, :) = startPoint + t * (endPoint - startPoint); %#ok<AGROW>
                end
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

        function tf = isInsidePolygon(point2d, polygon)
            x = point2d(1);
            y = point2d(2);
            inside = false;
            vertexCount = size(polygon, 1);
            j = vertexCount;

            for i = 1:vertexCount
                yi = polygon(i, 2);
                yj = polygon(j, 2);
                xi = polygon(i, 1);
                xj = polygon(j, 1);
                intersects = ((yi > y) ~= (yj > y)) && ...
                    (x < (xj - xi) * (y - yi) / max(yj - yi, eps) + xi);
                if intersects
                    inside = ~inside;
                end
                j = i;
            end

            tf = inside;
        end
    end
end
