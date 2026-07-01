classdef Environment
    % Environment  API общего описания среды моделирования.

    methods (Static)
        function roadInfo = findNearestRoad(environment, position)
            arguments
                environment (1, 1) struct
                position (1, 3) double
            end

            Environment.validateEnvironment(environment);
            segments = environment.RoadNetwork.Segments;
            bestDistance = inf;
            bestPoint = [nan, nan, position(3)];
            bestIndex = 0;
            bestHeading = 0;

            for segmentIdx = 1:size(segments, 1)
                segment = segments(segmentIdx, :);
                [point2d, distance] = Environment.closestPointOnSegment(position(1:2), segment);
                if distance < bestDistance
                    bestDistance = distance;
                    bestPoint = [point2d, position(3)];
                    bestIndex = segmentIdx;
                    bestHeading = atan2(segment(4) - segment(2), segment(3) - segment(1));
                end
            end

            roadInfo = struct( ...
                'Point', bestPoint, ...
                'SegmentIndex', bestIndex, ...
                'Distance', bestDistance, ...
                'Heading', bestHeading);
        end

        function zoneInfo = findNearestTreeZone(environment, position)
            zoneInfo = Environment.findNearestRadialZone( ...
                environment, position, environment.TreeZones, 'Height');
        end

        function zoneInfo = findNearestInspectionZone(environment, position)
            zoneInfo = Environment.findNearestRadialZone( ...
                environment, position, environment.InspectionZones, 'Importance');
        end

        function zoneInfo = findNearestPatrolZone(environment, position)
            arguments
                environment (1, 1) struct
                position (1, 3) double
            end

            Environment.validateEnvironment(environment);
            zones = environment.PatrolZones;
            bestDistance = inf;
            bestIndex = 0;

            for zoneIdx = 1:numel(zones)
                polygon = zones(zoneIdx).Polygon;
                centroid = mean(polygon, 1);
                distance = norm(position(1:2) - centroid);
                if distance < bestDistance
                    bestDistance = distance;
                    bestIndex = zoneIdx;
                end
            end

            if bestIndex == 0
                zoneInfo = Environment.emptyZoneInfo();
                return;
            end

            zone = zones(bestIndex);
            zoneInfo = struct( ...
                'Index', bestIndex, ...
                'Distance', bestDistance, ...
                'Zone', zone, ...
                'PreferredAltitude', zone.PreferredAltitude, ...
                'Priority', zone.Priority);
        end

        function tf = isInsideNoFlyZone(environment, position)
            arguments
                environment (1, 1) struct
                position (1, 3) double
            end

            Environment.validateEnvironment(environment);
            tf = false;

            for zoneIdx = 1:numel(environment.NoFlyZones)
                zone = environment.NoFlyZones(zoneIdx);
                if isfield(zone, 'Polygon')
                    if Environment.isPointInsidePolygon(position(1:2), zone.Polygon)
                        tf = true;
                        return;
                    end
                elseif isfield(zone, 'Center') && isfield(zone, 'Radius')
                    if norm(position(1:2) - zone.Center) <= zone.Radius
                        tf = true;
                        return;
                    end
                end
            end
        end

        function tf = isInsideEnvironment(environment, position)
            arguments
                environment (1, 1) struct
                position (1, 3) double
            end

            Environment.validateEnvironment(environment);
            tf = position(1) >= environment.XLimits(1) && position(1) <= environment.XLimits(2) && ...
                position(2) >= environment.YLimits(1) && position(2) <= environment.YLimits(2) && ...
                position(3) >= environment.ZLimits(1) && position(3) <= environment.ZLimits(2);
        end
    end

    methods (Static, Access = private)
        function validateEnvironment(environment)
            requiredFields = {
                'BoxSize'
                'RoadNetwork'
                'PatrolZones'
                'InspectionZones'
                'TreeZones'
                'NoFlyZones'
                'Terrain'
                'SpawnPoints'
                'RandomSeed'
                'XLimits'
                'YLimits'
                'ZLimits'
            };

            for fieldIdx = 1:numel(requiredFields)
                fieldName = requiredFields{fieldIdx};
                if ~isfield(environment, fieldName)
                    error('Environment:MissingField', ...
                        'Environment must contain field: %s', fieldName);
                end
            end
        end

        function zoneInfo = findNearestRadialZone(environment, position, zones, attributeName)
            Environment.validateEnvironment(environment);
            bestDistance = inf;
            bestIndex = 0;

            for zoneIdx = 1:numel(zones)
                distance = norm(position(1:2) - zones(zoneIdx).Center);
                if distance < bestDistance
                    bestDistance = distance;
                    bestIndex = zoneIdx;
                end
            end

            if bestIndex == 0
                zoneInfo = Environment.emptyZoneInfo();
                return;
            end

            zone = zones(bestIndex);
            zoneInfo = struct( ...
                'Index', bestIndex, ...
                'Distance', bestDistance, ...
                'Zone', zone);
            zoneInfo.(attributeName) = zone.(attributeName);
        end

        function zoneInfo = emptyZoneInfo()
            zoneInfo = struct( ...
                'Index', 0, ...
                'Distance', inf, ...
                'Zone', struct());
        end

        function [closestPoint2d, distance] = closestPointOnSegment(queryPoint2d, segment)
            startPoint = segment(1:2);
            endPoint = segment(3:4);
            segmentVector = endPoint - startPoint;
            segmentLengthSq = dot(segmentVector, segmentVector);

            if segmentLengthSq < 1e-9
                closestPoint2d = startPoint;
                distance = norm(queryPoint2d - startPoint);
                return;
            end

            projection = dot(queryPoint2d - startPoint, segmentVector) / segmentLengthSq;
            projection = min(max(projection, 0), 1);
            closestPoint2d = startPoint + projection * segmentVector;
            distance = norm(queryPoint2d - closestPoint2d);
        end

        function tf = isPointInsidePolygon(point2d, polygon)
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
