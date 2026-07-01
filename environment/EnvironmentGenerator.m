classdef EnvironmentGenerator
    % EnvironmentGenerator  Генератор общего описания среды моделирования.

    methods (Static)
        function environment = generate(args)
            arguments
                args.BoxSize (1, 3) double {mustBePositive}
                args.RandomSeed (1, 1) double = 42
                args.MinAltitude (1, 1) double {mustBeNonnegative} = 0
                args.MaxAltitude (1, 1) double {mustBeNonnegative} = nan
                args.SimulationTime (1, 1) double {mustBePositive} = 300
                args.TimeStep (1, 1) double {mustBePositive} = 1
            end

            if isnan(args.MaxAltitude)
                args.MaxAltitude = args.BoxSize(3);
            end

            if args.MaxAltitude < args.MinAltitude
                error('EnvironmentGenerator:InvalidAltitude', ...
                    'MaxAltitude must be greater than or equal to MinAltitude.');
            end

            stream = RandStream('mt19937ar', 'Seed', args.RandomSeed);
            halfXY = args.BoxSize(1:2) / 2;
            xLimits = [-halfXY(1), halfXY(1)];
            yLimits = [-halfXY(2), halfXY(2)];
            zLimits = [args.MinAltitude, args.MaxAltitude];

            environment = struct();
            environment.BoxSize = args.BoxSize;
            environment.RandomSeed = args.RandomSeed;
            environment.AreaSize = args.BoxSize;
            environment.XLimits = xLimits;
            environment.YLimits = yLimits;
            environment.ZLimits = zLimits;
            environment.MinAltitude = args.MinAltitude;
            environment.MaxAltitude = args.MaxAltitude;
            environment.SimulationTime = args.SimulationTime;
            environment.TimeStep = args.TimeStep;

            environment.Terrain = Terrain(xLimits, yLimits, args.RandomSeed);
            environment.RoadNetwork = EnvironmentGenerator.buildRoadNetwork(xLimits, yLimits, stream);
            environment.TreeZones = EnvironmentGenerator.buildTreeZones(xLimits, yLimits, stream);
            environment.PatrolZones = EnvironmentGenerator.buildPatrolZones( ...
                xLimits, yLimits, args.MinAltitude, args.MaxAltitude, stream);
            environment.InspectionZones = EnvironmentGenerator.buildInspectionZones(xLimits, yLimits, stream);
            environment.NoFlyZones = EnvironmentGenerator.buildNoFlyZones();
            environment.SpawnPoints = EnvironmentGenerator.buildSpawnPoints( ...
                environment, stream, args.RandomSeed);
        end
    end

    methods (Static, Access = private)
        function roadNetwork = buildRoadNetwork(xLimits, yLimits, stream)
            marginRatio = 0.08;
            xSpan = diff(xLimits);
            ySpan = diff(yLimits);
            xMargin = marginRatio * xSpan;
            yMargin = marginRatio * ySpan;

            numX = 3 + stream.randi(3);
            numY = 3 + stream.randi(3);
            xLines = linspace(xLimits(1) + xMargin, xLimits(2) - xMargin, numX);
            yLines = linspace(yLimits(1) + yMargin, yLimits(2) - yMargin, numY);
            yBounds = [yLimits(1) + yMargin, yLimits(2) - yMargin];
            xBounds = [xLimits(1) + xMargin, xLimits(2) - xMargin];

            segments = zeros(0, 4);

            for xIdx = 1:numel(xLines)
                yPoints = unique([yBounds(1), yLines, yBounds(2)]);
                for pointIdx = 1:(numel(yPoints) - 1)
                    segments(end + 1, :) = [ ...
                        xLines(xIdx), yPoints(pointIdx), ...
                        xLines(xIdx), yPoints(pointIdx + 1)]; %#ok<AGROW>
                end
            end

            for yIdx = 1:numel(yLines)
                xPoints = unique([xBounds(1), xLines, xBounds(2)]);
                for pointIdx = 1:(numel(xPoints) - 1)
                    segments(end + 1, :) = [ ...
                        xPoints(pointIdx), yLines(yIdx), ...
                        xPoints(pointIdx + 1), yLines(yIdx)]; %#ok<AGROW>
                end
            end

            roadNetwork = struct( ...
                'Segments', segments, ...
                'XLimits', xLimits, ...
                'YLimits', yLimits);
        end

        function treeZones = buildTreeZones(xLimits, yLimits, stream)
            zoneCount = 5 + stream.randi(16);
            treeZones = repmat(EnvironmentGenerator.emptyTreeZone(), zoneCount, 1);

            for zoneIdx = 1:zoneCount
                treeZones(zoneIdx).Center = [
                    EnvironmentGenerator.randomInRange(xLimits, stream), ...
                    EnvironmentGenerator.randomInRange(yLimits, stream)
                ];
                treeZones(zoneIdx).Radius = 30 + 70 * stream.rand();
                treeZones(zoneIdx).Height = 5 + 35 * stream.rand();
            end
        end

        function patrolZones = buildPatrolZones(xLimits, yLimits, minAltitude, maxAltitude, stream)
            zoneCount = 2 + stream.randi(5);
            patrolZones = repmat(EnvironmentGenerator.emptyPatrolZone(), zoneCount, 1);
            altitudeSpan = maxAltitude - minAltitude;

            for zoneIdx = 1:zoneCount
                center = [
                    EnvironmentGenerator.randomInRange(xLimits, stream, 0.2), ...
                    EnvironmentGenerator.randomInRange(yLimits, stream, 0.2)
                ];
                halfWidth = 0.08 * diff(xLimits) * (0.8 + 0.4 * stream.rand());
                halfHeight = 0.08 * diff(yLimits) * (0.8 + 0.4 * stream.rand());
                patrolZones(zoneIdx).Polygon = [
                    center(1) - halfWidth, center(2) - halfHeight
                    center(1) + halfWidth, center(2) - halfHeight
                    center(1) + halfWidth, center(2) + halfHeight
                    center(1) - halfWidth, center(2) + halfHeight
                ];
                patrolZones(zoneIdx).PreferredAltitude = minAltitude + altitudeSpan * (0.35 + 0.45 * stream.rand());
                patrolZones(zoneIdx).Priority = 1 + stream.randi(3);
            end
        end

        function inspectionZones = buildInspectionZones(xLimits, yLimits, stream)
            zoneCount = 5 + stream.randi(11);
            inspectionZones = repmat(EnvironmentGenerator.emptyInspectionZone(), zoneCount, 1);

            for zoneIdx = 1:zoneCount
                inspectionZones(zoneIdx).Center = [
                    EnvironmentGenerator.randomInRange(xLimits, stream), ...
                    EnvironmentGenerator.randomInRange(yLimits, stream)
                ];
                inspectionZones(zoneIdx).Radius = 25 + 60 * stream.rand();
                inspectionZones(zoneIdx).Importance = 0.2 + 0.8 * stream.rand();
            end
        end

        function noFlyZones = buildNoFlyZones()
            noFlyZones = struct( ...
                'Center', {}, ...
                'Radius', {}, ...
                'Polygon', {}, ...
                'MinAltitude', {}, ...
                'MaxAltitude', {});
        end

        function spawnPoints = buildSpawnPoints(environment, stream, randomSeed)
            if nargin < 3
                randomSeed = environment.RandomSeed;
            end

            areaScale = sqrt(diff(environment.XLimits) * diff(environment.YLimits)) / 1000;
            targetGround = max(8, round(8 * areaScale));
            targetBird = max(8, round(8 * areaScale));
            targetAirplane = max(6, round(6 * areaScale));
            targetQuad = max(8, round(8 * areaScale));

            spawnPoints = struct();
            spawnPoints.Ground = EnvironmentGenerator.spawnOnRoads(environment, 8, stream);
            spawnPoints.Bird = EnvironmentGenerator.spawnNearTreeZones(environment, 8, stream);
            spawnPoints.False = spawnPoints.Bird;
            spawnPoints.Airplane = EnvironmentGenerator.spawnInPatrolZones(environment, 6, stream);
            spawnPoints.AirplaneUAV = spawnPoints.Airplane;
            spawnPoints.Quadcopter = EnvironmentGenerator.spawnInInspectionZones(environment, 8, stream);

            extraGround = targetGround - size(spawnPoints.Ground, 1);
            extraBird = targetBird - size(spawnPoints.Bird, 1);
            extraAirplane = targetAirplane - size(spawnPoints.Airplane, 1);
            extraQuad = targetQuad - size(spawnPoints.Quadcopter, 1);

            if extraGround > 0 || extraBird > 0 || extraAirplane > 0 || extraQuad > 0
                supplementStream = RandStream('mt19937ar', 'Seed', round(randomSeed + 519));
                if extraGround > 0
                    spawnPoints.Ground = [
                        spawnPoints.Ground
                        EnvironmentGenerator.spawnOnRoads(environment, extraGround, supplementStream)];
                end
                if extraBird > 0
                    extraBirdPoints = EnvironmentGenerator.spawnNearTreeZones( ...
                        environment, extraBird, supplementStream);
                    spawnPoints.Bird = [spawnPoints.Bird; extraBirdPoints];
                    spawnPoints.False = spawnPoints.Bird;
                end
                if extraAirplane > 0
                    extraAirplanePoints = EnvironmentGenerator.spawnInPatrolZones( ...
                        environment, extraAirplane, supplementStream);
                    spawnPoints.Airplane = [spawnPoints.Airplane; extraAirplanePoints];
                    spawnPoints.AirplaneUAV = spawnPoints.Airplane;
                end
                if extraQuad > 0
                    spawnPoints.Quadcopter = [
                        spawnPoints.Quadcopter
                        EnvironmentGenerator.spawnInInspectionZones(environment, extraQuad, supplementStream)];
                end
            end
        end

        function points = spawnOnRoads(environment, pointCount, stream)
            segments = environment.RoadNetwork.Segments;
            points = zeros(pointCount, 3);
            marginRatio = 0.12;
            xSafe = environment.XLimits + marginRatio * diff(environment.XLimits) .* [-1, 1];
            ySafe = environment.YLimits + marginRatio * diff(environment.YLimits) .* [-1, 1];

            for pointIdx = 1:pointCount
                for attempt = 1:30
                    segment = segments(stream.randi(size(segments, 1)), :);
                    t = 0.15 + 0.7 * stream.rand();
                    xy = segment(1:2) + t * (segment(3:4) - segment(1:2));
                    if xy(1) >= xSafe(1) && xy(1) <= xSafe(2) && ...
                            xy(2) >= ySafe(1) && xy(2) <= ySafe(2)
                        break;
                    end
                end

                points(pointIdx, 1:2) = xy;
                points(pointIdx, 3) = environment.Terrain.Height(xy(1), xy(2));
            end
        end

        function points = spawnNearTreeZones(environment, pointCount, stream)
            zones = environment.TreeZones;
            points = zeros(pointCount, 3);

            for pointIdx = 1:pointCount
                zone = zones(stream.randi(numel(zones)));
                angle = 2 * pi * stream.rand();
                radius = zone.Radius * (0.2 + 0.8 * stream.rand());
                xy = zone.Center + radius * [cos(angle), sin(angle)];
                points(pointIdx, 1:2) = xy;
                points(pointIdx, 3) = zone.Height + 5 + 35 * stream.rand();
            end
        end

        function points = spawnInPatrolZones(environment, pointCount, stream)
            zones = environment.PatrolZones;
            points = zeros(pointCount, 3);

            for pointIdx = 1:pointCount
                zone = zones(stream.randi(numel(zones)));
                polygon = zone.Polygon;
                minXY = min(polygon, [], 1);
                maxXY = max(polygon, [], 1);
                xy = [
                    minXY(1) + (maxXY(1) - minXY(1)) * stream.rand(), ...
                    minXY(2) + (maxXY(2) - minXY(2)) * stream.rand()
                ];
                points(pointIdx, 1:2) = xy;
                points(pointIdx, 3) = zone.PreferredAltitude;
            end
        end

        function points = spawnInInspectionZones(environment, pointCount, stream)
            zones = environment.InspectionZones;
            points = zeros(pointCount, 3);

            for pointIdx = 1:pointCount
                zone = zones(stream.randi(numel(zones)));
                angle = 2 * pi * stream.rand();
                radius = zone.Radius * sqrt(stream.rand());
                xy = zone.Center + radius * [cos(angle), sin(angle)];
                points(pointIdx, 1:2) = xy;
                points(pointIdx, 3) = 40 + 80 * stream.rand();
            end
        end

        function zone = emptyTreeZone()
            zone = struct('Center', [0, 0], 'Radius', 0, 'Height', 0);
        end

        function zone = emptyPatrolZone()
            zone = struct('Polygon', zeros(0, 2), 'PreferredAltitude', 0, 'Priority', 0);
        end

        function zone = emptyInspectionZone()
            zone = struct('Center', [0, 0], 'Radius', 0, 'Importance', 0);
        end

        function value = randomInRange(limits, stream, marginRatio)
            if nargin < 3
                marginRatio = 0.1;
            end

            span = diff(limits);
            margin = marginRatio * span;
            value = limits(1) + margin + (span - 2 * margin) * stream.rand();
        end
    end
end
