% TestEnvironmentGenerator  Валидация EnvironmentGenerator (ТЗ №13.3).

function TestEnvironmentGenerator()
    rng(51);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Environment Generator Validation ===\n\n');

    boxSize = [2000, 2000, 500];
    randomSeed = 51;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', boxSize, ...
        'RandomSeed', randomSeed, ...
        'MinAltitude', 0, ...
        'MaxAltitude', boxSize(3), ...
        'SimulationTime', 120, ...
        'TimeStep', 1);

    errors = 0;
    errors = errors + validateEnvironmentCreated(environment);
    errors = errors + validateZonesInsideBox(environment);
    errors = errors + validateZonePresence(environment);
    errors = errors + validateSpawnPoints(environment);
    errors = errors + validateTerrain(environment);
    errors = errors + validateTreeZoneSpread(environment);
    errors = errors + validateApi(environment);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Environment Generator Validation PASSED\n');
    else
        fprintf('Environment Generator Validation FAILED\n');
    end
end

function errors = validateEnvironmentCreated(environment)
    errors = 0;
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
    };

    for fieldIdx = 1:numel(requiredFields)
        if ~isfield(environment, requiredFields{fieldIdx})
            fprintf('ERROR: Environment missing field %s.\n', requiredFields{fieldIdx});
            errors = errors + 1;
        end
    end
end

function errors = validateZonesInsideBox(environment)
    errors = 0;
    xLimits = environment.XLimits;
    yLimits = environment.YLimits;

    for zoneIdx = 1:numel(environment.TreeZones)
        center = environment.TreeZones(zoneIdx).Center;
        if ~isInsideXY(center, xLimits, yLimits, environment.TreeZones(zoneIdx).Radius)
            fprintf('ERROR: TreeZone %d is outside box.\n', zoneIdx);
            errors = errors + 1;
        end
    end

    for zoneIdx = 1:numel(environment.InspectionZones)
        center = environment.InspectionZones(zoneIdx).Center;
        if ~isInsideXY(center, xLimits, yLimits, environment.InspectionZones(zoneIdx).Radius)
            fprintf('ERROR: InspectionZone %d is outside box.\n', zoneIdx);
            errors = errors + 1;
        end
    end

    for zoneIdx = 1:numel(environment.PatrolZones)
        polygon = environment.PatrolZones(zoneIdx).Polygon;
        if ~all(isInsideXY(polygon, xLimits, yLimits, 0))
            fprintf('ERROR: PatrolZone %d is outside box.\n', zoneIdx);
            errors = errors + 1;
        end
    end
end

function errors = validateZonePresence(environment)
    errors = 0;

    if ~isfield(environment.RoadNetwork, 'Segments') || isempty(environment.RoadNetwork.Segments)
        fprintf('ERROR: Road network has no segments.\n');
        errors = errors + 1;
    end

    if isempty(environment.PatrolZones)
        fprintf('ERROR: No patrol zones generated.\n');
        errors = errors + 1;
    end

    if isempty(environment.InspectionZones)
        fprintf('ERROR: No inspection zones generated.\n');
        errors = errors + 1;
    end

    if numel(environment.TreeZones) < 5
        fprintf('ERROR: Expected at least 5 tree zones.\n');
        errors = errors + 1;
    end
end

function errors = validateSpawnPoints(environment)
    errors = 0;
    spawnPoints = environment.SpawnPoints;
    requiredTypes = {'Ground', 'Bird', 'Airplane', 'Quadcopter'};

    for typeIdx = 1:numel(requiredTypes)
        typeName = requiredTypes{typeIdx};
        if ~isfield(spawnPoints, typeName) || isempty(spawnPoints.(typeName))
            fprintf('ERROR: Missing spawn points for %s.\n', typeName);
            errors = errors + 1;
        end
    end

    groundPoints = spawnPoints.Ground;
    for pointIdx = 1:size(groundPoints, 1)
        roadInfo = Environment.findNearestRoad(environment, groundPoints(pointIdx, :));
        if roadInfo.Distance > 1.0
            fprintf('ERROR: Ground spawn point %d is not on a road.\n', pointIdx);
            errors = errors + 1;
        end
    end

    birdPoints = spawnPoints.Bird;
    for pointIdx = 1:size(birdPoints, 1)
        treeInfo = Environment.findNearestTreeZone(environment, birdPoints(pointIdx, :));
        if treeInfo.Distance > treeInfo.Zone.Radius * 1.2
            fprintf('ERROR: Bird spawn point %d is too far from tree zone.\n', pointIdx);
            errors = errors + 1;
        end
    end

    airplanePoints = spawnPoints.Airplane;
    for pointIdx = 1:size(airplanePoints, 1)
        patrolInfo = Environment.findNearestPatrolZone(environment, airplanePoints(pointIdx, :));
        if ~isPointInsidePolygon(airplanePoints(pointIdx, 1:2), patrolInfo.Zone.Polygon)
            fprintf('ERROR: Airplane spawn point %d is outside patrol zone.\n', pointIdx);
            errors = errors + 1;
        end
    end

    quadPoints = spawnPoints.Quadcopter;
    for pointIdx = 1:size(quadPoints, 1)
        inspectInfo = Environment.findNearestInspectionZone(environment, quadPoints(pointIdx, :));
        if inspectInfo.Distance > inspectInfo.Zone.Radius * 1.05
            fprintf('ERROR: Quadcopter spawn point %d is outside inspection zone.\n', pointIdx);
            errors = errors + 1;
        end
    end
end

function errors = validateTerrain(environment)
    errors = 0;
    sampleX = linspace(environment.XLimits(1), environment.XLimits(2), 5);
    sampleY = linspace(environment.YLimits(1), environment.YLimits(2), 5);

    for x = sampleX
        for y = sampleY
            height = environment.Terrain.Height(x, y);
            if height < 0 || height > 20
                fprintf('ERROR: Terrain height %.2f outside 0..20 range.\n', height);
                errors = errors + 1;
            end
        end
    end
end

function errors = validateTreeZoneSpread(environment)
    errors = 0;
    if numel(environment.TreeZones) < 2
        return;
    end

    centers = vertcat(environment.TreeZones.Center);
    maxCenterRadius = max(vecnorm(centers, 2, 2));
    minRadius = 0.30 * min(diff(environment.XLimits), diff(environment.YLimits));
    if maxCenterRadius < minRadius
        fprintf('ERROR: TreeZones are clustered near the origin.\n');
        errors = errors + 1;
    end
end

function errors = validateApi(environment)
    errors = 0;
    queryPoint = [0, 0, 100];

    apiCalls = {
        @() Environment.findNearestRoad(environment, queryPoint)
        @() Environment.findNearestTreeZone(environment, queryPoint)
        @() Environment.findNearestInspectionZone(environment, queryPoint)
        @() Environment.findNearestPatrolZone(environment, queryPoint)
        @() Environment.isInsideNoFlyZone(environment, queryPoint)
        @() Environment.isInsideEnvironment(environment, queryPoint)
    };

    for callIdx = 1:numel(apiCalls)
        try
            apiCalls{callIdx}();
        catch apiError
            fprintf('ERROR: Environment API call %d failed: %s\n', callIdx, apiError.message);
            errors = errors + 1;
        end
    end

    if ~Environment.isInsideEnvironment(environment, queryPoint)
        fprintf('ERROR: Origin query point should be inside environment.\n');
        errors = errors + 1;
    end

    outsidePoint = [environment.XLimits(2) + 100, 0, 100];
    if Environment.isInsideEnvironment(environment, outsidePoint)
        fprintf('ERROR: Outside query point should be rejected.\n');
        errors = errors + 1;
    end

    if Environment.isInsideNoFlyZone(environment, queryPoint)
        fprintf('ERROR: Empty no-fly zones should not block query point.\n');
        errors = errors + 1;
    end
end

function tf = isInsideXY(points, xLimits, yLimits, margin)
    tf = all(points(:, 1) >= xLimits(1) - margin) && ...
        all(points(:, 1) <= xLimits(2) + margin) && ...
        all(points(:, 2) >= yLimits(1) - margin) && ...
        all(points(:, 2) <= yLimits(2) + margin);
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
