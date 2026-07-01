% TestMainScenario  Валидация main.m для области 2000x2000 м (ТЗ №15.2).

function TestMainScenario()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Main Scenario 2000m Validation ===\n\n');

    tempDir = tempname;
    mkdir(tempDir);

    errors = 0;
    oldDir = pwd;

    boxSize = [2000, 2000, 500];
    numFalse = 6;
    numGround = 3;
    numAirplaneUAV = 2;
    numQuadcopter = 3;
    outputPeriod = 5;
    duration = 30;
    dt = 1;
    randomSeed = 42;
    expectedTargetCount = numFalse + numGround + numAirplaneUAV + numQuadcopter;
    expectedFrameCount = duration / outputPeriod + 1;

    try
        cd(projectRoot);
        csvFilename = fullfile(tempDir, 'radar_output.csv');
        matFilename = fullfile(tempDir, 'radar_output.mat');
        run('main.m');

        if ~isfile(csvFilename)
            fprintf('ERROR: CSV file was not created.\n');
            errors = errors + 1;
        end

        if ~isfile(matFilename)
            fprintf('ERROR: MAT file was not created.\n');
            errors = errors + 1;
        end

        if isfile(matFilename)
            loaded = load(matFilename);
            if ~isfield(loaded, 'radarOutputs')
                fprintf('ERROR: MAT file missing radarOutputs.\n');
                errors = errors + 1;
            elseif numel(loaded.radarOutputs) ~= expectedFrameCount
                fprintf('ERROR: Output frame count mismatch in MAT file.\n');
                errors = errors + 1;
            end
        end

        if isfile(csvFilename)
            csvTable = readtable(csvFilename);
            uniqueIds = unique(csvTable.ID);
            if numel(uniqueIds) ~= expectedTargetCount
                fprintf('ERROR: Expected %d targets, found %d in CSV.\n', ...
                    expectedTargetCount, numel(uniqueIds));
                errors = errors + 1;
            end

            uniqueTimes = unique(csvTable.Time);
            if numel(uniqueTimes) ~= expectedFrameCount
                fprintf('ERROR: Output frame count mismatch in CSV.\n');
                errors = errors + 1;
            end
        end

        errors = errors + validateEnvironmentScale(boxSize, randomSeed);
        errors = errors + validateTargetsInsideArea(boxSize, numFalse, numGround, ...
            numAirplaneUAV, numQuadcopter, duration, dt, randomSeed, outputPeriod);

        plotBoundsErrors = validatePlotBounds(boxSize, numFalse, numGround, ...
            numAirplaneUAV, numQuadcopter, duration, dt, randomSeed, outputPeriod);
        errors = errors + plotBoundsErrors;
        if plotBoundsErrors == 0
            fprintf('Plot Bounds 2000m Validation PASSED\n');
        else
            fprintf('Plot Bounds 2000m Validation FAILED\n');
        end
    catch runError
        fprintf('ERROR: main.m execution failed: %s\n', runError.message);
        errors = errors + 1;
    end

    closeTestFigures();
    cd(oldDir);

    if isfolder(tempDir)
        rmdir(tempDir, 's');
    end

    cleanupAccidentalOutputs(projectRoot);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Main Scenario 2000m Validation PASSED\n');
    else
        fprintf('Main Scenario 2000m Validation FAILED\n');
    end
end

function errors = validateEnvironmentScale(boxSize, randomSeed)
    errors = 0;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', boxSize, ...
        'RandomSeed', randomSeed, ...
        'MinAltitude', 0, ...
        'MaxAltitude', boxSize(3), ...
        'SimulationTime', 120, ...
        'TimeStep', 1);

    if ~isequal(environment.BoxSize, boxSize)
        fprintf('ERROR: Environment BoxSize does not match requested size.\n');
        errors = errors + 1;
    end

    if isempty(environment.RoadNetwork.Segments)
        fprintf('ERROR: Road network is empty for 2000 m area.\n');
        errors = errors + 1;
    end

    if numel(environment.PatrolZones) < 2
        fprintf('ERROR: PatrolZones were not generated for 2000 m area.\n');
        errors = errors + 1;
    end

    patrolSpan = max(environment.PatrolZones(1).Polygon(:, 1)) - ...
        min(environment.PatrolZones(1).Polygon(:, 1));
    if patrolSpan < 200
        fprintf('ERROR: PatrolZones are too small for 2000 m area.\n');
        errors = errors + 1;
    end

    if numel(environment.InspectionZones) < 5
        fprintf('ERROR: InspectionZones were not distributed for 2000 m area.\n');
        errors = errors + 1;
    end

    treeCenters = vertcat(environment.TreeZones.Center);
    maxCenterRadius = max(vecnorm(treeCenters, 2, 2));
    if maxCenterRadius < 0.30 * min(diff(environment.XLimits), diff(environment.YLimits))
        fprintf('ERROR: TreeZones cluster too close to the origin.\n');
        errors = errors + 1;
    end

    if size(environment.SpawnPoints.Ground, 1) < 8
        fprintf('ERROR: SpawnPoints were not scaled for 2000 m area.\n');
        errors = errors + 1;
    end

    spawnFields = {'Ground', 'Bird', 'Airplane', 'Quadcopter'};
    for fieldIdx = 1:numel(spawnFields)
        points = environment.SpawnPoints.(spawnFields{fieldIdx});
        for pointIdx = 1:size(points, 1)
            if ~Environment.isInsideEnvironment(environment, points(pointIdx, :))
                fprintf('ERROR: %s spawn point %d is outside the environment.\n', ...
                    spawnFields{fieldIdx}, pointIdx);
                errors = errors + 1;
            end
        end
    end
end

function errors = validateTargetsInsideArea(boxSize, numFalse, numGround, ...
        numAirplaneUAV, numQuadcopter, duration, dt, randomSeed, outputPeriod)
    errors = 0;

    config.NumFalse = numFalse;
    config.NumGround = numGround;
    config.NumAirplaneUAV = numAirplaneUAV;
    config.NumQuadcopter = numQuadcopter;
    config.BoxSize = boxSize;
    config.Duration = duration;
    config.Dt = dt;
    config.OutputPeriod = outputPeriod;
    config.RandomSeed = randomSeed;

    result = SimulationEngine().run(config);

    if numel(result.Targets) ~= numFalse + numGround + numAirplaneUAV + numQuadcopter
        fprintf('ERROR: Simulation target count mismatch.\n');
        errors = errors + 1;
    end

    if numel(result.OutputFrames) ~= duration / outputPeriod + 1
        fprintf('ERROR: OutputFrames count mismatch.\n');
        errors = errors + 1;
    end

    for targetIdx = 1:numel(result.Targets)
        target = result.Targets{targetIdx};
        if ~Environment.isInsideEnvironment(result.Environment, target.Position)
            fprintf('ERROR: Target %d is outside the simulation area.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function errors = validatePlotBounds(boxSize, numFalse, numGround, numAirplaneUAV, numQuadcopter, duration, dt, randomSeed, outputPeriod)
    errors = 0;

    fprintf('\n=== Plot Bounds 2000m Validation ===\n\n');

    try
        config.NumFalse = numFalse;
        config.NumGround = numGround;
        config.NumAirplaneUAV = numAirplaneUAV;
        config.NumQuadcopter = numQuadcopter;
        config.BoxSize = boxSize;
        config.Duration = duration;
        config.Dt = dt;
        config.OutputPeriod = outputPeriod;
        config.RandomSeed = randomSeed;

        result = SimulationEngine().run(config);
        PlotFlightMap(result);

        expectedX = [-boxSize(1) / 2, boxSize(1) / 2];
        expectedY = [-boxSize(2) / 2, boxSize(2) / 2];
        expectedZ = [0, boxSize(3)];

        fig = findobj(0, 'Type', 'figure', 'Name', 'Flight Map');
        if isempty(fig)
            fprintf('ERROR: PlotFlightMap did not create a figure.\n');
            errors = errors + 1;
            return;
        end

        ax3d = findAxisByTitle(fig, '3D trajectories');
        axTop = findAxisByTitle(fig, 'Top view (X-Y)');
        axAlt = findAxisByTitle(fig, 'Altitude vs time');

        if isempty(ax3d) || isempty(axTop) || isempty(axAlt)
            fprintf('ERROR: PlotFlightMap is missing expected axes.\n');
            errors = errors + 1;
            return;
        end

        if ~limitsMatch(xlim(ax3d), expectedX)
            fprintf('ERROR: 3D X limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                xlim(ax3d), expectedX);
            errors = errors + 1;
        end

        if ~limitsMatch(ylim(ax3d), expectedY)
            fprintf('ERROR: 3D Y limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                ylim(ax3d), expectedY);
            errors = errors + 1;
        end

        if ~limitsMatch(zlim(ax3d), expectedZ)
            fprintf('ERROR: 3D Z limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                zlim(ax3d), expectedZ);
            errors = errors + 1;
        end

        if ~limitsMatch(xlim(axTop), expectedX)
            fprintf('ERROR: Top view X limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                xlim(axTop), expectedX);
            errors = errors + 1;
        end

        if ~limitsMatch(ylim(axTop), expectedY)
            fprintf('ERROR: Top view Y limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                ylim(axTop), expectedY);
            errors = errors + 1;
        end

        if ~limitsMatch(ylim(axAlt), expectedZ)
            fprintf('ERROR: Altitude plot Y limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
                ylim(axAlt), expectedZ);
            errors = errors + 1;
        end

        errors = errors + validateBoundsWireframe(ax3d, expectedX, expectedY, expectedZ);
        errors = errors + validatePlotSourceHasNoLegacyBounds();
    catch plotError
        fprintf('ERROR: PlotFlightMap failed: %s\n', plotError.message);
        errors = errors + 1;
    end
end

function ax = findAxisByTitle(fig, titleText)
    ax = [];
    axesHandles = findobj(fig, 'Type', 'axes');
    for k = 1:numel(axesHandles)
        titleHandle = get(axesHandles(k), 'Title');
        if isprop(titleHandle, 'String') && strcmp(char(titleHandle.String), titleText)
            ax = axesHandles(k);
            return;
        end
    end
end

function tf = limitsMatch(actual, expected)
    tf = numel(actual) == 2 && numel(expected) == 2 && ...
        all(abs(actual - expected) <= 1e-6);
end

function errors = validateBoundsWireframe(ax3d, expectedX, expectedY, expectedZ)
    errors = 0;
    boundLines = findobj(ax3d, 'Type', 'Line', 'LineStyle', '--', ...
        'Color', [0.4, 0.4, 0.4], '-depth', 1);

    if isempty(boundLines)
        fprintf('ERROR: Simulation bounds wireframe was not drawn.\n');
        errors = errors + 1;
        return;
    end

    allX = [];
    allY = [];
    allZ = [];
    for lineIdx = 1:numel(boundLines)
        allX = [allX, get(boundLines(lineIdx), 'XData')]; %#ok<AGROW>
        allY = [allY, get(boundLines(lineIdx), 'YData')]; %#ok<AGROW>
        allZ = [allZ, get(boundLines(lineIdx), 'ZData')]; %#ok<AGROW>
    end

    if ~any(abs(allX - expectedX(1)) < 1) || ~any(abs(allX - expectedX(2)) < 1)
        fprintf('ERROR: Bounds wireframe X extent does not match boxSize.\n');
        errors = errors + 1;
    end

    if ~any(abs(allY - expectedY(1)) < 1) || ~any(abs(allY - expectedY(2)) < 1)
        fprintf('ERROR: Bounds wireframe Y extent does not match boxSize.\n');
        errors = errors + 1;
    end

    if ~any(abs(allZ - expectedZ(1)) < 1) || ~any(abs(allZ - expectedZ(2)) < 1)
        fprintf('ERROR: Bounds wireframe Z extent does not match boxSize.\n');
        errors = errors + 1;
    end

    legacyX = [-500, 500];
    legacyZ = 300;
    if any(abs(allX - legacyX(1)) < 1) && any(abs(allX - legacyX(2)) < 1) && ...
            ~any(abs(allX - expectedX(1)) < 1)
        fprintf('ERROR: PlotFlightMap still uses legacy 500 m X bounds.\n');
        errors = errors + 1;
    end

    if any(abs(allZ - legacyZ) < 1) && ~any(abs(allZ - expectedZ(2)) < 1)
        fprintf('ERROR: PlotFlightMap still uses legacy 300 m Z bounds.\n');
        errors = errors + 1;
    end
end

function errors = validatePlotSourceHasNoLegacyBounds()
    errors = 0;
    projectRoot = fileparts(mfilename('fullpath'));
    sourcePath = fullfile(projectRoot, 'simulation', 'PlotFlightMap.m');
    sourceText = fileread(sourcePath);

    forbiddenPatterns = {
        'xlim([-500'
        'xlim([ -500'
        'ylim([-500'
        'ylim([ -500'
        'zlim([0, 300]'
        'zlim([0 300]'
        'zlim([0,300]'
    };

    for patternIdx = 1:numel(forbiddenPatterns)
        if contains(sourceText, forbiddenPatterns{patternIdx})
            fprintf('ERROR: PlotFlightMap contains legacy hardcoded limit: %s\n', ...
                forbiddenPatterns{patternIdx});
            errors = errors + 1;
        end
    end
end

function closeTestFigures()
    figHandles = findall(0, 'Type', 'figure');
    if ~isempty(figHandles)
        close(figHandles);
    end
end

function cleanupAccidentalOutputs(projectRoot)
    accidentalFiles = {
        fullfile(projectRoot, 'radar_output.csv')
        fullfile(projectRoot, 'radar_output.mat')
    };

    for k = 1:numel(accidentalFiles)
        if isfile(accidentalFiles{k})
            delete(accidentalFiles{k});
        end
    end
end
