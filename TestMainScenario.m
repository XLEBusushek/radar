% TestMainScenario  Валидация финального сценария main.m (ТЗ №7).

function TestMainScenario()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Main Scenario Validation ===\n\n');

    tempDir = tempname;
    mkdir(tempDir);

    errors = 0;
    oldDir = pwd;

    try
        numFalse = 2;
        numGround = 2;
        numAir = 2;
        boxSize = [1000, 1000, 300];
        outputPeriod = 5;
        duration = 30;
        dt = 1;
        randomSeed = 42;
        csvFilename = fullfile(tempDir, 'radar_output.csv');
        matFilename = fullfile(tempDir, 'radar_output.mat');

        cd(projectRoot);
        run('main.m');

        expectedTargetCount = numFalse + numGround + numAir;
        expectedFrameCount = duration / outputPeriod + 1;

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
                fprintf('ERROR: Target count mismatch in CSV.\n');
                errors = errors + 1;
            end

            uniqueTimes = unique(csvTable.Time);
            if numel(uniqueTimes) ~= expectedFrameCount
                fprintf('ERROR: Output frame count mismatch in CSV.\n');
                errors = errors + 1;
            end
        end

        if ~tryPlotFlightMap(projectRoot)
            fprintf('ERROR: PlotFlightMap failed.\n');
            errors = errors + 1;
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
        fprintf('Main Scenario Validation PASSED\n');
    else
        fprintf('Main Scenario Validation FAILED\n');
    end
end

function ok = tryPlotFlightMap(projectRoot)
    ok = true;

    try
        addpath(projectRoot);
        setupRadarPaths();

        config.NumFalse = 1;
        config.NumGround = 1;
        config.NumAirplaneUAV = 1;
        config.NumQuadcopter = 0;
        config.BoxSize = [500, 500, 200];
        config.Duration = 10;
        config.Dt = 1;
        config.OutputPeriod = 5;
        config.RandomSeed = 1;

        result = SimulationEngine().run(config);
        PlotFlightMap(result);
    catch
        ok = false;
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
