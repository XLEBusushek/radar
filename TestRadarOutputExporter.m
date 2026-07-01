% TestRadarOutputExporter  Валидация RadarOutputExporter (ТЗ №6).

function TestRadarOutputExporter()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Radar Output Exporter Validation ===\n\n');

    config = createTestConfig();
    engine = SimulationEngine();
    result = engine.run(config);

    outputs = RadarOutputExporter.exportSimulation(result);
    tableData = RadarOutputExporter.toTable(outputs);

    expectedFrameCount = config.Duration / config.OutputPeriod + 1;
    expectedTargetCount = config.NumFalse + config.NumGround + ...
        config.NumAirplaneUAV + config.NumQuadcopter;

    errors = 0;
    errors = errors + validateFrameCount(outputs, expectedFrameCount);
    errors = errors + validateOutputFrames(outputs, result.OutputFrames, expectedTargetCount);
    errors = errors + validateTable(tableData);
    errors = errors + validateFileExport(outputs, projectRoot);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Radar Output Exporter Validation PASSED\n');
    else
        fprintf('Radar Output Exporter Validation FAILED\n');
    end
end

function config = createTestConfig()
    config.NumFalse = 2;
    config.NumGround = 2;
    config.NumAirplaneUAV = 1;
    config.NumQuadcopter = 1;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 60;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 42;
end

function errors = validateFrameCount(outputs, expectedFrameCount)
    errors = 0;

    if numel(outputs) ~= expectedFrameCount
        fprintf('ERROR: Expected %d output frames, got %d.\n', ...
            expectedFrameCount, numel(outputs));
        errors = errors + 1;
    end
end

function errors = validateOutputFrames(outputs, sourceFrames, expectedTargetCount)
    errors = 0;
    tolerance = 1e-9;

    for frameIdx = 1:numel(outputs)
        outputFrame = outputs(frameIdx);
        sourceFrame = sourceFrames(frameIdx);

        if numel(outputFrame.Targets) ~= expectedTargetCount
            fprintf('ERROR: Frame %d has %d targets, expected %d.\n', ...
                frameIdx, numel(outputFrame.Targets), expectedTargetCount);
            errors = errors + 1;
            continue;
        end

        for targetIdx = 1:expectedTargetCount
            targetOutput = outputFrame.Targets{targetIdx};
            snapshot = sourceFrame.Targets{targetIdx};

            errors = errors + validateTargetOutput(targetOutput, snapshot, tolerance, frameIdx, targetIdx);
        end
    end
end

function errors = validateTargetOutput(targetOutput, snapshot, tolerance, frameIdx, targetIdx)
    errors = 0;
    requiredFields = {'ID', 'Type', 'State', 'IsHidden', 'RCS', 'Time', 'Kinematics'};

    for fieldIdx = 1:numel(requiredFields)
        if ~isfield(targetOutput, requiredFields{fieldIdx})
            fprintf('ERROR: Frame %d target %d missing field %s.\n', ...
                frameIdx, targetIdx, requiredFields{fieldIdx});
            errors = errors + 1;
        end
    end

    if ~isequal(size(targetOutput.Kinematics), [3, 2])
        fprintf('ERROR: Frame %d target %d Kinematics size is not 3x2.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end

    positionColumn = targetOutput.Kinematics(:, 1);
    velocityColumn = targetOutput.Kinematics(:, 2);

    if any(abs(positionColumn - snapshot.Position(:)) > tolerance)
        fprintf('ERROR: Frame %d target %d position column mismatch.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end

    if any(abs(velocityColumn - snapshot.Velocity(:)) > tolerance)
        fprintf('ERROR: Frame %d target %d velocity column mismatch.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end

    if isempty(targetOutput.RCS) || targetOutput.RCS <= 0
        fprintf('ERROR: Frame %d target %d has invalid RCS.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end

    if snapshot.IsHidden && ~targetOutput.IsHidden
        fprintf('ERROR: Frame %d target %d hidden flag not preserved.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end

    if targetOutput.State ~= snapshot.CurrentState
        fprintf('ERROR: Frame %d target %d state mismatch.\n', frameIdx, targetIdx);
        errors = errors + 1;
    end
end

function errors = validateTable(tableData)
    errors = 0;
    requiredColumns = {
        'Time'
        'ID'
        'Type'
        'State'
        'IsHidden'
        'X'
        'Y'
        'Z'
        'Vx'
        'Vy'
        'Vz'
        'RCS'
        'BehaviorMode'
        'DesiredHeading'
        'DesiredSpeed'
        'DesiredAltitude'
        'BehaviorReason'
        'MissionType'
        'MissionWaypointIndex'
        'MissionReason'
        'MissionStatus'
        'MissionProgress'
        'MissionDistanceToWaypoint'
        'MissionCompletionReason'
        'MissionCancelReason'
    };

    for columnIdx = 1:numel(requiredColumns)
        if ~ismember(requiredColumns{columnIdx}, tableData.Properties.VariableNames)
            fprintf('ERROR: Table missing column %s.\n', requiredColumns{columnIdx});
            errors = errors + 1;
        end
    end

    if height(tableData) == 0
        fprintf('ERROR: Table is empty.\n');
        errors = errors + 1;
    end
end

function errors = validateFileExport(outputs, projectRoot)
    errors = 0;
    tempDir = fullfile(projectRoot, 'temp_test_export');

    if ~isfolder(tempDir)
        mkdir(tempDir);
    end

    csvFile = fullfile(tempDir, 'radar_output_test.csv');
    matFile = fullfile(tempDir, 'radar_output_test.mat');

    try
        RadarOutputExporter.toCSV(outputs, csvFile);
        RadarOutputExporter.toMAT(outputs, matFile);

        if ~isfile(csvFile)
            fprintf('ERROR: CSV file was not created.\n');
            errors = errors + 1;
        end

        if ~isfile(matFile)
            fprintf('ERROR: MAT file was not created.\n');
            errors = errors + 1;
        end

        if isfile(csvFile)
            csvTable = readtable(csvFile);
            if height(csvTable) ~= height(RadarOutputExporter.toTable(outputs))
                fprintf('ERROR: CSV row count mismatch.\n');
                errors = errors + 1;
            end
        end

        if isfile(matFile)
            loaded = load(matFile);
            if ~isfield(loaded, 'radarOutputs')
                fprintf('ERROR: MAT file does not contain radarOutputs.\n');
                errors = errors + 1;
            end
        end
    catch exportError
        fprintf('ERROR: File export failed: %s\n', exportError.message);
        errors = errors + 1;
    end

    if isfile(csvFile)
        delete(csvFile);
    end
    if isfile(matFile)
        delete(matFile);
    end
    if isfolder(tempDir)
        rmdir(tempDir);
    end
end
