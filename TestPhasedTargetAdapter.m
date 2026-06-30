% TestPhasedTargetAdapter  Валидация PhasedTargetAdapter (ТЗ №5).

function TestPhasedTargetAdapter()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Phased Target Adapter Validation ===\n\n');

    if ~PhasedTargetAdapter.isToolboxAvailable()
        fprintf('Phased Array System Toolbox is not available. Skipping PhasedTargetAdapter test.\n');
        return;
    end

    config = createTestConfig();
    engine = SimulationEngine();
    result = engine.run(config);

    operatingFrequency = 10e9;
    adapter = PhasedTargetAdapter(result.Targets, operatingFrequency);
    adapter = adapter.initialize();

    expectedTargetCount = numel(result.Targets);
    errors = 0;

    errors = errors + assertEqual(numel(adapter.Platforms), expectedTargetCount, ...
        'Platform count mismatch.');
    errors = errors + assertEqual(numel(adapter.RadarTargets), expectedTargetCount, ...
        'RadarTarget count mismatch.');

    for frameIdx = 1:numel(result.OutputFrames)
        frame = result.OutputFrames(frameIdx);
        adapter = adapter.updateFromFrame(frame);

        errors = errors + validateAdapterState(adapter, frame, expectedTargetCount);
    end

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Phased Target Adapter Validation PASSED\n');
    else
        fprintf('Phased Target Adapter Validation FAILED\n');
    end
end

function config = createTestConfig()
    config.NumFalse = 3;
    config.NumGround = 3;
    config.NumAirplaneUAV = 2;
    config.NumQuadcopter = 2;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 60;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 42;
end

function errors = validateAdapterState(adapter, frame, expectedTargetCount)
    errors = 0;
    tolerance = 1e-9;

    [positions, velocities, rcsValues] = adapter.getTargetStates();

    if ~isequal(size(positions), [3, expectedTargetCount])
        fprintf('ERROR: Positions size [%d %d], expected [3 %d].\n', ...
            size(positions, 1), size(positions, 2), expectedTargetCount);
        errors = errors + 1;
    end

    if ~isequal(size(velocities), [3, expectedTargetCount])
        fprintf('ERROR: Velocities size [%d %d], expected [3 %d].\n', ...
            size(velocities, 1), size(velocities, 2), expectedTargetCount);
        errors = errors + 1;
    end

    if numel(rcsValues) ~= expectedTargetCount
        fprintf('ERROR: RCS vector length %d, expected %d.\n', ...
            numel(rcsValues), expectedTargetCount);
        errors = errors + 1;
    end

    for targetIdx = 1:expectedTargetCount
        snapshot = frame.Targets{targetIdx};

        if any(abs(positions(:, targetIdx) - snapshot.Position(:)) > tolerance)
            fprintf('ERROR: Position mismatch for target %d.\n', snapshot.ID);
            errors = errors + 1;
        end

        if any(abs(velocities(:, targetIdx) - snapshot.Velocity(:)) > tolerance)
            fprintf('ERROR: Velocity mismatch for target %d.\n', snapshot.ID);
            errors = errors + 1;
        end

        if abs(rcsValues(targetIdx) - snapshot.RCS) > tolerance
            fprintf('ERROR: RCS mismatch for target %d.\n', snapshot.ID);
            errors = errors + 1;
        end

        if snapshot.IsHidden && ~adapter.IsHidden(targetIdx)
            fprintf('ERROR: Hidden flag not set for target %d.\n', snapshot.ID);
            errors = errors + 1;
        end

        if ~snapshot.IsHidden && adapter.IsHidden(targetIdx)
            fprintf('ERROR: Hidden flag incorrectly set for target %d.\n', snapshot.ID);
            errors = errors + 1;
        end
    end
end

function errors = assertEqual(actual, expected, message)
    errors = 0;
    if actual ~= expected
        fprintf('ERROR: %s (expected %d, got %d)\n', message, expected, actual);
        errors = 1;
    end
end
