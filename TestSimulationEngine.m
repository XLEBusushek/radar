% TestSimulationEngine  Валидация SimulationEngine (ТЗ №4).

function TestSimulationEngine()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    config = createTestConfig();
    errors = 0;

    fprintf('=== Simulation Engine Validation ===\n\n');

    engine = SimulationEngine();
    result = engine.run(config);

    expectedTargetCount = config.NumFalse + config.NumGround + ...
        config.NumAirplaneUAV + config.NumQuadcopter;

    errors = errors + assertEqual(numel(result.Targets), expectedTargetCount, ...
        'Target count mismatch.');

    errors = errors + validateOutputFrames(result, config, expectedTargetCount);
    errors = errors + validateSnapshots(result, config);
    errors = errors + validateStatistics(result, config);
    errors = errors + validateReproducibility(config, engine);

    PlotSimulationResult(result);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Simulation Engine Validation PASSED\n');
    else
        fprintf('Simulation Engine Validation FAILED\n');
    end
end

function config = createTestConfig()
    config.NumFalse = 5;
    config.NumGround = 5;
    config.NumAirplaneUAV = 3;
    config.NumQuadcopter = 3;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 300;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 42;
end

function errors = validateOutputFrames(result, config, expectedTargetCount)
    errors = 0;
    frames = result.OutputFrames;
    expectedTimes = 0:config.OutputPeriod:config.Duration;

    if numel(frames) ~= numel(expectedTimes)
        fprintf('ERROR: Expected %d output frames, got %d.\n', ...
            numel(expectedTimes), numel(frames));
        errors = errors + 1;
        return;
    end

    for k = 1:numel(expectedTimes)
        if abs(frames(k).Time - expectedTimes(k)) > 1e-9
            fprintf('ERROR: Frame %d time %.4f != expected %.4f.\n', ...
                k, frames(k).Time, expectedTimes(k));
            errors = errors + 1;
        end

        if k > 1
            delta = frames(k).Time - frames(k - 1).Time;
            if abs(delta - config.OutputPeriod) > 1e-9
                fprintf('ERROR: Frame interval %.4f != OutputPeriod.\n', delta);
                errors = errors + 1;
            end
        end

        if numel(frames(k).Targets) ~= expectedTargetCount
            fprintf('ERROR: Frame %d contains %d targets instead of %d.\n', ...
                k, numel(frames(k).Targets), expectedTargetCount);
            errors = errors + 1;
        end
    end
end

function errors = validateSnapshots(result, config)
    errors = 0;
    environment = result.Statistics.Environment;
    tolerance = 1e-6;
    requiredFields = {'ID', 'Type', 'Position', 'Velocity', 'RCS', 'CurrentState', 'IsHidden', 'Time'};

    for frameIdx = 1:numel(result.OutputFrames)
        frame = result.OutputFrames(frameIdx);

        for targetIdx = 1:numel(frame.Targets)
            snapshot = frame.Targets{targetIdx};

            for fieldIdx = 1:numel(requiredFields)
                if ~isfield(snapshot, requiredFields{fieldIdx})
                    fprintf('ERROR: Frame %d target %d missing field %s.\n', ...
                        frameIdx, targetIdx, requiredFields{fieldIdx});
                    errors = errors + 1;
                end
            end

            if any(isnan(snapshot.Position)) || any(isinf(snapshot.Position))
                fprintf('ERROR: Invalid Position in frame %d target %d.\n', frameIdx, targetIdx);
                errors = errors + 1;
            end

            if any(isnan(snapshot.Velocity)) || any(isinf(snapshot.Velocity))
                fprintf('ERROR: Invalid Velocity in frame %d target %d.\n', frameIdx, targetIdx);
                errors = errors + 1;
            end

            if snapshot.Position(1) < environment.XLimits(1) - tolerance || ...
                    snapshot.Position(1) > environment.XLimits(2) + tolerance
                fprintf('ERROR: Target %d X out of bounds in frame %d.\n', snapshot.ID, frameIdx);
                errors = errors + 1;
            end

            if snapshot.Position(2) < environment.YLimits(1) - tolerance || ...
                    snapshot.Position(2) > environment.YLimits(2) + tolerance
                fprintf('ERROR: Target %d Y out of bounds in frame %d.\n', snapshot.ID, frameIdx);
                errors = errors + 1;
            end

            if snapshot.Position(3) < environment.ZLimits(1) - tolerance || ...
                    snapshot.Position(3) > environment.ZLimits(2) + tolerance
                fprintf('ERROR: Target %d Z out of bounds in frame %d.\n', snapshot.ID, frameIdx);
                errors = errors + 1;
            end
        end
    end
end

function errors = validateStatistics(result, config)
    errors = 0;

    if ~isfield(result.Statistics, 'ExecutionTime') || result.Statistics.ExecutionTime <= 0
        fprintf('ERROR: Invalid execution time statistic.\n');
        errors = errors + 1;
    end

    falseCount = result.Statistics.CountByType.False;
    groundCount = result.Statistics.CountByType.Ground;
    airplaneCount = result.Statistics.CountByType.AirplaneUAV;
    quadcopterCount = result.Statistics.CountByType.Quadcopter;

    if falseCount ~= config.NumFalse || groundCount ~= config.NumGround || ...
            airplaneCount ~= config.NumAirplaneUAV || quadcopterCount ~= config.NumQuadcopter
        fprintf('ERROR: Statistics count by type mismatch.\n');
        errors = errors + 1;
    end
end

function errors = validateReproducibility(config, engine)
    errors = 0;

    resultA = engine.run(config);
    resultB = engine.run(config);

    if ~isequal(extractComparableResult(resultA), extractComparableResult(resultB))
        fprintf('ERROR: Repeated run with same RandomSeed produced different results.\n');
        errors = errors + 1;
    end
end

function comparable = extractComparableResult(result)
    comparable = struct();
    comparable.OutputFrames = result.OutputFrames;
    comparable.Statistics = result.Statistics;
    comparable.Statistics = rmfield(comparable.Statistics, 'ExecutionTime');

    targetData = cell(numel(result.Targets), 1);
    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        targetData{k} = struct( ...
            'ID', target.ID, ...
            'Type', target.Type, ...
            'HistoryPosition', target.HistoryPosition, ...
            'HistoryVelocity', target.HistoryVelocity, ...
            'HistorySpeed', target.HistorySpeed, ...
            'RCS', target.RCS);
    end
    comparable.Targets = targetData;
end

function errors = assertEqual(actual, expected, message)
    errors = 0;
    if actual ~= expected
        fprintf('ERROR: %s (expected %d, got %d)\n', message, expected, actual);
        errors = 1;
    end
end
