% TestMotionLogic  Валидация реализма движения целей (ТЗ №9).

function TestMotionLogic()
    rng(24);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== Motion Logic Validation ===\n\n');

    config = createTestConfig();
    result = SimulationEngine().run(config);
    metricsByType = collectMetrics(result);

    errors = 0;
    errors = errors + validateKinematicsExport(result);
    errors = errors + validateTypeBehavior(metricsByType, result.Config.Dt);

    printMetricsReport(result, metricsByType);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Motion Logic Validation PASSED\n');
    else
        fprintf('Motion Logic Validation FAILED\n');
    end
end

function config = createTestConfig()
    config.NumFalse = 3;
    config.NumGround = 3;
    config.NumAirplaneUAV = 3;
    config.NumQuadcopter = 3;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 180;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 24;
end

function metricsByType = collectMetrics(result)
    metricsByType = struct();

    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        metrics = MotionMetrics.compute(target, result.Config.Dt);
        typeName = matlab.lang.makeValidName(char(target.Type));

        if ~isfield(metricsByType, typeName)
            metricsByType.(typeName) = metrics;
        else
            metricsByType.(typeName) = [metricsByType.(typeName), metrics];
        end
    end
end

function errors = validateKinematicsExport(result)
    errors = 0;
    outputs = RadarOutputExporter.exportSimulation(result);

    for frameIdx = 1:numel(outputs)
        for targetIdx = 1:numel(outputs(frameIdx).Targets)
            targetOutput = outputs(frameIdx).Targets{targetIdx};

            if ~isequal(size(targetOutput.Kinematics), [3, 2])
                fprintf('ERROR: Kinematics size invalid for target %d.\n', targetOutput.ID);
                errors = errors + 1;
            end
        end
    end

    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        if numel(target.Position) ~= 3 || numel(target.Velocity) ~= 3
            fprintf('ERROR: Target %d does not have 3D position/velocity.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function errors = validateTypeBehavior(metricsByType, dt)
    errors = 0;

    birdMetrics = metricsByType.False;
    groundMetrics = metricsByType.Ground;
    airplaneMetrics = metricsByType.AirplaneUAV;
    quadMetrics = metricsByType.Quadcopter;

    errors = errors + validateBirdMetrics(birdMetrics);
    errors = errors + validateGroundMetrics(groundMetrics);
    errors = errors + validateAirplaneMetrics(airplaneMetrics, birdMetrics);
    errors = errors + validateQuadcopterMetrics(quadMetrics, airplaneMetrics);
    errors = errors + validateMotionSmoothness(metricsByType, dt);
end

function errors = validateBirdMetrics(metrics)
    errors = 0;

    for k = 1:numel(metrics)
        m = metrics(k);
        errors = errors + assertTrue(m.AltitudeRange > 0.5, ...
            sprintf('Bird %d has insufficient altitude variation.', m.ID));
        errors = errors + assertTrue(m.MeanTurnRate > deg2rad(2), ...
            sprintf('Bird %d turn rate too low.', m.ID));
        errors = errors + assertTrue(m.StraightSegmentLengthMean >= 10 && ...
            m.StraightSegmentLengthMean <= 80, ...
            sprintf('Bird %d straight segment length out of range.', m.ID));
        errors = errors + assertTrue(m.MaxAltitudeStep < 8, ...
            sprintf('Bird %d altitude changes in steps.', m.ID));
    end

    lowAltitudeHidden = sum([metrics.LowAltitudeHiddenCount]);
    hiddenTotal = sum([metrics.HiddenCount]);
    if hiddenTotal == 0 && lowAltitudeHidden == 0
        fprintf('WARNING: No Hidden states observed for birds in this run.\n');
    end
end

function errors = validateGroundMetrics(metrics)
    errors = 0;

    for k = 1:numel(metrics)
        m = metrics(k);
        errors = errors + assertTrue(m.AltitudeRange <= 6.5, ...
            sprintf('Ground %d altitude range too large.', m.ID));
        errors = errors + assertTrue(m.ForbiddenStateCount == 0, ...
            sprintf('Ground %d used forbidden states.', m.ID));
        errors = errors + assertTrue(m.MeanVerticalSpeed < 0.6, ...
            sprintf('Ground %d vertical speed too high.', m.ID));
        errors = errors + assertTrue(m.HorizontalDistance > 0.8 * m.TotalDistance, ...
            sprintf('Ground %d motion is not mainly horizontal.', m.ID));
    end
end

function errors = validateAirplaneMetrics(metrics, birdMetrics)
    errors = 0;
    birdTurnMean = mean([birdMetrics.MeanTurnRate]);
    birdSegmentMean = mean([birdMetrics.StraightSegmentLengthMean]);

    for k = 1:numel(metrics)
        m = metrics(k);
        errors = errors + assertTrue(m.HoverTime == 0, ...
            sprintf('Airplane %d has hover time.', m.ID));
        errors = errors + assertTrue(m.ForbiddenStateCount == 0, ...
            sprintf('Airplane %d used forbidden states.', m.ID));
        errors = errors + assertTrue(m.MeanTurnRate < deg2rad(8), ...
            sprintf('Airplane %d turns too frequently.', m.ID));
        errors = errors + assertTrue(m.MeanTurnRate < birdTurnMean, ...
            sprintf('Airplane %d turns more than bird.', m.ID));
        errors = errors + assertTrue(m.StraightSegmentLengthMean > birdSegmentMean, ...
            sprintf('Airplane %d straight segments are not longer than bird.', m.ID));
        errors = errors + assertTrue(m.AltitudeRange < 250, ...
            sprintf('Airplane %d altitude changes too aggressively.', m.ID));
    end
end

function errors = validateQuadcopterMetrics(metrics, airplaneMetrics)
    errors = 0;
    airplaneTurnMean = mean([airplaneMetrics.MeanTurnRate]);

    errors = errors + assertTrue(sum([metrics.HoverTime]) > 0, ...
        'No quadcopter hover episodes observed in simulation.');

    for k = 1:numel(metrics)
        m = metrics(k);
        errors = errors + assertTrue(m.AltitudeRange > 1.5, ...
            sprintf('Quadcopter %d altitude variation too small.', m.ID));
        errors = errors + assertTrue(m.MeanTurnRate > airplaneTurnMean, ...
            sprintf('Quadcopter %d is not more maneuverable than airplane.', m.ID));
        errors = errors + assertTrue(m.MaxSpeed <= 12.5, ...
            sprintf('Quadcopter %d exceeds speed profile.', m.ID));
    end
end

function printMetricsReport(result, metricsByType)
    fprintf('ID | Type         | Distance | AltRange | MeanSpeed | TurnRateMax | HiddenCount | Result\n');
    fprintf('---+--------------+----------+----------+-----------+-------------+-------------+-------\n');

    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        metrics = MotionMetrics.compute(target, result.Config.Dt);
        passed = evaluateTargetResult(metrics, metricsByType);
        fprintf('%2d | %-12s | %8.1f | %8.2f | %9.2f | %11.2f | %11d | %s\n', ...
            metrics.ID, char(metrics.Type), metrics.TotalDistance, metrics.AltitudeRange, ...
            metrics.MeanSpeed, metrics.MaxTurnRate, metrics.HiddenCount, passed);
    end
end

function resultText = evaluateTargetResult(metrics, metricsByType)
    switch char(metrics.Type)
        case char(TargetType.False)
            ok = metrics.AltitudeRange > 0.5 && metrics.MeanTurnRate > deg2rad(2);
        case char(TargetType.Ground)
            ok = metrics.AltitudeRange <= 6.5 && metrics.ForbiddenStateCount == 0;
        case char(TargetType.AirplaneUAV)
            ok = metrics.HoverTime == 0 && metrics.MeanTurnRate < deg2rad(8);
        case char(TargetType.Quadcopter)
            airplaneTurnMean = mean([metricsByType.AirplaneUAV.MeanTurnRate]);
            ok = metrics.AltitudeRange > 1.5 && metrics.MeanTurnRate > airplaneTurnMean;
        otherwise
            ok = false;
    end

    if ok
        resultText = 'OK';
    else
        resultText = 'FAIL';
    end
end

function errors = assertTrue(condition, message)
    errors = 0;
    if ~condition
        fprintf('ERROR: %s\n', message);
        errors = 1;
    end
end

function errors = validateMotionSmoothness(metricsByType, dt)
    errors = 0;
    typeNames = fieldnames(metricsByType);

    for k = 1:numel(typeNames)
        metrics = metricsByType.(typeNames{k});

        for metricIdx = 1:numel(metrics)
            m = metrics(metricIdx);
            profile = TargetProfileRegistry.getProfile(m.Type);
            maxAllowedUp = profile.MaxAcceleration * dt * 1.05 + 0.5;
            maxAllowedDown = profile.MaxDeceleration * dt * 1.05 + 0.5;
            maxAllowedStep = max(maxAllowedUp, maxAllowedDown);

            errors = errors + assertTrue(m.MaxSpeedStep <= maxAllowedStep, ...
                sprintf('Target %d speed jumps exceed acceleration limits.', m.ID));
            errors = errors + assertTrue(m.MaxAltitudeStep < 15, ...
                sprintf('Target %d altitude changes in large steps.', m.ID));
        end
    end
end
