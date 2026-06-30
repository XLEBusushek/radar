% TestSmallScenarioVisualLogic  Малый демонстрационный сценарий (ТЗ №12).

function TestSmallScenarioVisualLogic()
    rng(42);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Small Scenario Visual Logic ===\n\n');

    config = createSmallConfig();
    result = SimulationEngine().run(config);
    errors = 0;

    errors = errors + validateTargetCount(result);
    errors = errors + validatePerTypeBehavior(result, config.Dt);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Small Scenario Visual Logic PASSED\n');
    else
        fprintf('Small Scenario Visual Logic FAILED\n');
    end
end

function config = createSmallConfig()
    config.NumFalse = 1;
    config.NumGround = 1;
    config.NumAirplaneUAV = 1;
    config.NumQuadcopter = 1;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 300;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 42;
end

function errors = validateTargetCount(result)
    errors = 0;

    if numel(result.Targets) ~= 4
        fprintf('ERROR: Expected 4 targets, got %d.\n', numel(result.Targets));
        errors = errors + 1;
        return;
    end

    types = cellfun(@(t) char(t.Type), result.Targets, 'UniformOutput', false);
    required = {char(TargetType.False), char(TargetType.Ground), ...
        char(TargetType.AirplaneUAV), char(TargetType.Quadcopter)};

    for k = 1:numel(required)
        if ~any(strcmp(types, required{k}))
            fprintf('ERROR: Missing target type %s.\n', required{k});
            errors = errors + 1;
        end
    end
end

function errors = validatePerTypeBehavior(result, dt)
    errors = 0;
    tolerance = 1e-6;

    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        metrics = MotionMetrics.compute(target, dt);
        speeds = target.HistorySpeed;
        maxSpeedStep = 0;
        if numel(speeds) > 1
            maxSpeedStep = max(abs(diff(speeds)));
        end

        switch char(target.Type)
            case char(TargetType.Ground)
                profile = TargetProfileRegistry.getProfile(target.Type);
                if maxSpeedStep > profile.MaxDeceleration * dt + tolerance
                    fprintf('ERROR: Ground target %d speed not smooth.\n', target.ID);
                    errors = errors + 1;
                end
                if ~RoadNetwork.isOnRoad(target.Position, target.Heading, 15)
                    fprintf('ERROR: Ground target %d left road network.\n', target.ID);
                    errors = errors + 1;
                end
                if metrics.ForbiddenStateCount > 0
                    fprintf('ERROR: Ground target %d used forbidden states.\n', target.ID);
                    errors = errors + 1;
                end

            case char(TargetType.AirplaneUAV)
                profile = TargetProfileRegistry.getProfile(target.Type);
                if maxSpeedStep > profile.MaxDeceleration * dt + tolerance
                    fprintf('ERROR: Airplane %d speed not smooth.\n', target.ID);
                    errors = errors + 1;
                end
                if metrics.StraightSegmentLengthMean < 30
                    fprintf('ERROR: Airplane %d straight segments too short.\n', target.ID);
                    errors = errors + 1;
                end
                if metrics.HoverTime > 0
                    fprintf('ERROR: Airplane %d has hover time.\n', target.ID);
                    errors = errors + 1;
                end

            case char(TargetType.Quadcopter)
                if metrics.HoverTime <= 0
                    fprintf('ERROR: Quadcopter %d has no hover episodes.\n', target.ID);
                    errors = errors + 1;
                end

            case char(TargetType.False)
                if metrics.StraightSegmentLengthMean < 10 || metrics.StraightSegmentLengthMean > 80
                    fprintf('ERROR: Bird %d segment length out of range.\n', target.ID);
                    errors = errors + 1;
                end
                if metrics.AltitudeRange < 0.5
                    fprintf('ERROR: Bird %d has insufficient altitude variation.\n', target.ID);
                    errors = errors + 1;
                end
        end
    end
end
