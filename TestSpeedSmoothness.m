% TestSpeedSmoothness  Проверка плавности изменения скорости (ТЗ №12).

function TestSpeedSmoothness()
    rng(42);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Speed Smoothness Validation ===\n\n');

    config = createTestConfig();
    result = SimulationEngine().run(config);
    errors = 0;
    tolerance = 1e-6;
    dt = config.Dt;

    limits = struct( ...
        'False', 2.5, ...
        'Ground', 1.2, ...
        'AirplaneUAV', 0.6, ...
        'Quadcopter', 4.0);

    for k = 1:numel(result.Targets)
        target = result.Targets{k};
        typeName = matlab.lang.makeValidName(char(target.Type));
        speeds = target.HistorySpeed;

        if numel(speeds) < 2
            continue;
        end

        speedSteps = abs(diff(speeds));
        maxStep = max(speedSteps);
        limit = limits.(typeName) * dt + tolerance;

        if maxStep > limit
            fprintf('ERROR: Target %d (%s) max speed step %.4f exceeds %.4f.\n', ...
                target.ID, char(target.Type), maxStep, limit);
            errors = errors + 1;
        else
            fprintf('Target %d (%s): max speed step %.4f <= %.4f OK\n', ...
                target.ID, char(target.Type), maxStep, limit);
        end
    end

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Speed Smoothness Validation PASSED\n');
    else
        fprintf('Speed Smoothness Validation FAILED\n');
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
    config.RandomSeed = 42;
end
