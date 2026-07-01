% TestNaturalMotionBase  Базовая валидация Natural Motion Layer (ТЗ №14.1).

function TestNaturalMotionBase()
    rng(91);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Natural Motion Base Validation ===\n\n');

    dt = 1;
    environment = SimulationEnvironment.create([2000, 2000, 400], 0, 400, 120, dt);
    errors = 0;

    errors = errors + validateMissionIntegrity(environment, dt);
    errors = errors + validateIdentityIntegrity(environment, dt);
    errors = errors + validateSmoothNoise(environment, dt);
    errors = errors + validateNoiseLimits(environment, dt);
    errors = errors + validateCommandBounds(environment, dt);
    errors = errors + validateReproducibility(environment, dt);
    errors = errors + validateExporterFields(environment, dt);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Natural Motion Base Validation PASSED\n');
    else
        fprintf('Natural Motion Base Validation FAILED\n');
    end
end

function errors = validateMissionIntegrity(environment, dt)
    errors = 0;
    target = createTestTarget(environment);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

    missionBefore = captureMissionState(target);
    [target, ~] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);
    missionAfter = captureMissionState(target);

    if ~isequal(missionBefore.MissionCommand, missionAfter.MissionCommand)
        fprintf('ERROR: NaturalMotionLayer changed MissionCommand.\n');
        errors = errors + 1;
    end

    if ~isequal(missionBefore.MissionRoute, missionAfter.MissionRoute)
        fprintf('ERROR: NaturalMotionLayer changed MissionRoute.\n');
        errors = errors + 1;
    end

    if missionBefore.MissionType ~= missionAfter.MissionType
        fprintf('ERROR: NaturalMotionLayer changed MissionType.\n');
        errors = errors + 1;
    end

    if missionBefore.MissionWaypointIndex ~= missionAfter.MissionWaypointIndex
        fprintf('ERROR: NaturalMotionLayer changed MissionWaypointIndex.\n');
        errors = errors + 1;
    end
end

function errors = validateIdentityIntegrity(environment, dt)
    errors = 0;
    target = createTestTarget(environment);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

    idBefore = target.ID;
    typeBefore = target.Type;
    rcsBefore = target.RCS;

    [target, ~] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);

    if target.ID ~= idBefore
        fprintf('ERROR: NaturalMotionLayer changed target ID.\n');
        errors = errors + 1;
    end

    if target.Type ~= typeBefore
        fprintf('ERROR: NaturalMotionLayer changed target Type.\n');
        errors = errors + 1;
    end

    if target.RCS ~= rcsBefore
        fprintf('ERROR: NaturalMotionLayer changed target RCS.\n');
        errors = errors + 1;
    end
end

function errors = validateSmoothNoise(environment, dt)
    errors = 0;
    target = createTestTarget(environment);
    command = createActiveCommand(target, environment);
    motionProfile = NaturalMotionProfileRegistry.getProfile(target.Type);
    headingHistory = zeros(60, 1);

    for step = 1:60
        [target, command] = NaturalMotionLayer.apply(target, command, environment, dt);
        headingHistory(step) = target.NaturalMotionState.HeadingNoise;
    end

    headingSteps = abs(diff(headingHistory));
    if max(headingSteps) >= motionProfile.MaxHeadingNoise
        fprintf('ERROR: Heading noise step %.4f exceeds profile max %.4f.\n', ...
            max(headingSteps), motionProfile.MaxHeadingNoise);
        errors = errors + 1;
    end

    whiteNoiseSteps = motionProfile.MaxHeadingNoise * abs(2 * rand(59, 1) - 1);
    if mean(headingSteps) >= mean(whiteNoiseSteps)
        fprintf('ERROR: Heading noise is not smoother than independent jumps.\n');
        errors = errors + 1;
    end
end

function errors = validateNoiseLimits(environment, dt)
    errors = 0;
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.False);
    target = createTestTarget(environment);
    command = createActiveCommand(target, environment);

    for step = 1:80
        [target, command] = NaturalMotionLayer.apply(target, command, environment, dt);
        state = target.NaturalMotionState;

        if abs(state.HeadingNoise) > motionProfile.MaxHeadingNoise + 1e-9
            fprintf('ERROR: HeadingNoise %.4f exceeds max %.4f.\n', ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            errors = errors + 1;
        end

        if abs(state.SpeedNoise) > motionProfile.MaxSpeedNoise + 1e-9
            fprintf('ERROR: SpeedNoise %.4f exceeds max %.4f.\n', ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            errors = errors + 1;
        end

        if abs(state.AltitudeNoise) > motionProfile.MaxAltitudeNoise + 1e-9
            fprintf('ERROR: AltitudeNoise %.4f exceeds max %.4f.\n', ...
                state.AltitudeNoise, motionProfile.MaxAltitudeNoise);
            errors = errors + 1;
        end

        if norm(state.PositionNoise(1:2)) > motionProfile.MaxPositionNoise + 1e-9
            fprintf('ERROR: PositionNoise norm %.4f exceeds max %.4f.\n', ...
                norm(state.PositionNoise(1:2)), motionProfile.MaxPositionNoise);
            errors = errors + 1;
        end
    end
end

function errors = validateCommandBounds(environment, dt)
    errors = 0;
    target = createTestTarget(environment);
    profile = TargetProfileRegistry.getProfile(target.Type);
    command = createActiveCommand(target, environment);
    altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

    for step = 1:50
        [target, command] = NaturalMotionLayer.apply(target, command, environment, dt);

        if command.DesiredSpeed < profile.SpeedMin - 1e-9 || ...
                command.DesiredSpeed > profile.SpeedMax + 1e-9
            fprintf('ERROR: DesiredSpeed %.4f outside [%.4f, %.4f].\n', ...
                command.DesiredSpeed, profile.SpeedMin, profile.SpeedMax);
            errors = errors + 1;
        end

        if command.DesiredAltitude < altitudeLimits(1) - 1e-9 || ...
                command.DesiredAltitude > altitudeLimits(2) + 1e-9
            fprintf('ERROR: DesiredAltitude %.4f outside environment limits.\n', ...
                command.DesiredAltitude);
            errors = errors + 1;
        end

        if command.DesiredPosition(1) < environment.XLimits(1) - 1e-9 || ...
                command.DesiredPosition(1) > environment.XLimits(2) + 1e-9 || ...
                command.DesiredPosition(2) < environment.YLimits(1) - 1e-9 || ...
                command.DesiredPosition(2) > environment.YLimits(2) + 1e-9
            fprintf('ERROR: DesiredPosition XY is outside Environment bounds.\n');
            errors = errors + 1;
        end

        if command.DesiredHeading < -pi - 1e-9 || command.DesiredHeading > pi + 1e-9
            fprintf('ERROR: DesiredHeading %.4f is not normalized.\n', command.DesiredHeading);
            errors = errors + 1;
        end
    end
end

function errors = validateReproducibility(environment, dt)
    errors = 0;
    historyA = runNoiseHistory(environment, dt, 91);
    historyB = runNoiseHistory(environment, dt, 91);

    if ~isequal(historyA, historyB)
        fprintf('ERROR: Natural motion is not reproducible for the same RandomSeed.\n');
        errors = errors + 1;
    end
end

function errors = validateExporterFields(environment, dt)
    errors = 0;

    config.NumFalse = 1;
    config.NumGround = 0;
    config.NumAirplaneUAV = 0;
    config.NumQuadcopter = 0;
    config.BoxSize = environment.BoxSize;
    config.Duration = 5;
    config.Dt = dt;
    config.OutputPeriod = 5;
    config.RandomSeed = 91;

    result = SimulationEngine().run(config);
    outputs = RadarOutputExporter.exportSimulation(result);
    tableData = RadarOutputExporter.toTable(outputs);

    requiredColumns = {'HeadingNoise', 'SpeedNoise', 'AltitudeNoise', 'PositionNoiseNorm'};
    for colIdx = 1:numel(requiredColumns)
        if ~ismember(requiredColumns{colIdx}, tableData.Properties.VariableNames)
            fprintf('ERROR: CSV table missing column %s.\n', requiredColumns{colIdx});
            errors = errors + 1;
        end
    end
end

function history = runNoiseHistory(environment, dt, seed)
    rng(seed);
    RadarTargetModel.resetIdCounter();
    target = createTestTarget(environment);
    command = createActiveCommand(target, environment);
    history = zeros(40, 4);

    for step = 1:40
        [target, command] = NaturalMotionLayer.apply(target, command, environment, dt);
        state = target.NaturalMotionState;
        history(step, :) = [state.HeadingNoise, state.SpeedNoise, state.AltitudeNoise, ...
            norm(state.PositionNoise(1:2))];
    end
end

function state = captureMissionState(target)
    state = struct( ...
        'MissionCommand', target.MissionCommand, ...
        'MissionRoute', target.MissionRoute, ...
        'MissionType', target.MissionType, ...
        'MissionWaypointIndex', target.MissionWaypointIndex);
end

function target = createTestTarget(environment)
    target = TargetFactory.createRandom(TargetType.False, environment);
    center = [mean(environment.XLimits), mean(environment.YLimits), 25];
    target.Position = center;
end

function command = createActiveCommand(target, environment)
    command = BehaviorCommand.create( ...
        'BehaviorMode', BehaviorMode.TurnToWaypoint, ...
        'DesiredPosition', target.Position + [40, 25, 0], ...
        'DesiredHeading', target.Heading, ...
        'DesiredSpeed', target.Speed, ...
        'DesiredAltitude', target.Position(3) + 5, ...
        'HoldTime', 20, ...
        'Priority', 1, ...
        'Reason', 'Natural motion test command');
    command.DesiredPosition(3) = target.Position(3) + 5;
    command.DesiredPosition(1) = min(max(command.DesiredPosition(1), environment.XLimits(1)), ...
        environment.XLimits(2));
    command.DesiredPosition(2) = min(max(command.DesiredPosition(2), environment.YLimits(1)), ...
        environment.YLimits(2));
end
