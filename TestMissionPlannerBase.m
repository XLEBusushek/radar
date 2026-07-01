% TestMissionPlannerBase  Базовая валидация Mission Layer (ТЗ №13.1).

function TestMissionPlannerBase()
    rng(37);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Mission Planner Base Validation ===\n\n');

    dt = 1;
    environment = SimulationEnvironment.create( ...
        [2000, 2000, 400], 0, 400, 120, dt);
    targets = createMissionTargets(environment);
    errors = 0;

    [errorsInitial, targets] = validateInitialMissions(targets, environment, dt);
    errors = errors + errorsInitial;
    [errorsPersistence, targets] = validateMissionPersistence(targets, environment, dt);
    errors = errors + errorsPersistence;
    errors = errors + validateWaypointAdvance(targets, environment);
    errors = errors + validateReturnToAreaMission(environment, dt);
    errors = errors + validateBehaviorUsesMission(targets, environment, dt);
    errors = errors + validateExporterMissionFields();

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Mission Planner Base Validation PASSED\n');
    else
        fprintf('Mission Planner Base Validation FAILED\n');
    end
end

function targets = createMissionTargets(environment)
    typePlan = [
        TargetType.False
        TargetType.Ground
        TargetType.AirplaneUAV
        TargetType.Quadcopter
    ];

    targets = cell(numel(typePlan), 1);
    center = [mean(environment.XLimits), mean(environment.YLimits)];
    for k = 1:numel(typePlan)
        targets{k} = TargetFactory.createRandom(typePlan(k), environment);
        targets{k}.Position(1) = center(1) + (-100 + 200 * rand());
        targets{k}.Position(2) = center(2) + (-100 + 200 * rand());
    end
end

function [errors, targets] = validateInitialMissions(targets, environment, dt)
    errors = 0;
    expectedTypes = containers.Map( ...
        {char(TargetType.False), char(TargetType.Ground), char(TargetType.AirplaneUAV), char(TargetType.Quadcopter)}, ...
        {MissionType.MoveBetweenZones, MissionType.FollowRoadRoute, MissionType.PatrolRoute, MissionType.InspectArea});

    for k = 1:numel(targets)
        target = targets{k};
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        targets{k} = target;

        if ~MissionCommand.isActive(missionCommand)
            fprintf('ERROR: Target %d has no active MissionCommand.\n', target.ID);
            errors = errors + 1;
            continue;
        end

        if strlength(string(missionCommand.MissionType)) == 0
            fprintf('ERROR: Target %d has empty MissionType.\n', target.ID);
            errors = errors + 1;
        end

        expectedType = expectedTypes(char(target.Type));
        if missionCommand.MissionType ~= expectedType
            fprintf('ERROR: Target %d expected mission %s, got %s.\n', ...
                target.ID, char(expectedType), char(missionCommand.MissionType));
            errors = errors + 1;
        end

        if size(missionCommand.MissionRoute, 1) < 2
            fprintf('ERROR: Target %d MissionRoute has fewer than 2 points.\n', target.ID);
            errors = errors + 1;
        end

        if ~isequal(size(missionCommand.CurrentWaypoint), [1, 3])
            fprintf('ERROR: Target %d CurrentWaypoint size is not 1x3.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function [errors, targets] = validateMissionPersistence(targets, environment, dt)
    errors = 0;

    for k = 1:numel(targets)
        target = targets{k};
        [target, firstCommand] = MissionPlanner.plan(target, environment, dt);
        firstType = firstCommand.MissionType;
        firstReason = string(firstCommand.MissionReason);
        resets = 0;

        for step = 1:8
            [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
            if missionCommand.MissionType ~= firstType || ...
                    string(missionCommand.MissionReason) ~= firstReason
                resets = resets + 1;
            end
        end

        if resets > 0
            fprintf('ERROR: Target %d mission recreated %d times within hold window.\n', ...
                target.ID, resets);
            errors = errors + 1;
        end

        if target.MissionTime < 8
            fprintf('ERROR: Target %d MissionTime did not advance.\n', target.ID);
            errors = errors + 1;
        end

        targets{k} = target;
    end
end

function errors = validateWaypointAdvance(~, environment)
    errors = 0;
    threshold = 20;
    typePlan = [TargetType.False, TargetType.Ground, TargetType.AirplaneUAV, TargetType.Quadcopter];

    for k = 1:numel(typePlan)
        target = TargetFactory.createRandom(typePlan(k), environment);
        center = [mean(environment.XLimits), mean(environment.YLimits)];
        target.Position(1:2) = center + [-120, 80];
        [target, missionCommand] = MissionPlanner.plan(target, environment, 1);

        if size(missionCommand.MissionRoute, 1) < 2
            fprintf('ERROR: Target %d route too short for waypoint advance test.\n', target.ID);
            errors = errors + 1;
            continue;
        end

        initialIndex = missionCommand.CurrentWaypointIndex;
        target.Position = missionCommand.MissionRoute(initialIndex, :);
        missionCommand = MissionCommand.advanceWaypoint( ...
            missionCommand, target.Position, threshold);

        if missionCommand.CurrentWaypointIndex <= initialIndex && ~missionCommand.IsMissionComplete
            fprintf('ERROR: Target %d waypoint index did not advance.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function errors = validateReturnToAreaMission(environment, dt)
    errors = 0;
    target = TargetFactory.createRandom(TargetType.AirplaneUAV, environment);
    margin = 0.08 * diff(environment.XLimits);
    target.Position(1) = environment.XLimits(1) + margin;

    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    target.MissionCommand.Status = MissionStatus.Completed;
    target.MissionTime = missionCommand.MissionHoldTime;

    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);

    if missionCommand.MissionType ~= MissionType.ReturnToArea
        fprintf('ERROR: Boundary target did not receive ReturnToArea mission.\n');
        errors = errors + 1;
    end
end

function errors = validateBehaviorUsesMission(~, environment, dt)
    errors = 0;
    typePlan = [TargetType.False, TargetType.Ground, TargetType.AirplaneUAV, TargetType.Quadcopter];

    for k = 1:numel(typePlan)
        target = TargetFactory.createRandom(typePlan(k), environment);
        center = [mean(environment.XLimits), mean(environment.YLimits)];
        target.Position(1:2) = center + [60, -40];
        for step = 1:4
            [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
            if missionCommand.Status == MissionStatus.Executing
                break;
            end
        end

        if missionCommand.Status ~= MissionStatus.Executing
            fprintf('ERROR: Target %d mission did not reach Executing status.\n', target.ID);
            errors = errors + 1;
            continue;
        end

        [~, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

        if ~all(isfinite(behaviorCommand.DesiredPosition))
            fprintf('ERROR: Target %d behavior command has invalid desired position.\n', target.ID);
            errors = errors + 1;
            continue;
        end

        if norm(behaviorCommand.DesiredPosition - missionCommand.CurrentWaypoint) > 1e-6
            fprintf('ERROR: Target %d behavior waypoint does not match mission waypoint.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function errors = validateExporterMissionFields()
    errors = 0;

    config.NumFalse = 1;
    config.NumGround = 1;
    config.NumAirplaneUAV = 1;
    config.NumQuadcopter = 1;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 10;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 17;

    result = SimulationEngine().run(config);
    outputs = RadarOutputExporter.exportSimulation(result);
    tableData = RadarOutputExporter.toTable(outputs);

    requiredColumns = {'MissionType', 'MissionWaypointIndex', 'MissionReason'};
    for columnIdx = 1:numel(requiredColumns)
        if ~ismember(requiredColumns{columnIdx}, tableData.Properties.VariableNames)
            fprintf('ERROR: Export table missing column %s.\n', requiredColumns{columnIdx});
            errors = errors + 1;
        end
    end

    sample = outputs(1).Targets{1};
    requiredFields = {'MissionType', 'MissionWaypointIndex', 'MissionReason'};
    for fieldIdx = 1:numel(requiredFields)
        if ~isfield(sample, requiredFields{fieldIdx})
            fprintf('ERROR: Export output missing field %s.\n', requiredFields{fieldIdx});
            errors = errors + 1;
        end
    end

    if any(strlength(string(tableData.MissionType)) == 0)
        fprintf('ERROR: Export table contains empty MissionType values.\n');
        errors = errors + 1;
    end
end
