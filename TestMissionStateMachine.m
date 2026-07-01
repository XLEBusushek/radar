% TestMissionStateMachine  Валидация жизненного цикла миссии (ТЗ №13.2).

function TestMissionStateMachine()
    rng(43);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Mission State Machine Validation ===\n\n');

    dt = 1;
    environment = SimulationEnvironment.create( ...
        [2000, 2000, 400], 0, 400, 120, dt);
    errors = 0;

    errors = errors + validateCreatedStatus(environment);
    errors = errors + validatePlanningTransition(environment, dt);
    errors = errors + validateExecutingTransition(environment, dt);
    errors = errors + validateWaypointProgress(environment, dt);
    errors = errors + validateCompletion(environment, dt);
    errors = errors + validateCancellation(environment, dt);
    errors = errors + validateHiddenPause(environment, dt);
    errors = errors + validateMissionPersistence(environment, dt);
    errors = errors + validateExporterFields();

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Mission State Machine Validation PASSED\n');
    else
        fprintf('Mission State Machine Validation FAILED\n');
    end
end

function target = createCenterTarget(targetType, environment)
    target = TargetFactory.createRandom(targetType, environment);
    center = [mean(environment.XLimits), mean(environment.YLimits)];
    target.Position(1) = center(1) + 40;
    target.Position(2) = center(2) - 30;
end

function errors = validateCreatedStatus(environment)
    errors = 0;
    target = createCenterTarget(TargetType.False, environment);
    missionCommand = BirdMissionPlanner.createMission(target, environment);

    if missionCommand.Status ~= MissionStatus.Created
        fprintf('ERROR: New mission status is not Created.\n');
        errors = errors + 1;
    end
end

function errors = validatePlanningTransition(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.False, environment);
    missionCommand = BirdMissionPlanner.createMission(target, environment);
    target = target.setMissionCommand(missionCommand);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);

    if missionCommand.Status ~= MissionStatus.Planning
        fprintf('ERROR: Mission did not transition to Planning.\n');
        errors = errors + 1;
    end
end

function errors = validateExecutingTransition(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.Ground, environment);
    missionCommand = GroundMissionPlanner.createMission(target, environment);
    target = target.setMissionCommand(missionCommand);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);

    if missionCommand.Status ~= MissionStatus.Executing
        fprintf('ERROR: Mission did not transition to Executing.\n');
        errors = errors + 1;
    end
end

function errors = validateWaypointProgress(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.AirplaneUAV, environment);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);

    initialIndex = missionCommand.CurrentWaypointIndex;
    initialProgress = missionCommand.Progress;
    target.Position = missionCommand.CurrentWaypoint;
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);

    if missionCommand.CurrentWaypointIndex <= initialIndex && ~missionCommand.IsMissionComplete
        fprintf('ERROR: Waypoint index did not increase after reaching waypoint.\n');
        errors = errors + 1;
    end

    if missionCommand.Progress < initialProgress
        fprintf('ERROR: Mission progress decreased.\n');
        errors = errors + 1;
    end
end

function errors = validateCompletion(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.Quadcopter, environment);
    missionCommand = QuadcopterMissionPlanner.createMission(target, environment);
    target = target.setMissionCommand(missionCommand);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);

    for step = 1:500
        if missionCommand.Status == MissionStatus.Completed
            break;
        end

        if missionCommand.InspectionPhase == InspectionPhase.MoveToPoint
            target.Position = missionCommand.CurrentWaypoint;
        elseif missionCommand.InspectionPhase == InspectionPhase.AltitudeAdjust
            target.Position(3) = missionCommand.InspectionPhaseAltitude;
        end

        [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);
    end

    if missionCommand.Status ~= MissionStatus.Completed
        fprintf('ERROR: Mission did not become Completed after final waypoint.\n');
        errors = errors + 1;
    end

    if missionCommand.Progress < 1
        fprintf('ERROR: Completed mission progress is not 1.\n');
        errors = errors + 1;
    end
end

function errors = validateCancellation(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.False, environment);
    missionCommand = MissionCommand.create( ...
        'MissionType', MissionType.MoveBetweenZones, ...
        'MissionRoute', target.Position, ...
        'DesiredMissionSpeed', target.Speed, ...
        'DesiredMissionAltitude', target.Position(3), ...
        'MissionHoldTime', 10, ...
        'MissionReason', 'Invalid route test');
    target = target.setMissionCommand(missionCommand);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);
    [target, missionCommand] = MissionStateMachine.update(target, missionCommand, environment, dt);

    if missionCommand.Status ~= MissionStatus.Cancelled
        fprintf('ERROR: Invalid mission was not Cancelled.\n');
        errors = errors + 1;
    end
end

function errors = validateHiddenPause(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.False, environment);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);

    if missionCommand.Status ~= MissionStatus.Executing
        fprintf('ERROR: Hidden pause test could not reach Executing.\n');
        errors = errors + 1;
        return;
    end

    target.IsHidden = true;
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    if missionCommand.Status ~= MissionStatus.Paused
        fprintf('ERROR: Hidden target mission did not pause.\n');
        errors = errors + 1;
    end

    target.IsHidden = false;
    [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
    if missionCommand.Status ~= MissionStatus.Executing
        fprintf('ERROR: Mission did not resume Executing after hidden state ended.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionPersistence(environment, dt)
    errors = 0;
    target = createCenterTarget(TargetType.Ground, environment);
    [target, firstCommand] = MissionPlanner.plan(target, environment, dt);
    firstType = firstCommand.MissionType;
    firstReason = string(firstCommand.MissionReason);
    resets = 0;

    for step = 1:10
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        if missionCommand.MissionType ~= firstType || ...
                string(missionCommand.MissionReason) ~= firstReason
            resets = resets + 1;
        end
    end

    if resets > 0
        fprintf('ERROR: Mission recreated %d times within hold window.\n', resets);
        errors = errors + 1;
    end
end

function errors = validateExporterFields()
    errors = 0;

    config.NumFalse = 1;
    config.NumGround = 1;
    config.NumAirplaneUAV = 1;
    config.NumQuadcopter = 1;
    config.BoxSize = [1000, 1000, 300];
    config.Duration = 10;
    config.Dt = 1;
    config.OutputPeriod = 5;
    config.RandomSeed = 23;

    result = SimulationEngine().run(config);
    outputs = RadarOutputExporter.exportSimulation(result);
    tableData = RadarOutputExporter.toTable(outputs);

    requiredColumns = {
        'MissionStatus'
        'MissionProgress'
        'MissionDistanceToWaypoint'
        'MissionCompletionReason'
        'MissionCancelReason'
    };

    for columnIdx = 1:numel(requiredColumns)
        if ~ismember(requiredColumns{columnIdx}, tableData.Properties.VariableNames)
            fprintf('ERROR: Export table missing column %s.\n', requiredColumns{columnIdx});
            errors = errors + 1;
        end
    end

    sample = outputs(1).Targets{1};
    for columnIdx = 1:numel(requiredColumns)
        if ~isfield(sample, requiredColumns{columnIdx})
            fprintf('ERROR: Export output missing field %s.\n', requiredColumns{columnIdx});
            errors = errors + 1;
        end
    end
end
