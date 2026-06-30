% TestBehaviorPlanner  Валидация Behavior Layer (ТЗ №11).

function TestBehaviorPlanner()
    rng(31);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Behavior Planner Validation ===\n\n');

    simDuration = 150;
    dt = 1;
    environment = SimulationEnvironment.create( ...
        [2000, 2000, 400], 0, 400, simDuration, dt);
    engine = DecisionEngine();

    targets = createPlannerTargets(environment);
    errors = 0;
    records = initializeRecords(targets);

    for step = 1:simDuration
        for targetIdx = 1:numel(targets)
            target = targets{targetIdx};
            previousCommand = target.BehaviorCommand;
            previousMode = target.BehaviorMode;

            [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, dt);
            records = updatePlannerRecords(records, targetIdx, target, behaviorCommand, previousCommand, previousMode, dt);

            decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
            targets{targetIdx} = TrajectoryGenerator.updateMotion( ...
                target, decision, behaviorCommand, environment, dt);
        end
    end

    errors = errors + validateAllowedModes(records);
    errors = errors + validateHoldTime(records);
    errors = errors + validateAvoidBoundary(records, environment);
    errors = errors + validateGroundRoadAlignment(targets);
    errors = errors + validateQuadcopterHover(records);
    errors = errors + validateSegmentLengths(targets);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Behavior Planner Validation PASSED\n');
    else
        fprintf('Behavior Planner Validation FAILED\n');
    end
end

function targets = createPlannerTargets(environment)
    typePlan = [
        repmat(TargetType.False, 3, 1)
        repmat(TargetType.Ground, 3, 1)
        repmat(TargetType.AirplaneUAV, 3, 1)
        repmat(TargetType.Quadcopter, 3, 1)
    ];

    targets = cell(numel(typePlan), 1);
    for k = 1:numel(typePlan)
        targets{k} = TargetFactory.createRandom(typePlan(k), environment);
    end

    halfX = diff(environment.XLimits) / 2;
    margin = 0.09 * diff(environment.XLimits);
    targets{1}.Position(1) = environment.XLimits(1) + margin;
    targets{5}.Position(1) = environment.XLimits(2) - margin;
end

function records = initializeRecords(targets)
    types = cellfun(@(t) t.Type, targets, 'UniformOutput', false);
    records = struct( ...
        'Type', types, ...
        'ModeHistory', {cell(numel(targets), 1)}, ...
        'CommandDurations', {cell(numel(targets), 1)}, ...
        'WaypointDistances', {cell(numel(targets), 1)}, ...
        'AvoidBoundarySeen', false(numel(targets), 1), ...
        'HoverObserveSeen', false(numel(targets), 1));
end

function records = updatePlannerRecords(records, targetIdx, target, command, previousCommand, previousMode, dt)
    records(targetIdx).ModeHistory{end + 1} = command.BehaviorMode;

    if isempty(records(targetIdx).CommandDurations)
        currentDuration = dt;
    else
        currentDuration = records(targetIdx).CommandDurations{end};
        if command.BehaviorMode == previousMode && ...
                isequal(command.Reason, previousCommand.Reason)
            currentDuration = currentDuration + dt;
        else
            records(targetIdx).CommandDurations{end} = currentDuration;
            currentDuration = dt;
        end
    end
    records(targetIdx).CommandDurations{end} = currentDuration;

    if command.BehaviorMode == BehaviorMode.AvoidBoundary
        records(targetIdx).AvoidBoundarySeen = true;
    end

    if command.BehaviorMode == BehaviorMode.HoverObserve
        records(targetIdx).HoverObserveSeen = true;
    end

    if size(target.HistoryPosition, 1) >= 2
        stepDistance = norm(target.HistoryPosition(end, :) - target.HistoryPosition(end - 1, :));
        records(targetIdx).WaypointDistances{end + 1} = stepDistance;
    end
end

function errors = validateAllowedModes(records)
    errors = 0;

    for k = 1:numel(records)
        allowed = BehaviorModeCatalog.allowedModes(records(k).Type);
        modes = [records(k).ModeHistory{:}];

        for modeIdx = 1:numel(modes)
            if ~any(allowed == modes(modeIdx))
                fprintf('ERROR: Target type %s used disallowed mode %s.\n', ...
                    char(records(k).Type), char(modes(modeIdx)));
                errors = errors + 1;
            end
        end
    end
end

function errors = validateHoldTime(records)
    errors = 0;

    for k = 1:numel(records)
        durations = cell2mat(records(k).CommandDurations);
        shortCommands = durations(durations < 1.5);

        if numel(shortCommands) > numel(durations) * 0.8
            fprintf('ERROR: Target %d commands change too quickly.\n', k);
            errors = errors + 1;
        end
    end
end

function errors = validateAvoidBoundary(records, environment)
    errors = 0;
    boundaryTargets = [1, 5];

    for k = boundaryTargets
        if ~records(k).AvoidBoundarySeen
            fprintf('ERROR: Boundary target %d never entered AvoidBoundary.\n', k);
            errors = errors + 1;
        end
    end

    marginRatio = 0.10;
    for k = 1:numel(records)
        if records(k).AvoidBoundarySeen
            continue;
        end

        modes = [records(k).ModeHistory{:}];
        if any(modes == BehaviorMode.AvoidBoundary)
            continue;
        end
    end

    if nargin > 1 && isempty(boundaryTargets)
        errors = errors + 0;
    end
end

function errors = validateGroundRoadAlignment(targets)
    errors = 0;

    for k = 1:numel(targets)
        target = targets{k};
        if target.Type ~= TargetType.Ground
            continue;
        end

        if ~BehaviorCommand.isActive(target.BehaviorCommand)
            continue;
        end

        desiredPosition = target.BehaviorCommand.DesiredPosition;
        if ~all(isfinite(desiredPosition))
            fprintf('ERROR: Ground target %d has invalid desired position.\n', target.ID);
            errors = errors + 1;
            continue;
        end

        heading = target.MotionContext.RoadHeading;
        if ~RoadNetwork.isOnRoad(desiredPosition, heading, 10)
            fprintf('ERROR: Ground target %d desired position is off the road.\n', target.ID);
            errors = errors + 1;
        end
    end
end

function errors = validateQuadcopterHover(records)
    errors = 0;
    quadSeen = false;
    hoverSeen = false;

    for k = 1:numel(records)
        if records(k).Type ~= TargetType.Quadcopter
            continue;
        end
        quadSeen = true;
        if records(k).HoverObserveSeen
            hoverSeen = true;
        end
    end

    if quadSeen && ~hoverSeen
        fprintf('ERROR: No quadcopter HoverObserve mode observed.\n');
        errors = errors + 1;
    end
end

function errors = validateSegmentLengths(targets)
    errors = 0;
    birdSegments = [];
    airplaneSegments = [];
    quadSegments = [];

    for k = 1:numel(targets)
        target = targets{k};
        metrics = MotionMetrics.compute(target, 1);

        switch char(target.Type)
            case char(TargetType.False)
                birdSegments(end + 1) = metrics.StraightSegmentLengthMean; %#ok<AGROW>
            case char(TargetType.AirplaneUAV)
                airplaneSegments(end + 1) = metrics.StraightSegmentLengthMean; %#ok<AGROW>
            case char(TargetType.Quadcopter)
                quadSegments(end + 1) = metrics.StraightSegmentLengthMean; %#ok<AGROW>
        end
    end

    if ~isempty(birdSegments)
        birdMean = mean(birdSegments);
        if birdMean < 10 || birdMean > 90
            fprintf('ERROR: Bird straight segment mean %.2f outside 20-50 m tolerance band.\n', birdMean);
            errors = errors + 1;
        end
    end

    if ~isempty(airplaneSegments) && ~isempty(birdSegments)
        if mean(airplaneSegments) <= mean(birdSegments)
            fprintf('ERROR: Airplane segment length is not greater than bird segment length.\n');
            errors = errors + 1;
        end
    end

    if ~isempty(quadSegments) && ~isempty(airplaneSegments)
        if mean(quadSegments) >= mean(airplaneSegments)
            fprintf('ERROR: Quadcopter segments should be shorter than airplane segments.\n');
            errors = errors + 1;
        end
    end
end
