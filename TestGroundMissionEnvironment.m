% TestGroundMissionEnvironment  Валидация наземных миссий по Environment (ТЗ №13.4).

function TestGroundMissionEnvironment()
    rng(61);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Ground Mission Environment Validation ===\n\n');

    dt = 1;
    simDuration = 180;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', [2000, 2000, 400], ...
        'RandomSeed', 61, ...
        'MinAltitude', 0, ...
        'MaxAltitude', 400, ...
        'SimulationTime', simDuration, ...
        'TimeStep', dt);

    errors = 0;
    errors = errors + validateMissionRoute(environment);
    [target, errorsAfterSim] = runGroundSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateMissionCompletion(target);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Ground Mission Environment Validation PASSED\n');
    else
        fprintf('Ground Mission Environment Validation FAILED\n');
    end
end

function errors = validateMissionRoute(environment)
    errors = 0;
    spawnPoint = environment.SpawnPoints.Ground(1, :);
    targetStub = struct( ...
        'ID', 1, ...
        'Position', spawnPoint, ...
        'Type', TargetType.Ground, ...
        'MissionTime', 0);
    command = GroundMissionPlanner.createMission(targetStub, environment);
    route = command.MissionRoute;

    if ~RoadGraph.isRouteOnRoads(environment, route, 8)
        fprintf('ERROR: Mission route contains off-road points.\n');
        errors = errors + 1;
    end

    if ~RoadGraph.isRouteConnected(environment, route, 80)
        fprintf('ERROR: Mission route has disconnected road jumps.\n');
        errors = errors + 1;
    end

    if size(route, 1) < 2
        fprintf('ERROR: Mission route is too short.\n');
        errors = errors + 1;
    end

    if ~isfield(command, 'DestinationNodeIndex') || command.DestinationNodeIndex < 1
        fprintf('ERROR: Mission route missing destination node.\n');
        errors = errors + 1;
    end
end

function [target, errors] = runGroundSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createGroundTarget(environment);
    engine = DecisionEngine();
    speedBeforeTurn = [];
    turnSamples = 0;
    offRoadCount = 0;
    graph = RoadGraph.fromEnvironment(environment);
    previousPhase = '';

    for step = 1:simDuration
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);
        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        if ~RoadGraph.isOnRoadNetwork(environment, target.Position, 10)
            offRoadCount = offRoadCount + 1;
        end

        if (isfield(target.MotionContext, 'GroundPhase') && ...
                strcmp(target.MotionContext.GroundPhase, 'ApproachIntersection')) || ...
                (BehaviorCommand.isActive(behaviorCommand) && ...
                behaviorCommand.BehaviorMode == BehaviorMode.ApproachIntersection)
            speedBeforeTurn(end + 1) = target.Speed; %#ok<AGROW>
        end

        if isfield(target.MotionContext, 'GroundPhase') && ...
                strcmp(target.MotionContext.GroundPhase, 'Turn')
            turnSamples = turnSamples + 1;
            if ~strcmp(previousPhase, 'Turn') && ...
                    RoadGraph.distanceToNearestIntersection(graph, target.Position) > 45
                fprintf('ERROR: Turn detected away from intersection at step %d.\n', step);
                errors = errors + 1;
            end
        end

        if isfield(target.MotionContext, 'GroundPhase')
            previousPhase = target.MotionContext.GroundPhase;
        end
    end

    if offRoadCount > simDuration * 0.05
        fprintf('ERROR: Ground vehicle left road network too often (%d steps).\n', offRoadCount);
        errors = errors + 1;
    end

    if isempty(speedBeforeTurn)
        fprintf('ERROR: No approach speed reduction observed before turns.\n');
        errors = errors + 1;
    elseif ~any(speedBeforeTurn <= 13) && mean(speedBeforeTurn) > 15
        fprintf('ERROR: Approach speed not reduced enough before turns.\n');
        errors = errors + 1;
    end

    if turnSamples == 0
        fprintf('ERROR: No turn phase observed during simulation.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionCompletion(target)
    errors = 0;
    historyTypes = [target.MissionHistory.MissionType];
    completed = any(historyTypes == MissionType.FollowRoadRoute);

    if ~completed && ~(MissionCommand.isActive(target.MissionCommand) && ...
            target.MissionCommand.Status == MissionStatus.Executing)
        fprintf('ERROR: Ground mission did not remain active or complete.\n');
        errors = errors + 1;
    end

    if MissionStateMachine.isTerminal(target.MissionCommand) && ...
            target.MissionCommand.Progress < 1
        fprintf('ERROR: Completed mission has invalid progress.\n');
        errors = errors + 1;
    end
end

function target = createGroundTarget(environment)
    spawnPoint = environment.SpawnPoints.Ground(1, :);
    profile = TargetProfileRegistry.getProfile(TargetType.Ground);
    speed = TargetFactory.effectiveSpeedMin(profile) + 5;
    target = RadarTargetModel(TargetType.Ground, spawnPoint, 0, speed, 2.0);
    target = TargetFactory.initializeMotionContext(target, environment, profile);
end
