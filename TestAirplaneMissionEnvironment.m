% TestAirplaneMissionEnvironment  Валидация миссий AirplaneUAV по Environment (ТЗ №13.5).

function TestAirplaneMissionEnvironment()
    rng(71);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Airplane Mission Environment Validation ===\n\n');

    dt = 1;
    simDuration = 360;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', [8000, 8000, 500], ...
        'RandomSeed', 71, ...
        'MinAltitude', 0, ...
        'MaxAltitude', 500, ...
        'SimulationTime', simDuration, ...
        'TimeStep', dt);

    errors = 0;
    errors = errors + validateMissionRoute(environment);
    [target, errorsAfterSim] = runAirplaneSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateMissionCompletion(target);
    errors = errors + validateComparedSegmentLengths(target, environment, simDuration, dt);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Airplane Mission Environment Validation PASSED\n');
    else
        fprintf('Airplane Mission Environment Validation FAILED\n');
    end
end

function errors = validateMissionRoute(environment)
    errors = 0;
    spawnPoint = environment.SpawnPoints.Airplane(1, :);
    targetStub = struct( ...
        'ID', 1, ...
        'Position', spawnPoint, ...
        'Type', TargetType.AirplaneUAV, ...
        'MissionTime', 0);
    command = AirplaneMissionPlanner.createMission(targetStub, environment);
    route = command.MissionRoute;

    if ~isfield(command, 'PatrolZoneIndex') || command.PatrolZoneIndex < 1
        fprintf('ERROR: Mission route is not bound to a PatrolZone.\n');
        errors = errors + 1;
    else
        zone = environment.PatrolZones(command.PatrolZoneIndex);
        if abs(command.DesiredMissionAltitude - zone.PreferredAltitude) > 20.5
            fprintf('ERROR: Mission altitude is outside PatrolZone preferred range.\n');
            errors = errors + 1;
        end
    end

    waypointCount = size(route, 1);
    if waypointCount < 4 || waypointCount > 6
        fprintf('ERROR: Patrol route has %d waypoints, expected 4..6.\n', waypointCount);
        errors = errors + 1;
    end

    if command.DesiredMissionSpeed < 14 || command.DesiredMissionSpeed > 18
        fprintf('ERROR: Mission speed %.2f outside 14..18 m/s.\n', command.DesiredMissionSpeed);
        errors = errors + 1;
    end

    minSegment = PatrolRouteBuilder.minSegmentLength(route);
    if minSegment < 250
        fprintf('ERROR: Patrol route min segment %.1f m is below 250 m.\n', minSegment);
        errors = errors + 1;
    end

    if ~isfield(command, 'IsCyclic') || ~command.IsCyclic
        fprintf('ERROR: Patrol mission is not cyclic.\n');
        errors = errors + 1;
    end
end

function [target, errors] = runAirplaneSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createAirplaneTarget(environment);
    engine = DecisionEngine();
    profile = TargetProfileRegistry.getProfile(target.Type);
    missionAltitudeHistory = [];
    speedHistory = [];
    headingHistory = [];
    hoverCount = 0;
    wideTurnCount = 0;
    completedLap = false;
    initialWaypointIndex = 1;

    for step = 1:simDuration
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);
        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        missionAltitudeHistory(end + 1) = missionCommand.DesiredMissionAltitude; %#ok<AGROW>
        speedHistory(end + 1) = target.Speed; %#ok<AGROW>
        headingHistory(end + 1) = target.Heading; %#ok<AGROW>

        if decision.NextState == TargetBehaviorState.Hover
            hoverCount = hoverCount + 1;
        end

        if behaviorCommand.BehaviorMode == BehaviorMode.WideTurn
            wideTurnCount = wideTurnCount + 1;
        end

        if missionCommand.ReachedWaypointCount >= missionCommand.TotalWaypointCount && ...
                missionCommand.CurrentWaypointIndex == initialWaypointIndex
            completedLap = true;
        end
    end

    if hoverCount > 0
        fprintf('ERROR: Airplane entered Hover state during patrol.\n');
        errors = errors + 1;
    end

    if wideTurnCount == 0
        fprintf('ERROR: No WideTurn behavior observed during patrol.\n');
        errors = errors + 1;
    end

    if numel(unique(missionAltitudeHistory)) > 1
        fprintf('ERROR: Mission altitude changed during execution.\n');
        errors = errors + 1;
    end

    if mean(speedHistory) < 13 || mean(speedHistory) > 19
        fprintf('ERROR: Mean airplane speed %.2f outside patrol range.\n', mean(speedHistory));
        errors = errors + 1;
    end

    maxSpeedStep = max(abs(diff(speedHistory)));
    if maxSpeedStep > profile.MaxDeceleration * dt * 1.2 + 0.2 && ...
            maxSpeedStep > profile.MaxAcceleration * dt * 1.2 + 0.2
        fprintf('ERROR: Airplane speed changed too abruptly (step %.2f).\n', maxSpeedStep);
        errors = errors + 1;
    end

    if numel(headingHistory) > 1
        turnRates = abs(MotionKinematics.wrapAngle(diff(headingHistory))) / dt;
        if max(turnRates) > deg2rad(profile.MaxTurnRate) * 1.05
            fprintf('ERROR: Airplane turn rate exceeds profile.\n');
            errors = errors + 1;
        end
    end

    if ~completedLap
        fprintf('ERROR: Airplane did not complete a patrol lap.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionCompletion(target)
    errors = 0;

    if ~(MissionCommand.isActive(target.MissionCommand) && ...
            target.MissionCommand.Status == MissionStatus.Executing)
        fprintf('ERROR: Airplane patrol mission is not still executing.\n');
        errors = errors + 1;
    end

    if target.MissionCommand.MissionType ~= MissionType.PatrolRoute
        fprintf('ERROR: Airplane mission type is not PatrolRoute.\n');
        errors = errors + 1;
    end
end

function errors = validateComparedSegmentLengths(target, environment, simDuration, dt)
    errors = 0;
    airplaneMetrics = MotionMetrics.compute(target, dt);

    birdTarget = createComparisonTarget(TargetType.False, environment);
    quadTarget = createComparisonTarget(TargetType.Quadcopter, environment);
    engine = DecisionEngine();

    for step = 1:min(simDuration, 120)
        [birdTarget, birdMission] = MissionPlanner.plan(birdTarget, environment, dt);
        [birdTarget, birdBehavior] = BehaviorPlanner.plan(birdTarget, environment, birdMission, dt);
        birdDecision = engine.decide(birdTarget.toDecisionInput(), environment, birdBehavior);
        birdTarget = TrajectoryGenerator.updateMotion(birdTarget, birdDecision, birdBehavior, environment, dt);

        [quadTarget, quadMission] = MissionPlanner.plan(quadTarget, environment, dt);
        [quadTarget, quadBehavior] = BehaviorPlanner.plan(quadTarget, environment, quadMission, dt);
        quadDecision = engine.decide(quadTarget.toDecisionInput(), environment, quadBehavior);
        quadTarget = TrajectoryGenerator.updateMotion(quadTarget, quadDecision, quadBehavior, environment, dt);
    end

    birdMetrics = MotionMetrics.compute(birdTarget, dt);
    quadMetrics = MotionMetrics.compute(quadTarget, dt);

    if airplaneMetrics.StraightSegmentLengthMean <= birdMetrics.StraightSegmentLengthMean
        fprintf('ERROR: Airplane straight segments are not longer than bird segments.\n');
        errors = errors + 1;
    end

    if airplaneMetrics.StraightSegmentLengthMean <= quadMetrics.StraightSegmentLengthMean
        fprintf('ERROR: Airplane straight segments are not longer than quadcopter segments.\n');
        errors = errors + 1;
    end
end

function target = createAirplaneTarget(environment)
    spawnPoint = environment.SpawnPoints.Airplane(1, :);
    profile = TargetProfileRegistry.getProfile(TargetType.AirplaneUAV);
    speed = 14 + 2;
    target = RadarTargetModel(TargetType.AirplaneUAV, spawnPoint, 0, speed, 0.05);
    target = TargetFactory.initializeMotionContext(target, environment, profile);

    missionCommand = AirplaneMissionPlanner.createMission(target, environment);
    if size(missionCommand.MissionRoute, 1) >= 1
        routeDelta = missionCommand.MissionRoute(1, 1:2) - spawnPoint(1:2);
        if norm(routeDelta) > 1
            target.Heading = atan2(routeDelta(2), routeDelta(1));
            target.Velocity = [ ...
                target.Speed * cos(target.Heading), ...
                target.Speed * sin(target.Heading), ...
                0];
        end
    end
end

function target = createComparisonTarget(targetType, environment)
  profile = TargetProfileRegistry.getProfile(targetType);
  position = [0, 0, 120];
  if targetType == TargetType.Quadcopter
      position(3) = 80;
  end
  target = RadarTargetModel(targetType, position, 0, profile.SpeedMin + 2, 0.1);
  target = TargetFactory.initializeMotionContext(target, environment, profile);
end
