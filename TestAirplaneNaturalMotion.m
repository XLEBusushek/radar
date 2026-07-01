% TestAirplaneNaturalMotion  Валидация Natural Motion для AirplaneUAV (ТЗ №14.3).

function TestAirplaneNaturalMotion()
    rng(71);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Airplane Natural Motion Validation ===\n\n');

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
    [target, simStats, errorsAfterSim] = runAirplaneNaturalMotionSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateMissionIntegrity(target);
    errors = errors + validateNoiseLimits(simStats);
    errors = errors + validateSmoothDynamics(simStats);
    errors = errors + validateWaypointTolerance(simStats);
    errors = errors + validateCyclicPatrol(simStats, target);
    errors = errors + validateNoHover(simStats);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Airplane Natural Motion Validation PASSED\n');
    else
        fprintf('Airplane Natural Motion Validation FAILED\n');
    end
end

function [target, stats, errors] = runAirplaneNaturalMotionSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createAirplaneTarget(environment);
    engine = DecisionEngine();
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.AirplaneUAV);
    profile = TargetProfileRegistry.getProfile(TargetType.AirplaneUAV);

    stats = struct();
    stats.HeadingNoise = zeros(simDuration, 1);
    stats.SpeedNoise = zeros(simDuration, 1);
    stats.AltitudeNoise = zeros(simDuration, 1);
    stats.Speed = zeros(simDuration, 1);
    stats.Altitude = zeros(simDuration, 1);
    stats.Heading = zeros(simDuration, 1);
    stats.HoverCount = 0;
    stats.WideTurnCount = 0;
    stats.WaypointAdvanceCount = 0;
    stats.MinWaypointDistance = inf;
    stats.InitialWaypointIndex = 1;
    stats.CompletedLap = false;
    stats.MissionRouteSnapshot = [];

    for step = 1:simDuration
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

        missionBeforeNml = captureMissionSnapshot(target);
        [target, behaviorCommand] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);
        missionAfterNml = captureMissionSnapshot(target);

        if isempty(stats.MissionRouteSnapshot) && ~isempty(missionBeforeNml.MissionRoute)
            stats.MissionRouteSnapshot = missionBeforeNml.MissionRoute;
            stats.InitialWaypointIndex = missionBeforeNml.CurrentWaypointIndex;
        end

        if ~isequal(missionBeforeNml.MissionRoute, missionAfterNml.MissionRoute)
            fprintf('ERROR: Natural motion changed MissionRoute at step %d.\n', step);
            errors = errors + 1;
        end
        if missionBeforeNml.MissionType ~= missionAfterNml.MissionType
            fprintf('ERROR: Natural motion changed MissionType at step %d.\n', step);
            errors = errors + 1;
        end
        if missionBeforeNml.CurrentWaypointIndex ~= missionAfterNml.CurrentWaypointIndex
            fprintf('ERROR: Natural motion changed CurrentWaypointIndex at step %d.\n', step);
            errors = errors + 1;
        end

        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        state = target.NaturalMotionState;
        stats.HeadingNoise(step) = state.HeadingNoise;
        stats.SpeedNoise(step) = state.SpeedNoise;
        stats.AltitudeNoise(step) = state.AltitudeNoise;
        stats.Speed(step) = target.Speed;
        stats.Altitude(step) = target.Position(3);
        stats.Heading(step) = target.Heading;

        if decision.NextState == TargetBehaviorState.Hover
            stats.HoverCount = stats.HoverCount + 1;
        end
        if behaviorCommand.BehaviorMode == BehaviorMode.WideTurn
            stats.WideTurnCount = stats.WideTurnCount + 1;
        end

        waypointDistance = norm(target.Position(1:2) - missionCommand.CurrentWaypoint(1:2));
        stats.MinWaypointDistance = min(stats.MinWaypointDistance, waypointDistance);
        if isfield(missionCommand, 'WaypointTolerance') && ...
                waypointDistance <= missionCommand.WaypointTolerance
            stats.WaypointAdvanceCount = stats.WaypointAdvanceCount + 1;
        end

        if missionCommand.ReachedWaypointCount >= missionCommand.TotalWaypointCount && ...
                missionCommand.CurrentWaypointIndex == stats.InitialWaypointIndex
            stats.CompletedLap = true;
        end
    end

    if max(abs(stats.HeadingNoise)) > motionProfile.MaxHeadingNoise + 1e-9
        fprintf('ERROR: HeadingNoise %.4f rad exceeds max %.4f rad.\n', ...
            max(abs(stats.HeadingNoise)), motionProfile.MaxHeadingNoise);
        errors = errors + 1;
    end

    if max(abs(stats.SpeedNoise)) > motionProfile.MaxSpeedNoise + 1e-9
        fprintf('ERROR: SpeedNoise %.4f exceeds max %.4f.\n', ...
            max(abs(stats.SpeedNoise)), motionProfile.MaxSpeedNoise);
        errors = errors + 1;
    end

    if max(abs(stats.AltitudeNoise)) > motionProfile.MaxAltitudeNoise + 1e-9
        fprintf('ERROR: AltitudeNoise %.4f exceeds max %.4f.\n', ...
            max(abs(stats.AltitudeNoise)), motionProfile.MaxAltitudeNoise);
        errors = errors + 1;
    end

    if stats.WideTurnCount == 0
        fprintf('ERROR: No WideTurn behavior observed during patrol.\n');
        errors = errors + 1;
    end

    if std(stats.HeadingNoise) < deg2rad(0.05)
        fprintf('ERROR: HeadingNoise did not vary during simulation.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionIntegrity(target)
    errors = 0;

    if target.MissionCommand.MissionType ~= MissionType.PatrolRoute
        fprintf('ERROR: Mission type is not PatrolRoute.\n');
        errors = errors + 1;
    end

    if ~(MissionCommand.isActive(target.MissionCommand) && ...
            target.MissionCommand.Status == MissionStatus.Executing)
        fprintf('ERROR: Patrol mission is not executing.\n');
        errors = errors + 1;
    end

    if isempty(target.MissionRoute) || any(~isfinite(target.MissionRoute(:)))
        fprintf('ERROR: Mission route became invalid.\n');
        errors = errors + 1;
    end
end

function errors = validateNoiseLimits(stats)
    errors = 0;
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.AirplaneUAV);

    if max(abs(stats.HeadingNoise)) > motionProfile.MaxHeadingNoise + 1e-9
        fprintf('ERROR: HeadingNoise exceeds profile limit.\n');
        errors = errors + 1;
    end

    if max(abs(stats.AltitudeNoise)) > motionProfile.MaxAltitudeNoise + 1e-9
        fprintf('ERROR: AltitudeNoise exceeds profile limit.\n');
        errors = errors + 1;
    end

    if max(abs(stats.SpeedNoise)) > motionProfile.MaxSpeedNoise + 1e-9
        fprintf('ERROR: SpeedNoise exceeds profile limit.\n');
        errors = errors + 1;
    end
end

function errors = validateSmoothDynamics(stats)
    errors = 0;
    profile = TargetProfileRegistry.getProfile(TargetType.AirplaneUAV);

    speedSteps = abs(diff(stats.Speed));
    maxSpeedStep = max(speedSteps);
    if maxSpeedStep > profile.MaxAcceleration * 1.2 + 0.3
        fprintf('ERROR: Speed jump %.3f is too abrupt.\n', maxSpeedStep);
        errors = errors + 1;
    end

    altitudeSteps = abs(diff(stats.Altitude));
    if max(altitudeSteps) > 2.5
        fprintf('ERROR: Altitude jump %.3f is too abrupt.\n', max(altitudeSteps));
        errors = errors + 1;
    end

    if numel(stats.Heading) > 1
        turnRates = abs(MotionKinematics.wrapAngle(diff(stats.Heading)));
        if max(turnRates) > deg2rad(profile.MaxTurnRate) * 1.05
            fprintf('ERROR: Heading changed too sharply for airplane inertia.\n');
            errors = errors + 1;
        end
    end
end

function errors = validateWaypointTolerance(stats)
    errors = 0;
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.AirplaneUAV);

    if stats.MinWaypointDistance > motionProfile.WaypointToleranceMax + 5
        fprintf('ERROR: Closest waypoint distance %.1f m exceeds tolerance band.\n', ...
            stats.MinWaypointDistance);
        errors = errors + 1;
    end

    if stats.WaypointAdvanceCount == 0
        fprintf('ERROR: No waypoint reached within tolerance radius.\n');
        errors = errors + 1;
    end
end

function errors = validateCyclicPatrol(stats, target)
    errors = 0;

    if ~stats.CompletedLap
        fprintf('ERROR: Cyclic patrol lap was not completed.\n');
        errors = errors + 1;
    end

    if ~isfield(target.MissionCommand, 'IsCyclic') || ~target.MissionCommand.IsCyclic
        fprintf('ERROR: Patrol mission is not cyclic.\n');
        errors = errors + 1;
    end
end

function errors = validateNoHover(stats)
    errors = 0;

    if stats.HoverCount > 0
        fprintf('ERROR: Airplane entered Hover state %d times.\n', stats.HoverCount);
        errors = errors + 1;
    end
end

function snapshot = captureMissionSnapshot(target)
    snapshot = struct( ...
        'MissionRoute', target.MissionRoute, ...
        'CurrentWaypointIndex', target.MissionWaypointIndex, ...
        'MissionType', target.MissionType);
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
