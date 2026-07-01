% TestGroundNaturalMotion  Валидация Natural Motion для наземного транспорта (ТЗ №14.2).

function TestGroundNaturalMotion()
    rng(61);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Ground Natural Motion Validation ===\n\n');

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
    [target, simStats, errorsAfterSim] = runGroundNaturalMotionSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateMissionIntegrity(target);
    errors = errors + validateRoadGraphIntegrity(target, environment);
    errors = errors + validateLaneOffsetNoise(simStats);
    errors = errors + validateRoadWidth(simStats, environment);
    errors = errors + validateSpeedSmoothness(simStats);
    errors = errors + validateRideHeight(simStats);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Ground Natural Motion Validation PASSED\n');
    else
        fprintf('Ground Natural Motion Validation FAILED\n');
    end
end

function [target, stats, errors] = runGroundNaturalMotionSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createGroundTarget(environment);
    engine = DecisionEngine();
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.Ground);
    profile = TargetProfileRegistry.getProfile(TargetType.Ground);

    stats = struct();
    stats.LaneOffsetNoise = zeros(simDuration, 1);
    stats.LateralOffset = zeros(simDuration, 1);
    stats.Speed = zeros(simDuration, 1);
    stats.AltitudeAboveTerrain = zeros(simDuration, 1);
    stats.ApproachSpeeds = [];
    stats.TurnSamples = 0;
    stats.OffRoadSteps = 0;

    graph = RoadGraph.fromEnvironment(environment);
    previousPhase = '';

    for step = 1:simDuration
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

        missionBeforeNml = captureMissionSnapshot(target);
        [target, behaviorCommand] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);
        missionAfterNml = captureMissionSnapshot(target);

        if ~isequal(missionBeforeNml.MissionRoute, missionAfterNml.MissionRoute)
            fprintf('ERROR: Natural motion changed MissionRoute at step %d.\n', step);
            errors = errors + 1;
        end
        if missionBeforeNml.CurrentWaypointIndex ~= missionAfterNml.CurrentWaypointIndex
            fprintf('ERROR: Natural motion changed CurrentWaypointIndex at step %d.\n', step);
            errors = errors + 1;
        end
        if missionBeforeNml.MissionType ~= missionAfterNml.MissionType
            fprintf('ERROR: Natural motion changed MissionType at step %d.\n', step);
            errors = errors + 1;
        end

        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        terrainHeight = environment.Terrain.Height(target.Position(1), target.Position(2));
        stats.LaneOffsetNoise(step) = target.NaturalMotionState.LaneOffsetNoise;
        stats.LateralOffset(step) = RoadGraph.lateralOffsetFromRoad(environment, target.Position);
        stats.Speed(step) = target.Speed;
        stats.AltitudeAboveTerrain(step) = target.Position(3) - terrainHeight;

        if ~RoadGraph.isOnRoadNetwork(environment, target.Position, 10)
            stats.OffRoadSteps = stats.OffRoadSteps + 1;
        end

        if isfield(target.MotionContext, 'GroundPhase') && ...
                strcmp(target.MotionContext.GroundPhase, 'ApproachIntersection')
            stats.ApproachSpeeds(end + 1) = target.Speed; %#ok<AGROW>
        end

        if isfield(target.MotionContext, 'GroundPhase') && ...
                strcmp(target.MotionContext.GroundPhase, 'Turn')
            stats.TurnSamples = stats.TurnSamples + 1;
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

    if max(abs(stats.LaneOffsetNoise)) > motionProfile.MaxLaneOffsetNoise + 1e-9
        fprintf('ERROR: LaneOffsetNoise %.4f exceeds max %.4f.\n', ...
            max(abs(stats.LaneOffsetNoise)), motionProfile.MaxLaneOffsetNoise);
        errors = errors + 1;
    end

    if stats.OffRoadSteps > simDuration * 0.05
        fprintf('ERROR: Vehicle left road network too often (%d steps).\n', stats.OffRoadSteps);
        errors = errors + 1;
    end

    if isempty(stats.ApproachSpeeds)
        fprintf('ERROR: No approach speed reduction observed before turns.\n');
        errors = errors + 1;
    elseif mean(stats.ApproachSpeeds) > 16
        fprintf('ERROR: Approach phase did not reduce speed enough.\n');
        errors = errors + 1;
    end

    if stats.TurnSamples == 0
        fprintf('ERROR: No turn phase observed during simulation.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionIntegrity(target)
    errors = 0;

    if ~MissionCommand.isActive(target.MissionCommand)
        fprintf('ERROR: Ground mission is not active after simulation.\n');
        errors = errors + 1;
    end

    if target.MissionCommand.MissionType ~= MissionType.FollowRoadRoute
        fprintf('ERROR: Ground mission type changed to %s.\n', char(target.MissionCommand.MissionType));
        errors = errors + 1;
    end

    if isempty(target.MissionRoute) || any(~isfinite(target.MissionRoute(:)))
        fprintf('ERROR: Mission route became invalid.\n');
        errors = errors + 1;
    end
end

function errors = validateRoadGraphIntegrity(target, environment)
    errors = 0;

    if ~RoadGraph.isRouteOnRoads(environment, target.MissionRoute, 8)
        fprintf('ERROR: Mission route contains off-road points.\n');
        errors = errors + 1;
    end

    if ~RoadGraph.isRouteConnected(environment, target.MissionRoute, 80)
        fprintf('ERROR: Mission route has disconnected road jumps.\n');
        errors = errors + 1;
    end

    if isfield(target.MotionContext, 'RoadNodePath') && ~isempty(target.MotionContext.RoadNodePath)
        if any(target.MotionContext.RoadNodePath < 1)
            fprintf('ERROR: RoadNodePath contains invalid node indices.\n');
            errors = errors + 1;
        end
    end
end

function errors = validateLaneOffsetNoise(stats)
    errors = 0;

    if max(abs(stats.LaneOffsetNoise)) > 0.7 + 1e-9
        fprintf('ERROR: LaneOffsetNoise exceeded +/-0.7 m.\n');
        errors = errors + 1;
    end

    if std(stats.LaneOffsetNoise) < 0.01
        fprintf('ERROR: LaneOffsetNoise did not vary during simulation.\n');
        errors = errors + 1;
    end
end

function errors = validateRoadWidth(stats, ~)
    errors = 0;
    halfWidth = RoadGraph.ROAD_WIDTH / 2;
    tolerance = 0.35;

    if max(abs(stats.LateralOffset)) > halfWidth + tolerance
        fprintf('ERROR: Lateral offset %.2f m exceeds road half-width %.2f m.\n', ...
            max(abs(stats.LateralOffset)), halfWidth);
        errors = errors + 1;
    end

    if std(stats.LateralOffset) < 0.05
        fprintf('ERROR: Vehicle did not drift within lane.\n');
        errors = errors + 1;
    end
end

function errors = validateSpeedSmoothness(stats)
    errors = 0;
    profile = TargetProfileRegistry.getProfile(TargetType.Ground);
    maxStep = profile.MaxAcceleration * 1.2 + 0.5;
    speedSteps = abs(diff(stats.Speed));

    if max(speedSteps) > maxStep + 1e-6
        fprintf('ERROR: Speed jump %.3f exceeds smooth limit %.3f.\n', max(speedSteps), maxStep);
        errors = errors + 1;
    end

    if std(stats.Speed) < 0.02
        fprintf('ERROR: Speed did not show natural micro-variation.\n');
        errors = errors + 1;
    end
end

function errors = validateRideHeight(stats)
    errors = 0;
    rideHeight = 0.8;
    tolerance = 0.3;

    if any(stats.AltitudeAboveTerrain < rideHeight - tolerance - 0.05) || ...
            any(stats.AltitudeAboveTerrain > rideHeight + tolerance + 0.05)
        fprintf('ERROR: Ride height left Terrain + %.1f +/- %.1f range.\n', rideHeight, tolerance);
        errors = errors + 1;
    end
end

function snapshot = captureMissionSnapshot(target)
    snapshot = struct( ...
        'MissionRoute', target.MissionRoute, ...
        'CurrentWaypointIndex', target.MissionWaypointIndex, ...
        'MissionType', target.MissionType);
end

function target = createGroundTarget(environment)
    spawnPoint = environment.SpawnPoints.Ground(1, :);
    profile = TargetProfileRegistry.getProfile(TargetType.Ground);
    speed = TargetFactory.effectiveSpeedMin(profile) + 5;
    target = RadarTargetModel(TargetType.Ground, spawnPoint, 0, speed, 2.0);
    target = TargetFactory.initializeMotionContext(target, environment, profile);
end
