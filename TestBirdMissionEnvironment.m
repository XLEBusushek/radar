% TestBirdMissionEnvironment  Validation of Bird missions from Environment (TZ 13.7).

function TestBirdMissionEnvironment()
    rng(71);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Bird Mission Environment Validation ===\n\n');

    dt = 1;
    simDuration = 240;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', [6000, 6000, 400], ...
        'RandomSeed', 71, ...
        'MinAltitude', 0, ...
        'MaxAltitude', 400, ...
        'SimulationTime', simDuration, ...
        'TimeStep', dt);

    errors = 0;
    errors = errors + validateMissionRoute(environment);
    [~, errorsAfterSim, simRecords] = runBirdSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateHiddenBias(simRecords);
    errors = errors + validateNaturalMotion(simRecords);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Bird Mission Environment Validation PASSED\n');
    else
        fprintf('Bird Mission Environment Validation FAILED\n');
    end
end

function errors = validateMissionRoute(environment)
    errors = 0;
    spawnPoint = environment.SpawnPoints.Bird(1, :);
    targetStub = struct( ...
        'ID', 1, ...
        'Position', spawnPoint, ...
        'Type', TargetType.False, ...
        'MissionTime', 0);
    command = BirdMissionPlanner.createMission(targetStub, environment);
    route = command.MissionRoute;

    if ~isfield(command, 'BirdTreeZoneIndex') || command.BirdTreeZoneIndex < 1
        fprintf('ERROR: Mission route is not bound to a TreeZone.\n');
        errors = errors + 1;
    end

    if command.MissionType ~= MissionType.MoveBetweenZones
        fprintf('ERROR: Bird mission type is not MoveBetweenZones.\n');
        errors = errors + 1;
    end

    waypointCount = size(route, 1);
    if waypointCount < 3 || waypointCount > 8
        fprintf('ERROR: Bird route has %d waypoints, expected 3..8.\n', waypointCount);
        errors = errors + 1;
    end

    if command.DesiredMissionSpeed < 5 || command.DesiredMissionSpeed > 15
        fprintf('ERROR: Mission speed %.2f outside 5..15 m/s.\n', command.DesiredMissionSpeed);
        errors = errors + 1;
    end

    segmentLengths = TreeZoneRouteBuilder.allSegmentLengths(route);
    shortSegments = segmentLengths(segmentLengths >= 19.5 & segmentLengths <= 50.5);
    if isempty(shortSegments)
        fprintf('ERROR: No route segments in the 20..50 m range.\n');
        errors = errors + 1;
    end

    if any(route(:, 3) < -0.5 | route(:, 3) > 40.5)
        fprintf('ERROR: Route altitude is outside 0..40 m.\n');
        errors = errors + 1;
    end

    if sum(route(:, 3) <= 15.5) < 1
        fprintf('ERROR: Route has no low-altitude waypoints near trees.\n');
        errors = errors + 1;
    end

    if ~isfield(command, 'BirdPhase') || command.BirdPhase ~= BirdPhase.MoveToWaypoint
        fprintf('ERROR: Bird mission does not start in MoveToWaypoint phase.\n');
        errors = errors + 1;
    end
end

function [target, errors, records] = runBirdSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createBirdTarget(environment);
    engine = DecisionEngine();

    records = struct( ...
        'LowAltitudeHideSeen', false, ...
        'WaypointChanges', 0, ...
        'StableWaypointSteps', [], ...
        'MissionSpeedSamples', [], ...
        'AltitudeSamples', [], ...
        'HiddenBelow12', 0, ...
        'HiddenAbove12', 0, ...
        'StepsBelow12', 0, ...
        'StepsAbove12', 0, ...
        'HiddenNearTreeBelow12', 0, ...
        'HiddenAwayTreeBelow12', 0, ...
        'StepsNearTreeBelow12', 0, ...
        'StepsAwayTreeBelow12', 0, ...
        'MissionRouteChanged', false, ...
        'MissionCommandChangedByMotion', false, ...
        'MaxWindDrift', 0, ...
        'MaxHeadingNoise', 0, ...
        'MaxAltitudeNoise', 0, ...
        'MaxSpeedNoise', 0, ...
        'AltitudeNoiseSamples', []);

    previousWaypointIndex = 0;
    stableSteps = 0;

    for step = 1:simDuration
        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);

        missionCommandBeforeMotion = target.MissionCommand;
        [target, behaviorCommand] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);

        if ~missionSnapshotEqual(missionCommandBeforeMotion, target.MissionCommand)
            records.MissionCommandChangedByMotion = true;
        end
        if ~isequal(missionCommandBeforeMotion.MissionRoute, target.MissionCommand.MissionRoute)
            records.MissionRouteChanged = true;
        end

        state = target.NaturalMotionState;
        records.MaxWindDrift = max(records.MaxWindDrift, norm(state.WindDrift(1:2)));
        records.MaxHeadingNoise = max(records.MaxHeadingNoise, abs(state.HeadingNoise));
        records.MaxAltitudeNoise = max(records.MaxAltitudeNoise, abs(state.AltitudeNoise));
        records.MaxSpeedNoise = max(records.MaxSpeedNoise, abs(state.SpeedNoise));
        records.AltitudeNoiseSamples(end + 1) = state.AltitudeNoise; %#ok<AGROW>

        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        if isfield(missionCommand, 'BirdPhase') && ...
                missionCommand.BirdPhase == BirdPhase.LowAltitudeHide
            records.LowAltitudeHideSeen = true;
        end

        if missionCommand.Status == MissionStatus.Executing
            records.MissionSpeedSamples(end + 1) = missionCommand.DesiredMissionSpeed; %#ok<AGROW>
        end
        records.AltitudeSamples(end + 1) = target.Position(3); %#ok<AGROW>

        if target.Position(3) < 12
            records.StepsBelow12 = records.StepsBelow12 + 1;
            if target.IsHidden
                records.HiddenBelow12 = records.HiddenBelow12 + 1;
            end

            zoneInfo = Environment.findNearestTreeZone(environment, target.Position);
            nearTree = zoneInfo.Index > 0 && zoneInfo.Distance <= zoneInfo.Zone.Radius * 1.3;
            if nearTree
                records.StepsNearTreeBelow12 = records.StepsNearTreeBelow12 + 1;
                if target.IsHidden
                    records.HiddenNearTreeBelow12 = records.HiddenNearTreeBelow12 + 1;
                end
            else
                records.StepsAwayTreeBelow12 = records.StepsAwayTreeBelow12 + 1;
                if target.IsHidden
                    records.HiddenAwayTreeBelow12 = records.HiddenAwayTreeBelow12 + 1;
                end
            end
        else
            records.StepsAbove12 = records.StepsAbove12 + 1;
            if target.IsHidden
                records.HiddenAbove12 = records.HiddenAbove12 + 1;
            end
        end

        if missionCommand.CurrentWaypointIndex ~= previousWaypointIndex
            records.WaypointChanges = records.WaypointChanges + 1;
            if stableSteps > 0
                records.StableWaypointSteps(end + 1) = stableSteps; %#ok<AGROW>
            end
            stableSteps = 0;
            previousWaypointIndex = missionCommand.CurrentWaypointIndex;
        else
            stableSteps = stableSteps + 1;
        end
    end

    if ~records.LowAltitudeHideSeen
        fprintf('ERROR: LowAltitudeHide phase was not observed.\n');
        errors = errors + 1;
    end

    if isempty(records.MissionSpeedSamples) || ...
            any(records.MissionSpeedSamples < 4.8 | records.MissionSpeedSamples > 15.2)
        fprintf('ERROR: Mission speed left the 5..15 m/s range.\n');
        errors = errors + 1;
    end

    if isempty(records.AltitudeSamples) || ...
            any(records.AltitudeSamples < -0.5 | records.AltitudeSamples > 40.5)
        fprintf('ERROR: Simulated altitude left the 0..40 m range.\n');
        errors = errors + 1;
    end

    if sum(records.AltitudeSamples <= 15.5) < simDuration * 0.05
        fprintf('ERROR: Not enough low-altitude flight near trees.\n');
        errors = errors + 1;
    end

    if ~isempty(records.StableWaypointSteps) && max(records.StableWaypointSteps) < 2
        fprintf('ERROR: Waypoint index changed too frequently during flight.\n');
        errors = errors + 1;
    end

    if records.WaypointChanges > simDuration * 0.5
        fprintf('ERROR: Waypoint index changed on most simulation steps.\n');
        errors = errors + 1;
    end
end

function errors = validateHiddenBias(records)
    errors = 0;

    if records.StepsBelow12 < 5 || records.StepsAbove12 < 5
        fprintf('ERROR: Not enough altitude samples to validate Hidden bias.\n');
        errors = errors + 1;
        return;
    end

    hiddenRateBelow = records.HiddenBelow12 / records.StepsBelow12;
    hiddenRateAbove = records.HiddenAbove12 / records.StepsAbove12;

    if hiddenRateBelow <= hiddenRateAbove
        fprintf('ERROR: Hidden is not more frequent below 12 m (%.3f vs %.3f).\n', ...
            hiddenRateBelow, hiddenRateAbove);
        errors = errors + 1;
    end

    if records.StepsNearTreeBelow12 >= 3 && records.StepsAwayTreeBelow12 >= 3
        hiddenNearTree = records.HiddenNearTreeBelow12 / records.StepsNearTreeBelow12;
        hiddenAwayTree = records.HiddenAwayTreeBelow12 / records.StepsAwayTreeBelow12;
        if hiddenNearTree <= hiddenAwayTree
            fprintf('ERROR: Hidden is not more frequent near TreeZones below 12 m (%.3f vs %.3f).\n', ...
                hiddenNearTree, hiddenAwayTree);
            errors = errors + 1;
        end
    end
end

function errors = validateNaturalMotion(records)
    errors = 0;
    motionProfile = NaturalMotionProfileRegistry.birdMissionProfile();

    if records.MissionRouteChanged
        fprintf('ERROR: Mission route changed during natural motion.\n');
        errors = errors + 1;
    end

    if records.MissionCommandChangedByMotion
        fprintf('ERROR: MissionCommand changed during natural motion layer.\n');
        errors = errors + 1;
    end

    if records.MaxWindDrift < 0.05
        fprintf('ERROR: Wind drift was not observed during bird mission.\n');
        errors = errors + 1;
    end

    if records.MaxWindDrift > motionProfile.MaxWindDrift + 1e-6
        fprintf('ERROR: Wind drift %.2f m exceeds %.2f m limit.\n', ...
            records.MaxWindDrift, motionProfile.MaxWindDrift);
        errors = errors + 1;
    end

    if records.MaxHeadingNoise > motionProfile.MaxHeadingNoise + 1e-6
        fprintf('ERROR: Heading noise %.4f rad exceeds %.4f rad limit.\n', ...
            records.MaxHeadingNoise, motionProfile.MaxHeadingNoise);
        errors = errors + 1;
    end

    if records.MaxAltitudeNoise > motionProfile.MaxAltitudeNoise + 1e-6
        fprintf('ERROR: Altitude noise %.2f m exceeds %.2f m limit.\n', ...
            records.MaxAltitudeNoise, motionProfile.MaxAltitudeNoise);
        errors = errors + 1;
    end

    if records.MaxSpeedNoise > motionProfile.MaxSpeedNoise + 1e-6
        fprintf('ERROR: Speed noise %.2f m/s exceeds %.2f m/s limit.\n', ...
            records.MaxSpeedNoise, motionProfile.MaxSpeedNoise);
        errors = errors + 1;
    end

    if numel(records.AltitudeNoiseSamples) > 2
        altitudeNoiseSteps = abs(diff(records.AltitudeNoiseSamples));
        if max(altitudeNoiseSteps) > 0.35
            fprintf('ERROR: Altitude noise step %.2f m is too abrupt.\n', max(altitudeNoiseSteps));
            errors = errors + 1;
        end
    end
end

function tf = missionSnapshotEqual(before, after)
    tf = isequal(before.MissionType, after.MissionType) && ...
        isequal(before.MissionRoute, after.MissionRoute) && ...
        isequal(before.MissionReason, after.MissionReason) && ...
        isequal(before.CurrentWaypointIndex, after.CurrentWaypointIndex) && ...
        isequal(before.Status, after.Status);

    if isfield(before, 'BirdPhase') && isfield(after, 'BirdPhase')
        tf = tf && before.BirdPhase == after.BirdPhase;
    end
    if isfield(before, 'BirdTreeZoneIndex') && isfield(after, 'BirdTreeZoneIndex')
        tf = tf && before.BirdTreeZoneIndex == after.BirdTreeZoneIndex;
    end
end

function target = createBirdTarget(environment)
    spawnPoint = environment.SpawnPoints.Bird(1, :);
    profile = TargetProfileRegistry.getProfile(TargetType.False);
    speed = 8;
    target = RadarTargetModel(TargetType.False, spawnPoint, 0, speed, 0.01);
    target = TargetFactory.initializeMotionContext(target, environment, profile);

    missionCommand = BirdMissionPlanner.createMission(target, environment);
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
