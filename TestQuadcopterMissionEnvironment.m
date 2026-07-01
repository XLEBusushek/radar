% TestQuadcopterMissionEnvironment  Валидация миссий Quadcopter по Environment (ТЗ №13.6).

function TestQuadcopterMissionEnvironment()
    rng(83);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();
    RadarTargetModel.resetIdCounter();

    fprintf('=== Quadcopter Mission Environment Validation ===\n\n');

    dt = 1;
    simDuration = 360;
    environment = EnvironmentGenerator.generate( ...
        'BoxSize', [8000, 8000, 500], ...
        'RandomSeed', 83, ...
        'MinAltitude', 0, ...
        'MaxAltitude', 500, ...
        'SimulationTime', simDuration, ...
        'TimeStep', dt);

    errors = 0;
    errors = errors + validateMissionRoute(environment);
    [target, errorsAfterSim, simRecords] = runQuadcopterSimulation(environment, simDuration, dt);
    errors = errors + errorsAfterSim;
    errors = errors + validateMissionCompletion(target, simRecords);
    errors = errors + validateNaturalMotion(simRecords);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Quadcopter Mission Environment Validation PASSED\n');
    else
        fprintf('Quadcopter Mission Environment Validation FAILED\n');
    end
end

function errors = validateMissionRoute(environment)
    errors = 0;
    spawnPoint = environment.SpawnPoints.Quadcopter(1, :);
    targetStub = struct( ...
        'ID', 1, ...
        'Position', spawnPoint, ...
        'Type', TargetType.Quadcopter, ...
        'MissionTime', 0);
    command = QuadcopterMissionPlanner.createMission(targetStub, environment);
    route = command.MissionRoute;

    if ~isfield(command, 'InspectionZoneIndex') || command.InspectionZoneIndex < 1
        fprintf('ERROR: Mission route is not bound to an InspectionZone.\n');
        errors = errors + 1;
    end

    if command.MissionType ~= MissionType.InspectArea
        fprintf('ERROR: Quadcopter mission type is not InspectArea.\n');
        errors = errors + 1;
    end

    waypointCount = size(route, 1);
    if waypointCount < 3 || waypointCount > 6
        fprintf('ERROR: Inspection route has %d waypoints, expected 3..6.\n', waypointCount);
        errors = errors + 1;
    end

    if command.DesiredMissionSpeed < 5 || command.DesiredMissionSpeed > 12
        fprintf('ERROR: Mission speed %.2f outside 5..12 m/s.\n', command.DesiredMissionSpeed);
        errors = errors + 1;
    end

    minSegment = InspectionRouteBuilder.minSegmentLength(route);
    maxSegment = InspectionRouteBuilder.maxSegmentLength(route);
    if minSegment < 29.5
        fprintf('ERROR: Inspection route min segment %.1f m is below 30 m.\n', minSegment);
        errors = errors + 1;
    end
    if maxSegment > 150.5
        fprintf('ERROR: Inspection route max segment %.1f m exceeds 150 m.\n', maxSegment);
        errors = errors + 1;
    end

    if any(route(:, 3) < 39.5 | route(:, 3) > 120.5)
        fprintf('ERROR: Route altitude is outside 40..120 m.\n');
        errors = errors + 1;
    end

    if ~isfield(command, 'InspectionPhase') || ...
            command.InspectionPhase ~= InspectionPhase.MoveToPoint
        fprintf('ERROR: Inspection mission does not start in MoveToPoint phase.\n');
        errors = errors + 1;
    end
end

function [target, errors, records] = runQuadcopterSimulation(environment, simDuration, dt)
    errors = 0;
    RadarTargetModel.resetIdCounter();
    target = createQuadcopterTarget(environment);
    engine = DecisionEngine();

    records = struct( ...
        'HoverObserveSeen', false, ...
        'HoverEpisodes', struct('Speeds', {}, 'Altitudes', {}, 'WaypointIndex', {}, ...
            'Positions', {}, 'Headings', {}, 'Anchors', {}, 'DriftDistances', {}), ...
        'CurrentHoverSpeeds', [], ...
        'CurrentHoverAltitudes', [], ...
        'CurrentHoverPositions', [], ...
        'CurrentHoverHeadings', [], ...
        'CurrentHoverDrifts', [], ...
        'CurrentHoverAnchor', [nan, nan, nan], ...
        'CurrentHoverWaypoint', 0, ...
        'CurrentHoverStep', 0, ...
        'MissionCompleted', false, ...
        'NewMissionCreated', false, ...
        'InitialMissionZone', 0, ...
        'PostCompletionZone', 0, ...
        'LastReachedCount', 0, ...
        'LastTotalCount', 0, ...
        'LastWaypointIndex', 0, ...
        'WaypointChangesDuringHover', 0, ...
        'HoverToMoveTransitions', 0, ...
        'InitialMissionRoute', [], ...
        'MissionRouteChanged', false, ...
        'MissionCommandChangedByMotion', false, ...
        'MaxHoverDrift', 0, ...
        'MaxHoverPositionError', 0, ...
        'MaxHeadingNoise', 0, ...
        'MaxAltitudeNoise', 0);

    records.InitialMissionZone = 0;
    previousPhase = InspectionPhase.MoveToPoint;
    previousWaypointIndex = 1;
    previousReachedCount = 0;
    previousTotalCount = 0;
    previousWaypointIndex = 0;
    missionCount = 0;

    for step = 1:simDuration
        records.LastReachedCount = 0;
        records.LastTotalCount = 0;
        records.LastWaypointIndex = 0;
        if step > 1
            records.LastReachedCount = previousReachedCount;
            records.LastTotalCount = previousTotalCount;
            records.LastWaypointIndex = previousWaypointIndex;
        end

        [target, missionCommand] = MissionPlanner.plan(target, environment, dt);

        if records.InitialMissionZone < 1 && isfield(missionCommand, 'InspectionZoneIndex')
            records.InitialMissionZone = missionCommand.InspectionZoneIndex;
            records.InitialMissionRoute = target.MissionRoute;
            previousPhase = missionCommand.InspectionPhase;
            previousWaypointIndex = missionCommand.CurrentWaypointIndex;
        end

        missionCommandBeforeMotion = target.MissionCommand;
        [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);
        [target, behaviorCommand] = NaturalMotionLayer.apply(target, behaviorCommand, environment, dt);

        if ~missionSnapshotEqual(missionCommandBeforeMotion, target.MissionCommand)
            records.MissionCommandChangedByMotion = true;
        end
        if ~isequal(missionCommandBeforeMotion.MissionRoute, target.MissionCommand.MissionRoute)
            records.MissionRouteChanged = true;
        end

        state = target.NaturalMotionState;
        records.MaxHeadingNoise = max(records.MaxHeadingNoise, abs(state.HeadingNoise));
        records.MaxAltitudeNoise = max(records.MaxAltitudeNoise, abs(state.AltitudeNoise));

        decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
        target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

        if missionCommand.IsMissionComplete || missionCommand.Status == MissionStatus.Completed
            records.MissionCompleted = true;
        end

        if missionCommand.ReachedWaypointCount >= missionCommand.TotalWaypointCount
            records.MissionCompleted = true;
        end

        if numel(target.MissionHistory) > missionCount
            missionCount = numel(target.MissionHistory);
            if records.LastWaypointIndex >= records.LastTotalCount && records.LastTotalCount > 0
                records.MissionCompleted = true;
            end
            if missionCount > 1 && isfield(target.MissionCommand, 'InspectionZoneIndex')
                records.NewMissionCreated = true;
                records.PostCompletionZone = target.MissionCommand.InspectionZoneIndex;
            end
        end

        previousReachedCount = missionCommand.ReachedWaypointCount;
        previousTotalCount = missionCommand.TotalWaypointCount;
        previousWaypointIndex = missionCommand.CurrentWaypointIndex;

        if missionCommand.InspectionPhase == InspectionPhase.HoverObserve
            records.HoverObserveSeen = true;

            if missionCommand.CurrentWaypointIndex ~= records.CurrentHoverWaypoint
                records = finalizeHoverEpisode(records);
                records.CurrentHoverWaypoint = missionCommand.CurrentWaypointIndex;
                records.CurrentHoverStep = 0;
                if isfield(missionCommand, 'InspectionHoverPosition') && ...
                        all(isfinite(missionCommand.InspectionHoverPosition))
                    records.CurrentHoverAnchor = missionCommand.InspectionHoverPosition;
                else
                    records.CurrentHoverAnchor = target.Position;
                end
            end

            records.CurrentHoverStep = records.CurrentHoverStep + 1;
            records.CurrentHoverSpeeds(end + 1) = target.Speed; %#ok<AGROW>
            records.CurrentHoverAltitudes(end + 1) = target.Position(3); %#ok<AGROW>
            records.CurrentHoverPositions(end + 1, :) = target.Position; %#ok<AGROW>
            records.CurrentHoverHeadings(end + 1) = target.Heading; %#ok<AGROW>
            records.CurrentHoverDrifts(end + 1, :) = state.HoverDrift(1:2); %#ok<AGROW>

            driftDistance = norm(state.HoverDrift(1:2));
            records.MaxHoverDrift = max(records.MaxHoverDrift, driftDistance);
            if records.CurrentHoverStep >= 3
                hoverDriftXY = norm(target.Position(1:2) - records.CurrentHoverAnchor(1:2));
                records.MaxHoverPositionError = max(records.MaxHoverPositionError, hoverDriftXY);
            end

            if missionCommand.CurrentWaypointIndex ~= previousWaypointIndex
                records.WaypointChangesDuringHover = records.WaypointChangesDuringHover + 1;
            end

            if behaviorCommand.BehaviorMode ~= BehaviorMode.HoverObserve && ...
                    contains(behaviorCommand.Reason, 'Mission:')
                fprintf('ERROR: HoverObserve phase without HoverObserve behavior mode.\n');
                errors = errors + 1;
            end
        elseif records.CurrentHoverWaypoint > 0
            records = finalizeHoverEpisode(records);
            records.CurrentHoverWaypoint = 0;
            records.CurrentHoverStep = 0;
        end

        if previousPhase == InspectionPhase.HoverObserve && ...
                missionCommand.InspectionPhase == InspectionPhase.MoveToPoint
            records.HoverToMoveTransitions = records.HoverToMoveTransitions + 1;
        end

        previousPhase = missionCommand.InspectionPhase;
        previousWaypointIndex = missionCommand.CurrentWaypointIndex;
    end

    records = finalizeHoverEpisode(records);

    if ~records.HoverObserveSeen
        fprintf('ERROR: HoverObserve phase was not observed.\n');
        errors = errors + 1;
    end

    if ~isempty(records.HoverEpisodes)
        for episodeIdx = 1:numel(records.HoverEpisodes)
            episode = records.HoverEpisodes(episodeIdx);
            if numel(episode.Speeds) <= 2
                continue;
            end
            steadySpeeds = episode.Speeds(3:end);
            if any(steadySpeeds > 1.2)
                fprintf('ERROR: HoverObserve steady speed exceeded 1 m/s (max %.2f).\n', max(steadySpeeds));
                errors = errors + 1;
            end

            hoverAltitudeRange = max(episode.Altitudes) - min(episode.Altitudes);
            if hoverAltitudeRange > 2.5
                fprintf('ERROR: HoverObserve altitude varied by %.2f m at waypoint %d.\n', ...
                    hoverAltitudeRange, episode.WaypointIndex);
                errors = errors + 1;
            end

            if isfield(episode, 'Altitudes') && numel(episode.Altitudes) > 2
                altitudeSteps = abs(diff(episode.Altitudes(3:end)));
                if max(altitudeSteps) > 0.6
                    fprintf('ERROR: HoverObserve altitude step %.2f m is too abrupt.\n', ...
                        max(altitudeSteps));
                    errors = errors + 1;
                end
            end
        end
    end

    if records.WaypointChangesDuringHover > 0
        fprintf('ERROR: Waypoint index changed during HoverObserve.\n');
        errors = errors + 1;
    end

    if records.HoverToMoveTransitions == 0
        fprintf('ERROR: No transition from HoverObserve to next point.\n');
        errors = errors + 1;
    end
end

function errors = validateNaturalMotion(records)
    errors = 0;
    motionProfile = NaturalMotionProfileRegistry.getProfile(TargetType.Quadcopter);

    if records.MissionRouteChanged
        fprintf('ERROR: Mission route changed during natural motion.\n');
        errors = errors + 1;
    end

    if records.MissionCommandChangedByMotion
        fprintf('ERROR: MissionCommand changed during natural motion layer.\n');
        errors = errors + 1;
    end

    if records.MaxHoverDrift < 0.05
        fprintf('ERROR: Hover drift was not observed during HoverObserve.\n');
        errors = errors + 1;
    end

    if records.MaxHoverDrift > motionProfile.MaxHoverDrift + 1e-6
        fprintf('ERROR: Hover drift %.2f m exceeds %.2f m limit.\n', ...
            records.MaxHoverDrift, motionProfile.MaxHoverDrift);
        errors = errors + 1;
    end

    if isfield(records, 'MaxHoverPositionError') && ...
            records.MaxHoverPositionError > motionProfile.MaxHoverDrift + 0.35
        fprintf('ERROR: Hover position drift %.2f m exceeds %.2f m limit.\n', ...
            records.MaxHoverPositionError, motionProfile.MaxHoverDrift);
        errors = errors + 1;
    end

    if records.MaxHeadingNoise > motionProfile.MaxHeadingNoise + 1e-6
        fprintf('ERROR: Heading noise %.4f rad exceeds %.4f rad limit.\n', ...
            records.MaxHeadingNoise, motionProfile.MaxHeadingNoise);
        errors = errors + 1;
    end

    if records.MaxAltitudeNoise > motionProfile.MaxHoverAltitudeNoise + 1e-6
        fprintf('ERROR: Altitude noise %.4f m exceeds %.4f m limit.\n', ...
            records.MaxAltitudeNoise, motionProfile.MaxHoverAltitudeNoise);
        errors = errors + 1;
    end

    if isempty(records.HoverEpisodes)
        fprintf('ERROR: No hover episodes recorded for natural motion validation.\n');
        errors = errors + 1;
        return;
    end

    headingVariationSeen = false;
    driftVariationSeen = false;
    for episodeIdx = 1:numel(records.HoverEpisodes)
        episode = records.HoverEpisodes(episodeIdx);
        if isfield(episode, 'Headings') && numel(episode.Headings) > 2
            headingSpan = max(abs(MotionKinematics.wrapAngle(diff(episode.Headings(3:end)))));
            if headingSpan > deg2rad(0.3)
                headingVariationSeen = true;
            end
        end
        if isfield(episode, 'DriftDistances') && any(episode.DriftDistances > 0.05)
            driftVariationSeen = true;
        end
    end

    if ~headingVariationSeen
        fprintf('ERROR: Hover heading did not show natural oscillation.\n');
        errors = errors + 1;
    end

    if ~driftVariationSeen
        fprintf('ERROR: Hover drift variation was not observed.\n');
        errors = errors + 1;
    end
end

function errors = validateMissionCompletion(target, records)
    errors = 0;

    if ~records.MissionCompleted
        fprintf('ERROR: Quadcopter inspection mission did not complete.\n');
        errors = errors + 1;
    end

    if ~records.NewMissionCreated
        fprintf('ERROR: No new inspection mission was created after completion.\n');
        errors = errors + 1;
    end

    if records.NewMissionCreated && records.PostCompletionZone < 1
        fprintf('ERROR: New mission is not bound to an InspectionZone.\n');
        errors = errors + 1;
    end

    if target.MissionCommand.MissionType ~= MissionType.InspectArea
        fprintf('ERROR: Active mission after completion is not InspectArea.\n');
        errors = errors + 1;
    end
end

function records = finalizeHoverEpisode(records)
    if records.CurrentHoverWaypoint < 1 || isempty(records.CurrentHoverSpeeds)
        records.CurrentHoverSpeeds = [];
        records.CurrentHoverAltitudes = [];
        records.CurrentHoverPositions = [];
        records.CurrentHoverHeadings = [];
        records.CurrentHoverDrifts = [];
        return;
    end

    driftDistances = vecnorm(records.CurrentHoverDrifts, 2, 2);

    episode = struct( ...
        'Speeds', records.CurrentHoverSpeeds, ...
        'Altitudes', records.CurrentHoverAltitudes, ...
        'WaypointIndex', records.CurrentHoverWaypoint, ...
        'Positions', records.CurrentHoverPositions, ...
        'Headings', records.CurrentHoverHeadings, ...
        'Anchors', records.CurrentHoverAnchor, ...
        'DriftDistances', driftDistances);
    records.HoverEpisodes(end + 1) = episode; %#ok<AGROW>
    records.CurrentHoverSpeeds = [];
    records.CurrentHoverAltitudes = [];
    records.CurrentHoverPositions = [];
    records.CurrentHoverHeadings = [];
    records.CurrentHoverDrifts = [];
end

function tf = missionSnapshotEqual(before, after)
    tf = isequal(before.MissionType, after.MissionType) && ...
        isequal(before.MissionRoute, after.MissionRoute) && ...
        isequal(before.MissionReason, after.MissionReason) && ...
        isequal(before.CurrentWaypointIndex, after.CurrentWaypointIndex) && ...
        isequal(before.Status, after.Status);

    if isfield(before, 'InspectionZoneIndex') && isfield(after, 'InspectionZoneIndex')
        tf = tf && before.InspectionZoneIndex == after.InspectionZoneIndex;
    end
    if isfield(before, 'InspectionPhase') && isfield(after, 'InspectionPhase')
        tf = tf && before.InspectionPhase == after.InspectionPhase;
    end
end

function target = createQuadcopterTarget(environment)
    spawnPoint = environment.SpawnPoints.Quadcopter(1, :);
    profile = TargetProfileRegistry.getProfile(TargetType.Quadcopter);
    speed = 6;
    target = RadarTargetModel(TargetType.Quadcopter, spawnPoint, 0, speed, 0.08);
    target = TargetFactory.initializeMotionContext(target, environment, profile);

    missionCommand = QuadcopterMissionPlanner.createMission(target, environment);
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
