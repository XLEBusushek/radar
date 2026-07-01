% TestTrajectoryGenerator  Валидация TrajectoryGenerator (ТЗ №3).

function TestTrajectoryGenerator()
    rng(21);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    simDuration = 300;
    dt = 1;
    targetsPerType = 5;
    errors = 0;

    environment = SimulationEnvironment.create( ...
        [10000, 10000, 5000], 0, 5000, simDuration, dt);

    engine = DecisionEngine();
    targets = createTargetSet(environment, targetsPerType);
    numTargets = numel(targets);

    timeAxis = (0:simDuration) * dt;
    speedHistory = nan(numTargets, simDuration + 1);
    altitudeHistory = nan(numTargets, simDuration + 1);
    headingHistory = nan(numTargets, simDuration + 1);
    trajectoryHistory = nan(numTargets, simDuration + 1, 3);
    labels = strings(numTargets, 1);

    fprintf('=== Trajectory Generator Validation ===\n');
    fprintf('Targets: %d | Duration: %d s | dt: %d s\n\n', numTargets, simDuration, dt);

    for targetIdx = 1:numTargets
        target = targets{targetIdx};
        labels(targetIdx) = sprintf('ID %d (%s)', target.ID, target.Type);
        profile = TargetProfileRegistry.getProfile(target.Type);
        altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

        speedHistory(targetIdx, 1) = target.Speed;
        altitudeHistory(targetIdx, 1) = target.Position(3);
        headingHistory(targetIdx, 1) = target.Heading;
        trajectoryHistory(targetIdx, 1, :) = target.Position;

        for step = 1:simDuration
            prevHeading = target.Heading;
            prevSpeed = target.Speed;

            [target, missionCommand] = MissionPlanner.plan(target, environment, dt);
            [target, behaviorCommand] = BehaviorPlanner.plan(target, environment, missionCommand, dt);
            decision = engine.decide(target.toDecisionInput(), environment, behaviorCommand);
            target = TrajectoryGenerator.updateMotion(target, decision, behaviorCommand, environment, dt);

            errors = errors + validateTargetStep(target, profile, altitudeLimits, environment, ...
                prevHeading, prevSpeed, dt, step);

            speedHistory(targetIdx, step + 1) = target.Speed;
            altitudeHistory(targetIdx, step + 1) = target.Position(3);
            headingHistory(targetIdx, step + 1) = target.Heading;
            trajectoryHistory(targetIdx, step + 1, :) = target.Position;
        end

        targets{targetIdx} = target;
    end

    plotValidationResults(timeAxis, trajectoryHistory, speedHistory, altitudeHistory, headingHistory, labels);

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('Trajectory Generator Validation PASSED\n');
    else
        fprintf('Trajectory Generator Validation FAILED\n');
    end
end

function targets = createTargetSet(environment, targetsPerType)
    typePlan = [
        repmat(TargetType.False, targetsPerType, 1)
        repmat(TargetType.Ground, targetsPerType, 1)
        repmat(TargetType.AirplaneUAV, targetsPerType, 1)
        repmat(TargetType.Quadcopter, targetsPerType, 1)
    ];

    targets = cell(numel(typePlan), 1);
    for k = 1:numel(typePlan)
        targets{k} = TargetFactory.createRandom(typePlan(k), environment);
    end
end

function errors = validateTargetStep(target, profile, altitudeLimits, environment, prevHeading, prevSpeed, dt, step)
    errors = 0;
    tolerance = 1e-6;
    [speedMin, speedMax] = getSpeedLimits(profile, target.CurrentState);

    if target.Speed < speedMin - tolerance || target.Speed > speedMax + tolerance
        fprintf('ERROR: Target %d speed %.4f out of [%.4f, %.4f] at step %d.\n', ...
            target.ID, target.Speed, speedMin, speedMax, step);
        errors = errors + 1;
    end

    if target.Position(3) < altitudeLimits(1) - tolerance || target.Position(3) > altitudeLimits(2) + tolerance
        fprintf('ERROR: Target %d altitude %.4f out of [%.4f, %.4f] at step %d.\n', ...
            target.ID, target.Position(3), altitudeLimits(1), altitudeLimits(2), step);
        errors = errors + 1;
    end

    if any(isnan(target.Position)) || any(isinf(target.Position))
        fprintf('ERROR: Target %d invalid position at step %d.\n', target.ID, step);
        errors = errors + 1;
    end

    if target.Position(1) < environment.XLimits(1) - tolerance || target.Position(1) > environment.XLimits(2) + tolerance
        fprintf('ERROR: Target %d X out of bounds at step %d.\n', target.ID, step);
        errors = errors + 1;
    end

    if target.Position(2) < environment.YLimits(1) - tolerance || target.Position(2) > environment.YLimits(2) + tolerance
        fprintf('ERROR: Target %d Y out of bounds at step %d.\n', target.ID, step);
        errors = errors + 1;
    end

    if target.Position(3) < environment.ZLimits(1) - tolerance || target.Position(3) > environment.ZLimits(2) + tolerance
        fprintf('ERROR: Target %d Z out of environment bounds at step %d.\n', target.ID, step);
        errors = errors + 1;
    end

    headingDelta = abs(MotionKinematics.wrapAngle(target.Heading - prevHeading));
    maxHeadingStep = deg2rad(profile.MaxTurnRate) * dt + tolerance;
    if headingDelta > maxHeadingStep
        fprintf('ERROR: Target %d heading change %.4f deg exceeds limit at step %d.\n', ...
            target.ID, rad2deg(headingDelta), step);
        errors = errors + 1;
    end

    speedDelta = abs(target.Speed - prevSpeed);
    if target.Speed > prevSpeed
        maxSpeedStep = profile.MaxAcceleration * dt + tolerance;
    else
        maxSpeedStep = profile.MaxDeceleration * dt + tolerance;
    end
    if speedDelta > maxSpeedStep
        fprintf('ERROR: Target %d speed change %.4f exceeds limit at step %d.\n', ...
            target.ID, speedDelta, step);
        errors = errors + 1;
    end
end

function plotValidationResults(timeAxis, trajectoryHistory, speedHistory, altitudeHistory, headingHistory, labels)
    colors = lines(size(trajectoryHistory, 1));

    figure('Name', '3D Trajectories', 'NumberTitle', 'off');
    hold on;
    for k = 1:size(trajectoryHistory, 1)
        traj = squeeze(trajectoryHistory(k, :, :));
        plot3(traj(:, 1), traj(:, 2), traj(:, 3), 'Color', colors(k, :), 'DisplayName', labels(k));
    end
    grid on;
    xlabel('X, m');
    ylabel('Y, m');
    zlabel('Z, m');
    title('3D target trajectories');
    legend('Location', 'eastoutside');
    hold off;

    figure('Name', 'Speed vs Time', 'NumberTitle', 'off');
    plot(timeAxis, speedHistory', 'LineWidth', 1.0);
    grid on;
    xlabel('Time, s');
    ylabel('Speed, m/s');
    title('Speed(t)');

    figure('Name', 'Altitude vs Time', 'NumberTitle', 'off');
    plot(timeAxis, altitudeHistory', 'LineWidth', 1.0);
    grid on;
    xlabel('Time, s');
    ylabel('Altitude, m');
    title('Altitude(t)');

    figure('Name', 'Heading vs Time', 'NumberTitle', 'off');
    plot(timeAxis, rad2deg(headingHistory'), 'LineWidth', 1.0);
    grid on;
    xlabel('Time, s');
    ylabel('Heading, deg');
    title('Heading(t)');
end

function [speedMin, speedMax] = getSpeedLimits(profile, behaviorState)
    if behaviorState == TargetBehaviorState.Hover && profile.CanHover
        speedMin = profile.HoverSpeedMin;
        speedMax = profile.SpeedMax;
    elseif profile.CanHover && ~isnan(profile.CruiseSpeedMin)
        speedMin = profile.HoverSpeedMin;
        speedMax = profile.SpeedMax;
    else
        speedMin = TargetFactory.effectiveSpeedMin(profile, behaviorState);
        speedMax = profile.SpeedMax;
    end
end
