% TestTargetProfiles  Валидация TargetProfileRegistry и TargetFactory (ТЗ №2.2).

function TestTargetProfiles()
    rng(7);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    errors = 0;

    environment = SimulationEnvironment.create( ...
        [10000, 10000, 5000], 0, 5000, 600, 1);

    targetTypes = TargetType.allValues();

    fprintf('=== Target Profile Validation ===\n\n');

    for k = 1:numel(targetTypes)
        targetType = targetTypes(k);
        profile = TargetProfileRegistry.getProfile(targetType);

        errors = errors + assertProfileStructure(profile, targetType);
        errors = errors + assertProfileRanges(profile, targetType);
        errors = errors + assertCapabilityConsistency(profile, targetType);
    end

    sampleCount = 1000;
    fprintf('Checking %d random targets...\n', sampleCount);

    for sampleIdx = 1:sampleCount
        targetType = targetTypes(randi(numel(targetTypes)));
        target = TargetFactory.createRandom(targetType, environment);
        profile = TargetProfileRegistry.getProfile(targetType);
        altitudeLimits = TargetFactory.resolveAltitudeLimits(profile, environment);

        if target.Speed < profile.SpeedMin || target.Speed > profile.SpeedMax
            fprintf('ERROR: Speed out of range for %s (%.4f).\n', targetType, target.Speed);
            errors = errors + 1;
        end

        if target.RCS < profile.RCSMin || target.RCS > profile.RCSMax
            fprintf('ERROR: RCS out of range for %s (%.6f).\n', targetType, target.RCS);
            errors = errors + 1;
        end

        if target.Position(3) < altitudeLimits(1) || target.Position(3) > altitudeLimits(2)
            fprintf('ERROR: Altitude out of range for %s (%.4f).\n', targetType, target.Position(3));
            errors = errors + 1;
        end
    end

    fprintf('\nErrors: %d\n', errors);

    if errors == 0
        fprintf('Target Profile Validation PASSED\n');
    else
        fprintf('Target Profile Validation FAILED\n');
    end
end

function errors = assertProfileStructure(profile, targetType)
    errors = 0;
    requiredFields = {
        'SpeedMin'
        'SpeedMax'
        'RCSMin'
        'RCSMax'
        'AltitudeMin'
        'AltitudeMax'
        'HoverSpeedMin'
        'CruiseSpeedMin'
        'MaxTurnRate'
        'MaxPitchRate'
        'MaxAcceleration'
        'CanHover'
        'CanClimb'
        'CanDescend'
    };

    for k = 1:numel(requiredFields)
        if ~isprop(profile, requiredFields{k})
            fprintf('ERROR: Profile for %s missing field %s.\n', targetType, requiredFields{k});
            errors = errors + 1;
        end
    end

    if errors == 0
        fprintf('Profile structure valid for %s.\n', targetType);
    end
end

function errors = assertProfileRanges(profile, targetType)
    errors = 0;

    if ~(profile.SpeedMin < profile.SpeedMax)
        fprintf('ERROR: Speed range invalid for %s.\n', targetType);
        errors = errors + 1;
    end

    if ~(profile.RCSMin < profile.RCSMax)
        fprintf('ERROR: RCS range invalid for %s.\n', targetType);
        errors = errors + 1;
    end

    if ~(profile.AltitudeMin <= profile.AltitudeMax)
        fprintf('ERROR: Altitude range invalid for %s.\n', targetType);
        errors = errors + 1;
    end

    if errors == 0
        fprintf('Profile ranges valid for %s.\n', targetType);
    end
end

function errors = assertCapabilityConsistency(profile, targetType)
    errors = 0;

    switch char(targetType)
        case char(TargetType.False)
            expected = [false, true, true];
        case char(TargetType.Ground)
            expected = [false, false, false];
        case char(TargetType.AirplaneUAV)
            expected = [false, true, true];
        case char(TargetType.Quadcopter)
            expected = [true, true, true];
        otherwise
            expected = [];
    end

    actual = [profile.CanHover, profile.CanClimb, profile.CanDescend];
    if ~isequal(actual, expected)
        fprintf('ERROR: Capability flags mismatch for %s.\n', targetType);
        errors = errors + 1;
    end
end
