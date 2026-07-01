classdef SimulationEngine
    % SimulationEngine  Управление полной симуляцией радиолокационных целей.

    methods
        function result = run(~, config)
            arguments
                ~
                config (1, 1) struct
            end

            startTime = tic;
            SimulationEngine.validateConfig(config);
            config.BoxSize = config.BoxSize(:)';

            rng(config.RandomSeed);
            RadarTargetModel.resetIdCounter();

            environment = EnvironmentGenerator.generate( ...
                'BoxSize', config.BoxSize, ...
                'RandomSeed', config.RandomSeed, ...
                'MinAltitude', 0, ...
                'MaxAltitude', config.BoxSize(3), ...
                'SimulationTime', config.Duration, ...
                'TimeStep', config.Dt);

            decisionEngine = DecisionEngine();
            targets = SimulationEngine.createTargets(config, environment);
            outputFrames = SimulationEngine.captureOutputFrame(targets, 0);

            numSteps = round(config.Duration / config.Dt);
            for step = 1:numSteps
                for targetIdx = 1:numel(targets)
                    target = targets{targetIdx};
                    [target, missionCommand] = MissionPlanner.plan(target, environment, config.Dt);
                    [target, behaviorCommand] = BehaviorPlanner.plan( ...
                        target, environment, missionCommand, config.Dt);
                    [target, behaviorCommand] = NaturalMotionLayer.apply( ...
                        target, behaviorCommand, environment, config.Dt);
                    decision = decisionEngine.decide(target.toDecisionInput(), environment, behaviorCommand);
                    targets{targetIdx} = TrajectoryGenerator.updateMotion( ...
                        target, decision, behaviorCommand, environment, config.Dt);
                end

                currentTime = step * config.Dt;
                if SimulationEngine.isOutputTime(currentTime, config.OutputPeriod)
                    outputFrames = [outputFrames, SimulationEngine.captureOutputFrame(targets, currentTime)]; %#ok<AGROW>
                end
            end

            result = struct();
            result.Targets = targets;
            result.OutputFrames = outputFrames;
            result.Config = config;
            result.Environment = environment;
            result.Statistics = SimulationEngine.computeStatistics( ...
                targets, outputFrames, environment, toc(startTime));
        end
    end

    methods (Static, Access = private)
        function validateConfig(config)
            requiredFields = {
                'NumFalse'
                'NumGround'
                'NumAirplaneUAV'
                'NumQuadcopter'
                'BoxSize'
                'Duration'
                'Dt'
                'OutputPeriod'
                'RandomSeed'
            };

            for k = 1:numel(requiredFields)
                fieldName = requiredFields{k};
                if ~isfield(config, fieldName)
                    error('SimulationEngine:InvalidConfig', ...
                        'Config must contain field: %s', fieldName);
                end
            end

            if any(config.BoxSize <= 0)
                error('SimulationEngine:InvalidConfig', 'BoxSize values must be positive.');
            end

            if config.Duration <= 0 || config.Dt <= 0 || config.OutputPeriod <= 0
                error('SimulationEngine:InvalidConfig', ...
                    'Duration, Dt and OutputPeriod must be positive.');
            end
        end

        function targets = createTargets(config, environment)
            typePlan = [
                repmat(TargetType.False, config.NumFalse, 1)
                repmat(TargetType.Ground, config.NumGround, 1)
                repmat(TargetType.AirplaneUAV, config.NumAirplaneUAV, 1)
                repmat(TargetType.Quadcopter, config.NumQuadcopter, 1)
            ];

            targets = cell(numel(typePlan), 1);
            for k = 1:numel(typePlan)
                targets{k} = TargetFactory.createRandom(typePlan(k), environment);
            end
        end

        function tf = isOutputTime(currentTime, outputPeriod)
            tolerance = 1e-9;
            tf = abs(mod(currentTime, outputPeriod)) < tolerance || ...
                abs(mod(currentTime, outputPeriod) - outputPeriod) < tolerance;
        end

        function frame = captureOutputFrame(targets, currentTime)
            frame = struct();
            frame.Time = currentTime;
            frame.Targets = cell(numel(targets), 1);

            for k = 1:numel(targets)
                frame.Targets{k} = SimulationEngine.targetSnapshot(targets{k}, currentTime);
            end
        end

        function snapshot = targetSnapshot(target, currentTime)
            behaviorCommand = target.BehaviorCommand;
            desiredHeading = target.Heading;
            desiredSpeed = target.Speed;
            desiredAltitude = target.Position(3);
            behaviorReason = "";

            if isstruct(behaviorCommand) && BehaviorCommand.isActive(behaviorCommand)
                if isfinite(behaviorCommand.DesiredHeading)
                    desiredHeading = behaviorCommand.DesiredHeading;
                end
                if isfinite(behaviorCommand.DesiredSpeed)
                    desiredSpeed = behaviorCommand.DesiredSpeed;
                end
                if isfinite(behaviorCommand.DesiredAltitude)
                    desiredAltitude = behaviorCommand.DesiredAltitude;
                end
                behaviorReason = string(behaviorCommand.Reason);
            end

            snapshot = struct( ...
                'ID', target.ID, ...
                'Type', target.Type, ...
                'Position', target.Position, ...
                'Velocity', target.Velocity, ...
                'RCS', target.RCS, ...
                'CurrentState', target.CurrentState, ...
                'IsHidden', target.IsHidden, ...
                'BehaviorMode', target.BehaviorMode, ...
                'DesiredHeading', desiredHeading, ...
                'DesiredSpeed', desiredSpeed, ...
                'DesiredAltitude', desiredAltitude, ...
                'BehaviorReason', behaviorReason, ...
                'HeadingNoise', target.NaturalMotionState.HeadingNoise, ...
                'SpeedNoise', target.NaturalMotionState.SpeedNoise, ...
                'AltitudeNoise', target.NaturalMotionState.AltitudeNoise, ...
                'PositionNoiseNorm', norm(target.NaturalMotionState.PositionNoise(1:2)), ...
                'MissionType', target.MissionType, ...
                'MissionWaypointIndex', target.MissionWaypointIndex, ...
                'MissionReason', SimulationEngine.missionReason(target), ...
                'MissionStatus', SimulationEngine.missionField(target, 'Status'), ...
                'MissionProgress', SimulationEngine.missionField(target, 'Progress'), ...
                'MissionDistanceToWaypoint', SimulationEngine.missionField(target, 'DistanceToCurrentWaypoint'), ...
                'MissionCompletionReason', SimulationEngine.missionField(target, 'CompletionReason'), ...
                'MissionCancelReason', SimulationEngine.missionField(target, 'CancelReason'), ...
                'Time', currentTime);
        end

        function value = missionField(target, fieldName)
            if MissionCommand.isActive(target.MissionCommand) && ...
                    isfield(target.MissionCommand, fieldName)
                rawValue = target.MissionCommand.(fieldName);
                if isenum(rawValue)
                    value = string(rawValue);
                elseif isnumeric(rawValue)
                    value = rawValue;
                else
                    value = string(rawValue);
                end
            elseif strcmp(fieldName, 'Progress') || strcmp(fieldName, 'DistanceToCurrentWaypoint')
                value = 0;
            else
                value = "";
            end
        end

        function reason = missionReason(target)
            if MissionCommand.isActive(target.MissionCommand)
                reason = string(target.MissionCommand.MissionReason);
            else
                reason = "";
            end
        end

        function statistics = computeStatistics(targets, outputFrames, environment, executionTime)
            targetTypes = TargetType.allValues();
            statistics = struct();
            statistics.ExecutionTime = executionTime;
            statistics.HiddenStateCount = 0;
            statistics.CountByType = struct();
            statistics.AverageSpeedByType = struct();
            statistics.AverageAltitudeByType = struct();

            for k = 1:numel(targetTypes)
                typeName = matlab.lang.makeValidName(char(targetTypes(k)));
                typeTargets = targets(cellfun(@(t) t.Type == targetTypes(k), targets));

                statistics.CountByType.(typeName) = numel(typeTargets);

                if isempty(typeTargets)
                    statistics.AverageSpeedByType.(typeName) = 0;
                    statistics.AverageAltitudeByType.(typeName) = 0;
                    continue;
                end

                speeds = cellfun(@(t) t.Speed, typeTargets);
                altitudes = cellfun(@(t) t.Position(3), typeTargets);
                statistics.AverageSpeedByType.(typeName) = mean(speeds);
                statistics.AverageAltitudeByType.(typeName) = mean(altitudes);
            end

            for frameIdx = 1:numel(outputFrames)
                frameTargets = outputFrames(frameIdx).Targets;
                for targetIdx = 1:numel(frameTargets)
                    if frameTargets{targetIdx}.IsHidden
                        statistics.HiddenStateCount = statistics.HiddenStateCount + 1;
                    end
                end
            end

            statistics.Environment = struct( ...
                'XLimits', environment.XLimits, ...
                'YLimits', environment.YLimits, ...
                'ZLimits', environment.ZLimits);
        end
    end
end
