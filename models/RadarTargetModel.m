classdef RadarTargetModel
    % RadarTargetModel  Базовая кинематическая модель радиолокационной цели.
    %
    % На данном этапе класс описывает физику движения одной цели и хранение
    % её состояния. Логика выбора поведения подключается на следующих этапах.
    %
    % Пример:
    %   target = RadarTargetModel( ...
    %       TargetType.AirplaneUAV, [0 0 1000], 0, 50, 1.0);
    %   target = target.update(0.1);

    properties
        ID                % Уникальный идентификатор цели
        Type              % Тип цели (TargetType)
        Position (1, 3) double  % [x y z], м
        Velocity (1, 3) double  % [vx vy vz], м/с
        Heading double    % Азимут движения, рад
        Pitch double      % Угол набора высоты, рад
        Speed double      % Модуль скорости, м/с
        RCS double        % ЭПР, м²
        CurrentState      % Текущее состояние (TargetBehaviorState)
        StateTime double  % Время в текущем состоянии, с
        HistoryPosition   % История координат, N×3
        HistoryVelocity   % История скоростей, N×3
        HistoryHeading    % История курса
        HistoryPitch      % История тангажа
        HistorySpeed      % История модуля скорости
        HistoryState      % История состояний поведения
        BehaviorCoefficients  % Индивидуальные коэффициенты поведения
        IsHidden (1, 1) logical = false  % Признак скрытого состояния
        MotionContext                 % Дополнительное состояние модели движения
        BehaviorCommand               % Активная команда поведенческого слоя
        BehaviorMode                  % Текущий поведенческий режим
        BehaviorTime double = 0       % Время в текущем режиме, с
        BehaviorHoldTime double = 0   % Длительность удержания режима, с
        TargetWaypoint (1, 3) double = [nan, nan, nan]
        LastWaypoint (1, 3) double = [nan, nan, nan]
        LastBehaviorChangeTime double = 0
        DistanceSinceLastBehaviorChange double = 0
        BehaviorCommandStartTime double = 0
        BehaviorCommandStartPosition (1, 3) double = [nan, nan, nan]
        BehaviorCommandDistance double = 0
        HistoryWaypoint               % История waypoints, N×3
        MissionCommand                % Активная миссионная команда
        MissionType                   % Текущий тип миссии
        MissionTime double = 0        % Время в текущей миссии, с
        MissionStartTime double = 0   % Время начала миссии
        MissionRoute                    % Текущий маршрут миссии, N×3
        MissionWaypointIndex double = 1
        MissionHistory                % История миссий
        NaturalMotionState            % Состояние плавного шума Natural Motion
    end

    methods
        function obj = RadarTargetModel(targetType, initialPosition, initialHeading, speed, rcs, args)
            arguments
                targetType (1, 1) string
                initialPosition (1, 3) double
                initialHeading (1, 1) double
                speed (1, 1) double {mustBeNonnegative}
                rcs (1, 1) double {mustBeNonnegative}
                args.BehaviorCoefficients (1, 1) BehaviorCoefficients = BehaviorCoefficients.createRandom()
            end

            if ~TargetType.isValid(targetType)
                error('RadarTargetModel:InvalidType', ...
                    'Недопустимый тип цели: %s', targetType);
            end

            obj.ID = RadarTargetModel.generateID();
            obj.Type = targetType;
            obj.Position = initialPosition;
            obj.Heading = initialHeading;
            obj.Pitch = 0;
            obj.Speed = speed;
            obj.RCS = rcs;
            obj.BehaviorCoefficients = args.BehaviorCoefficients;
            obj.CurrentState = TargetBehaviorState.FlyStraight;
            obj.StateTime = 0;
            obj.HistoryPosition = zeros(0, 3);
            obj.HistoryVelocity = zeros(0, 3);
            obj.HistoryHeading = zeros(0, 1);
            obj.HistoryPitch = zeros(0, 1);
            obj.HistorySpeed = zeros(0, 1);
            obj.HistoryState = TargetBehaviorState.empty(0, 1);
            obj.IsHidden = false;
            obj.MotionContext = struct();
            obj.BehaviorCommand = BehaviorCommand.empty();
            obj.BehaviorMode = BehaviorMode.Cruise;
            obj.BehaviorTime = 0;
            obj.BehaviorHoldTime = 0;
            obj.TargetWaypoint = [nan, nan, nan];
            obj.LastWaypoint = [nan, nan, nan];
            obj.LastBehaviorChangeTime = 0;
            obj.DistanceSinceLastBehaviorChange = 0;
            obj.BehaviorCommandStartTime = 0;
            obj.BehaviorCommandStartPosition = [nan, nan, nan];
            obj.BehaviorCommandDistance = 0;
            obj.HistoryWaypoint = zeros(0, 3);
            obj.MissionCommand = MissionCommand.empty();
            obj.MissionType = MissionType.Idle;
            obj.MissionTime = 0;
            obj.MissionStartTime = 0;
            obj.MissionRoute = zeros(0, 3);
            obj.MissionWaypointIndex = 1;
            obj.MissionHistory = RadarTargetModel.emptyMissionHistoryRow();
            obj.NaturalMotionState = RadarTargetModel.emptyNaturalMotionState();

            obj = obj.updateVelocityFromKinematics();
            obj = obj.saveHistory();
        end

        function obj = update(obj, dt)
            arguments
                obj (1, 1) RadarTargetModel
                dt (1, 1) double {mustBePositive}
            end

            obj = obj.updateVelocityFromKinematics();
            obj.Position = obj.Position + obj.Velocity * dt;
            obj.StateTime = obj.StateTime + dt;
            obj = obj.saveHistory();
        end

        function obj = saveHistory(obj)
            obj.HistoryPosition = [obj.HistoryPosition; obj.Position];
            obj.HistoryVelocity = [obj.HistoryVelocity; obj.Velocity];
            obj.HistoryHeading = [obj.HistoryHeading; obj.Heading];
            obj.HistoryPitch = [obj.HistoryPitch; obj.Pitch];
            obj.HistorySpeed = [obj.HistorySpeed; obj.Speed];
            obj.HistoryState = [obj.HistoryState; obj.CurrentState];
        end

        function state = getState(obj)
            state = struct( ...
                'ID', obj.ID, ...
                'Type', obj.Type, ...
                'Position', obj.Position, ...
                'Velocity', obj.Velocity, ...
                'Heading', obj.Heading, ...
                'Pitch', obj.Pitch, ...
                'Speed', obj.Speed, ...
                'RCS', obj.RCS, ...
                'CurrentState', obj.CurrentState, ...
                'StateTime', obj.StateTime, ...
                'BehaviorCoefficients', obj.BehaviorCoefficients, ...
                'IsHidden', obj.IsHidden);
        end

        function input = toDecisionInput(obj)
            input = struct( ...
                'Type', obj.Type, ...
                'Position', obj.Position, ...
                'Velocity', obj.Velocity, ...
                'Speed', obj.Speed, ...
                'Heading', obj.Heading, ...
                'Pitch', obj.Pitch, ...
                'CurrentState', obj.CurrentState, ...
                'StateTime', obj.StateTime, ...
                'BehaviorCoefficients', obj.BehaviorCoefficients);
        end

        function obj = applyDecision(obj, decision)
            arguments
                obj (1, 1) RadarTargetModel
                decision (1, 1) struct
            end

            if decision.NextState ~= obj.CurrentState
                obj.CurrentState = decision.NextState;
                obj.StateTime = 0;
            end
        end

        function obj = tickBehaviorTime(obj, dt)
            obj.BehaviorTime = obj.BehaviorTime + dt;
        end

        function tf = isBehaviorCommandActive(obj)
            tf = obj.BehaviorHoldTime > 0 && obj.BehaviorTime < obj.BehaviorHoldTime;
        end

        function obj = setBehaviorCommand(obj, command)
            if ~BehaviorCommand.isActive(command)
                return;
            end

            if obj.isBehaviorCommandActive() && ...
                    command.BehaviorMode == obj.BehaviorMode && ...
                    strcmp(command.Reason, obj.BehaviorCommand.Reason)
                if command.Priority > obj.BehaviorCommand.Priority
                    obj.BehaviorCommand.Priority = command.Priority;
                end
                if all(isfinite(command.DesiredPosition))
                    obj.BehaviorCommand.DesiredPosition = command.DesiredPosition;
                end
                if isfinite(command.DesiredHeading)
                    obj.BehaviorCommand.DesiredHeading = command.DesiredHeading;
                end
                return;
            end

            obj.LastWaypoint = obj.TargetWaypoint;
            if all(isfinite(command.DesiredPosition))
                obj.TargetWaypoint = command.DesiredPosition;
                obj.HistoryWaypoint = [obj.HistoryWaypoint; command.DesiredPosition];
            end
            obj.LastBehaviorChangeTime = obj.BehaviorTime;
            obj.DistanceSinceLastBehaviorChange = 0;
            obj.BehaviorCommandStartTime = obj.BehaviorTime;
            obj.BehaviorCommandStartPosition = obj.Position;
            obj.BehaviorCommandDistance = 0;
            obj.BehaviorTime = 0;

            if isstruct(obj.MotionContext)
                obj.MotionContext.SmoothedDesiredSpeed = obj.Speed;
                if isfinite(command.DesiredSpeed)
                    obj.MotionContext.CommandDesiredSpeed = command.DesiredSpeed;
                end
            end

            obj.BehaviorCommand = command;
            obj.BehaviorMode = command.BehaviorMode;
            obj.BehaviorHoldTime = command.HoldTime;
        end

        function obj = recordBehaviorDistance(obj, distance)
            obj.DistanceSinceLastBehaviorChange = obj.DistanceSinceLastBehaviorChange + distance;
            obj.BehaviorCommandDistance = obj.BehaviorCommandDistance + distance;
        end

        function obj = tickMissionTime(obj, dt)
            obj.MissionTime = obj.MissionTime + dt;
        end

        function tf = isMissionActive(obj)
            if ~MissionCommand.isActive(obj.MissionCommand)
                tf = false;
                return;
            end

            if MissionStateMachine.isTerminal(obj.MissionCommand)
                tf = obj.MissionTime < obj.MissionCommand.MissionHoldTime;
                return;
            end

            tf = true;
        end

        function obj = setMissionCommand(obj, command)
            if ~MissionCommand.isActive(command) && command.MissionHoldTime <= 0
                return;
            end

            isNewMission = ~MissionCommand.isActive(obj.MissionCommand) || ...
                command.MissionPriority > obj.MissionCommand.MissionPriority || ...
                (MissionStateMachine.isTerminal(obj.MissionCommand) && ...
                obj.MissionTime >= obj.MissionCommand.MissionHoldTime) || ...
                (command.MissionType ~= obj.MissionCommand.MissionType && ...
                command.MissionPriority >= obj.MissionCommand.MissionPriority);

            if isNewMission
                obj.MissionTime = 0;
                obj.MissionStartTime = 0;
                obj.MissionWaypointIndex = command.CurrentWaypointIndex;
                command.Status = MissionStatus.Created;
                command.PreviousStatus = MissionStatus.Created;
                command.StatusStartTime = 0;
                command.StatusTime = 0;
                obj = obj.appendMissionHistory(command);
            end

            obj.MissionCommand = command;
            obj.MissionType = command.MissionType;
            obj.MissionRoute = command.MissionRoute;
            obj.MissionWaypointIndex = command.CurrentWaypointIndex;

            if obj.Type == TargetType.Ground && isfield(command, 'GroundLaneOffset')
                obj.MotionContext.LaneOffset = command.GroundLaneOffset;
                obj.MotionContext.CurrentRoadSegmentIndex = command.CurrentRoadSegmentIndex;
                obj.MotionContext.DestinationNodeIndex = command.DestinationNodeIndex;
                obj.MotionContext.GroundPhase = 'Drive';
                obj.MotionContext.DistanceOnRoad = 0;
                if isfield(command, 'RoadNodePath')
                    obj.MotionContext.RoadNodePath = command.RoadNodePath;
                end
                if size(command.MissionRoute, 1) >= 2
                    routeDelta = command.MissionRoute(2, 1:2) - command.MissionRoute(1, 1:2);
                    if norm(routeDelta) > 1
                        routeHeading = atan2(routeDelta(2), routeDelta(1));
                        obj.MotionContext.RoadHeading = routeHeading;
                    end
                end
            end

            if obj.Type == TargetType.AirplaneUAV && command.MissionType == MissionType.PatrolRoute
                if size(command.MissionRoute, 1) >= 1
                    routeDelta = command.MissionRoute(1, 1:2) - obj.Position(1:2);
                    if norm(routeDelta) > 1
                        routeHeading = atan2(routeDelta(2), routeDelta(1));
                        obj.MotionContext.RoadHeading = routeHeading;
                    end
                end
                if isfield(command, 'TurnStartDistance')
                    obj.MotionContext.TurnStartDistance = command.TurnStartDistance;
                end
            end
        end

        function obj = updateMissionCommand(obj, command)
            obj.MissionCommand = command;
            obj.MissionWaypointIndex = command.CurrentWaypointIndex;
            if command.CurrentWaypointIndex <= size(command.MissionRoute, 1)
                obj.MissionRoute = command.MissionRoute;
            end
        end

        function obj = appendMissionHistory(obj, command)
            row = struct( ...
                'Time', obj.MissionTime, ...
                'MissionType', command.MissionType, ...
                'Status', command.Status, ...
                'CurrentWaypointIndex', command.CurrentWaypointIndex, ...
                'Progress', command.Progress, ...
                'DistanceToCurrentWaypoint', command.DistanceToCurrentWaypoint, ...
                'CompletionReason', string(command.CompletionReason), ...
                'CancelReason', string(command.CancelReason), ...
                'MissionReason', string(command.MissionReason));
            obj.MissionHistory = [obj.MissionHistory; row];
        end
    end

    methods (Static)
        function resetIdCounter()
            RadarTargetModel.manageIdCounter('reset');
        end
    end

    methods (Access = private)
        function obj = updateVelocityFromKinematics(obj)
            obj.Velocity = [
                obj.Speed * cos(obj.Pitch) * cos(obj.Heading), ...
                obj.Speed * cos(obj.Pitch) * sin(obj.Heading), ...
                obj.Speed * sin(obj.Pitch)
            ];
        end
    end

    methods (Static, Access = private)
        function row = emptyMissionHistoryRow()
            row = struct( ...
                'Time', {}, ...
                'MissionType', {}, ...
                'Status', {}, ...
                'CurrentWaypointIndex', {}, ...
                'Progress', {}, ...
                'DistanceToCurrentWaypoint', {}, ...
                'CompletionReason', {}, ...
                'CancelReason', {}, ...
                'MissionReason', {});
        end

        function state = emptyNaturalMotionState()
            state = struct( ...
                'HeadingNoise', 0, ...
                'SpeedNoise', 0, ...
                'AltitudeNoise', 0, ...
                'PositionNoise', [0, 0, 0], ...
                'LaneOffsetNoise', 0, ...
                'RoadHeightNoise', 0, ...
                'HoverDrift', [0, 0], ...
                'WindDrift', [0, 0], ...
                'LastUpdateTime', 0, ...
                'NoiseStream', []);
        end

        function id = generateID()
            id = RadarTargetModel.manageIdCounter('next');
        end

        function id = manageIdCounter(action)
            persistent nextID;

            if isempty(nextID)
                nextID = 1;
            end

            if nargin < 1 || strcmp(action, 'next')
                id = nextID;
                nextID = nextID + 1;
                return;
            end

            if strcmp(action, 'reset')
                nextID = 1;
                id = 1;
            end
        end
    end
end
