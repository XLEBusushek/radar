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
        HistoryWaypoint               % История waypoints, N×3
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
            obj.HistoryWaypoint = zeros(0, 3);

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

            modeChanged = isempty(obj.BehaviorCommand) || ...
                ~isfield(obj.BehaviorCommand, 'BehaviorMode') || ...
                command.BehaviorMode ~= obj.BehaviorMode || ...
                command.Priority > obj.BehaviorCommand.Priority;

            if modeChanged || ~obj.isBehaviorCommandActive()
                obj.LastWaypoint = obj.TargetWaypoint;
                if all(isfinite(command.DesiredPosition))
                    obj.TargetWaypoint = command.DesiredPosition;
                    obj.HistoryWaypoint = [obj.HistoryWaypoint; command.DesiredPosition];
                end
                obj.LastBehaviorChangeTime = obj.BehaviorTime;
                obj.DistanceSinceLastBehaviorChange = 0;
                obj.BehaviorTime = 0;
            end

            obj.BehaviorCommand = command;
            obj.BehaviorMode = command.BehaviorMode;
            obj.BehaviorHoldTime = command.HoldTime;
        end

        function obj = recordBehaviorDistance(obj, distance)
            obj.DistanceSinceLastBehaviorChange = obj.DistanceSinceLastBehaviorChange + distance;
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
