classdef DecisionEngine
    % DecisionEngine  Модуль принятия решений о поведении радиолокационной цели.
    %
    % Класс не изменяет координаты цели. Он выбирает следующее состояние
    % на основе матрицы переходов, коэффициентов поведения и контекста среды.
    %
    % Пример:
    %   engine = DecisionEngine();
    %   decision = engine.decide(target.toDecisionInput(), environment);

    methods
        function decision = decide(~, target, environment)
            arguments
                ~
                target (1, 1) struct
                environment (1, 1) struct
            end

            DecisionEngine.validateTargetInput(target);
            DecisionEngine.validateEnvironmentInput(environment);

            currentState = target.CurrentState;
            if target.StateTime < BehaviorStateCatalog.minDuration(currentState)
                decision = DecisionEngine.buildDecision(currentState);
                return;
            end

            transitionMatrix = TransitionMatrixRegistry.getMatrix(target.Type);
            currentIndex = BehaviorStateCatalog.index(currentState);
            probabilities = transitionMatrix(currentIndex, :);

            context = DecisionContext.create(target, environment, probabilities);
            probabilities = ProbabilityModifiers.apply(probabilities, context);

            nextIndex = DecisionEngine.sampleState(probabilities);
            nextState = BehaviorStateCatalog.fromIndex(nextIndex);
            decision = DecisionEngine.buildDecision(nextState);
        end
    end

    methods (Static, Access = private)
        function decision = buildDecision(nextState)
            decision = struct( ...
                'NextState', nextState, ...
                'DesiredHeadingChange', 0, ...
                'DesiredPitchChange', 0, ...
                'DesiredSpeed', [], ...
                'DesiredAltitude', []);
        end

        function nextIndex = sampleState(probabilities)
            cumulative = cumsum(probabilities);
            randomValue = rand();
            nextIndex = find(randomValue <= cumulative, 1, 'first');

            if isempty(nextIndex)
                nextIndex = numel(probabilities);
            end
        end

        function validateTargetInput(target)
            requiredFields = {
                'Type'
                'Position'
                'Velocity'
                'Speed'
                'Heading'
                'Pitch'
                'CurrentState'
                'StateTime'
                'BehaviorCoefficients'
            };

            for k = 1:numel(requiredFields)
                fieldName = requiredFields{k};
                if ~isfield(target, fieldName)
                    error('DecisionEngine:InvalidTarget', ...
                        'Target input must contain field: %s', fieldName);
                end
            end
        end

        function validateEnvironmentInput(environment)
            requiredFields = {
                'AreaSize'
                'MinAltitude'
                'MaxAltitude'
                'SimulationTime'
                'TimeStep'
                'XLimits'
                'YLimits'
                'ZLimits'
            };

            for k = 1:numel(requiredFields)
                fieldName = requiredFields{k};
                if ~isfield(environment, fieldName)
                    error('DecisionEngine:InvalidEnvironment', ...
                        'Environment input must contain field: %s', fieldName);
                end
            end
        end
    end
end
