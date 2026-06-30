classdef DecisionContext
    % DecisionContext  Контекст для модификации вероятностей перехода.

    methods (Static)
        function context = create(target, environment, baseProbabilities)
            context.Target = target;
            context.Environment = environment;
            context.BaseProbabilities = baseProbabilities;
            context.ValidMask = BehaviorStateCatalog.validStateMask(target.Type);
            context.StateIndices = DecisionContext.stateIndices();
        end

        function indices = stateIndices()
            persistent cachedIndices;

            if isempty(cachedIndices)
                states = BehaviorStateCatalog.orderedStates();
                cachedIndices = struct();
                for k = 1:numel(states)
                    fieldName = char(states(k));
                    cachedIndices.(fieldName) = k;
                end
            end

            indices = cachedIndices;
        end
    end
end
