classdef BehaviorStateCatalog
    % BehaviorStateCatalog  Единый порядок состояний и правила длительности.

    methods (Static)
        function states = orderedStates()
            persistent cachedStates;

            if isempty(cachedStates)
                cachedStates = [
                    TargetBehaviorState.FlyStraight
                    TargetBehaviorState.TurnLeft
                    TargetBehaviorState.TurnRight
                    TargetBehaviorState.Climb
                    TargetBehaviorState.Descend
                    TargetBehaviorState.Hover
                    TargetBehaviorState.SpeedUp
                    TargetBehaviorState.SlowDown
                    TargetBehaviorState.Hidden
                ];
            end

            states = cachedStates;
        end

        function n = count()
            n = numel(BehaviorStateCatalog.orderedStates());
        end

        function idx = index(state)
            states = BehaviorStateCatalog.orderedStates();
            idx = find(states == state, 1, 'first');
            if isempty(idx)
                error('BehaviorStateCatalog:UnknownState', ...
                    'Unknown behavior state: %s', string(state));
            end
        end

        function state = fromIndex(idx)
            states = BehaviorStateCatalog.orderedStates();
            state = states(idx);
        end

        function minTime = minDuration(state)
            minTimeValues = [3, 1, 1, 2, 2, 2, 2, 2, 3];
            stateIndex = BehaviorStateCatalog.index(state);
            minTime = minTimeValues(stateIndex);
        end

        function mask = validStateMask(targetType)
            mask = true(BehaviorStateCatalog.count(), 1);
            invalidStates = BehaviorStateCatalog.invalidStates(targetType);
            for state = invalidStates
                mask(BehaviorStateCatalog.index(state)) = false;
            end
        end

        function invalidStates = invalidStates(targetType)
            hover = TargetBehaviorState.Hover;
            climb = TargetBehaviorState.Climb;
            descend = TargetBehaviorState.Descend;
            none = TargetBehaviorState.empty(1, 0);

            invalidByType = containers.Map( ...
                cellstr(TargetType.allValues()), ...
                {hover, [hover, climb, descend], hover, none});

            invalidStates = invalidByType(char(targetType));
        end
    end
end
