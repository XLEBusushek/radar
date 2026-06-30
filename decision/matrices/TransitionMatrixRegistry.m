classdef TransitionMatrixRegistry
    % TransitionMatrixRegistry  Связь типа цели с базовой матрицей переходов.

    methods (Static)
        function P = getMatrix(targetType)
            matrixMap = TransitionMatrixRegistry.matrixFunctionMap();
            matrixBuilder = matrixMap(char(targetType));
            P = matrixBuilder();
        end

        function map = matrixFunctionMap()
            persistent cachedMap;

            if isempty(cachedMap)
                cachedMap = containers.Map( ...
                    cellstr(TargetType.allValues()), ...
                    {@BirdTransitionMatrix, @GroundTransitionMatrix, ...
                    @AirplaneTransitionMatrix, @QuadTransitionMatrix});
            end

            map = cachedMap;
        end
    end
end
