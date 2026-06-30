classdef MotionModelRegistry
    % MotionModelRegistry  Связь типа цели с моделью движения.

    methods (Static)
        function updateFn = getModel(targetType)
            modelMap = MotionModelRegistry.modelMap();
            updateFn = modelMap(char(targetType));
        end
    end

    methods (Static, Access = private)
        function modelMap = modelMap()
            persistent cachedMap;

            if isempty(cachedMap)
                cachedMap = containers.Map( ...
                    cellstr(TargetType.allValues()), ...
                    {@(t, d, bc, p, e, dt) BirdMotionModel.update(t, d, bc, p, e, dt), ...
                    @(t, d, bc, p, e, dt) GroundMotionModel.update(t, d, bc, p, e, dt), ...
                    @(t, d, bc, p, e, dt) AirplaneMotionModel.update(t, d, bc, p, e, dt), ...
                    @(t, d, bc, p, e, dt) QuadcopterMotionModel.update(t, d, bc, p, e, dt)});
            end

            modelMap = cachedMap;
        end
    end
end
