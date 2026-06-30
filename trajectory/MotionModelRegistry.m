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
                    {@(t, d, p, e, dt) BirdMotionModel.update(t, d, p, e, dt), ...
                    @(t, d, p, e, dt) GroundMotionModel.update(t, d, p, e, dt), ...
                    @(t, d, p, e, dt) AirplaneMotionModel.update(t, d, p, e, dt), ...
                    @(t, d, p, e, dt) QuadcopterMotionModel.update(t, d, p, e, dt)});
            end

            modelMap = cachedMap;
        end
    end
end
