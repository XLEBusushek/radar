classdef SimulationEnvironment
    % SimulationEnvironment  Параметры среды моделирования для Decision Engine.

    methods (Static)
        function environment = create(areaSize, minAltitude, maxAltitude, simulationTime, timeStep)
            arguments
                areaSize (1, 3) double {mustBePositive}
                minAltitude (1, 1) double {mustBeNonnegative}
                maxAltitude (1, 1) double {mustBeNonnegative}
                simulationTime (1, 1) double {mustBePositive}
                timeStep (1, 1) double {mustBePositive}
            end

            if maxAltitude < minAltitude
                error('SimulationEnvironment:InvalidAltitude', ...
                    'maxAltitude must be greater than or equal to minAltitude.');
            end

            halfXY = areaSize(1:2) / 2;
            environment.AreaSize = areaSize;
            environment.BoxSize = areaSize;
            environment.XLimits = [-halfXY(1), halfXY(1)];
            environment.YLimits = [-halfXY(2), halfXY(2)];
            environment.ZLimits = [minAltitude, maxAltitude];
            environment.MinAltitude = minAltitude;
            environment.MaxAltitude = maxAltitude;
            environment.SimulationTime = simulationTime;
            environment.TimeStep = timeStep;
        end
    end
end
