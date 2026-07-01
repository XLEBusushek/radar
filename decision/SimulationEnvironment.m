classdef SimulationEnvironment
    % SimulationEnvironment  Параметры среды моделирования для Decision Engine.

    methods (Static)
        function environment = create(areaSize, minAltitude, maxAltitude, simulationTime, timeStep, randomSeed)
            arguments
                areaSize (1, 3) double {mustBePositive}
                minAltitude (1, 1) double {mustBeNonnegative}
                maxAltitude (1, 1) double {mustBeNonnegative}
                simulationTime (1, 1) double {mustBePositive}
                timeStep (1, 1) double {mustBePositive}
                randomSeed (1, 1) double = 42
            end

            environment = EnvironmentGenerator.generate( ...
                'BoxSize', areaSize, ...
                'RandomSeed', randomSeed, ...
                'MinAltitude', minAltitude, ...
                'MaxAltitude', maxAltitude, ...
                'SimulationTime', simulationTime, ...
                'TimeStep', timeStep);
        end
    end
end
