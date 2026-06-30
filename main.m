% main  Финальный сценарий запуска полной симуляции (ТЗ №7).
%
% Параметры можно переопределить перед запуском:
%   numFalse = 2; ...; run('main.m')

setupRadarPaths();

%% Пользовательские параметры
if ~exist('numFalse', 'var'), numFalse = 5; end
if ~exist('numGround', 'var'), numGround = 5; end
if ~exist('numAir', 'var'), numAir = 6; end
if ~exist('boxSize', 'var'), boxSize = [1000, 1000, 300]; end
if ~exist('outputPeriod', 'var'), outputPeriod = 5; end
if ~exist('duration', 'var'), duration = 300; end
if ~exist('dt', 'var'), dt = 1; end
if ~exist('randomSeed', 'var'), randomSeed = 42; end

numAirplaneUAV = floor(numAir / 2);
numQuadcopter = numAir - numAirplaneUAV;

config.NumFalse = numFalse;
config.NumGround = numGround;
config.NumAirplaneUAV = numAirplaneUAV;
config.NumQuadcopter = numQuadcopter;
config.BoxSize = boxSize;
config.Duration = duration;
config.Dt = dt;
config.OutputPeriod = outputPeriod;
config.RandomSeed = randomSeed;

engine = SimulationEngine();
result = engine.run(config);

outputs = RadarOutputExporter.exportSimulation(result);
tableData = RadarOutputExporter.toTable(outputs); %#ok<NASGU>

if ~exist('csvFilename', 'var'), csvFilename = 'radar_output.csv'; end
if ~exist('matFilename', 'var'), matFilename = 'radar_output.mat'; end

RadarOutputExporter.toCSV(outputs, csvFilename);
RadarOutputExporter.toMAT(outputs, matFilename);

PlotFlightMap(result);

totalTargets = numel(result.Targets);
outputFrameCount = numel(outputs);

fprintf('Simulation completed successfully.\n');
fprintf('Targets total: %d\n', totalTargets);
fprintf('Output frames: %d\n', outputFrameCount);
fprintf('CSV saved: %s\n', csvFilename);
fprintf('MAT saved: %s\n', matFilename);
