% main  Финальный сценарий запуска полной симуляции (ТЗ №7, №15.1–15.2).
%
% Режимы демонстрации:
%   demoMode = "small";   % 4 цели
%   demoMode = "medium";  % 14 целей
%   demoMode = "full";    % 28 целей
%
% Параметры можно переопределить перед запуском:
%   demoMode = "medium"; numFalse = 4; boxSize = [1500, 1500, 500]; run('main.m')

setupRadarPaths();

%% Режим демонстрации
if ~exist('demoMode', 'var')
    demoMode = "small";
end

defaultBoxSize = [2000, 2000, 500];
demoPresets = struct( ...
    'small', struct( ...
        'NumFalse', 1, ...
        'NumGround', 1, ...
        'NumAirplaneUAV', 1, ...
        'NumQuadcopter', 1, ...
        'BoxSize', defaultBoxSize, ...
        'Duration', 300), ...
    'medium', struct( ...
        'NumFalse', 6, ...
        'NumGround', 3, ...
        'NumAirplaneUAV', 2, ...
        'NumQuadcopter', 3, ...
        'BoxSize', defaultBoxSize, ...
        'Duration', 300), ...
    'full', struct( ...
        'NumFalse', 12, ...
        'NumGround', 6, ...
        'NumAirplaneUAV', 4, ...
        'NumQuadcopter', 6, ...
        'BoxSize', defaultBoxSize, ...
        'Duration', 360));

modeKey = char(demoMode);
if ~isfield(demoPresets, modeKey)
    error('main:InvalidDemoMode', 'Unknown demoMode: %s', modeKey);
end
preset = demoPresets.(modeKey);

%% Пользовательские параметры
if ~exist('numFalse', 'var'), numFalse = preset.NumFalse; end
if ~exist('numGround', 'var'), numGround = preset.NumGround; end
if ~exist('numAirplaneUAV', 'var'), numAirplaneUAV = preset.NumAirplaneUAV; end
if ~exist('numQuadcopter', 'var'), numQuadcopter = preset.NumQuadcopter; end
if ~exist('boxSize', 'var'), boxSize = preset.BoxSize; end
if ~exist('outputPeriod', 'var'), outputPeriod = 5; end
if ~exist('duration', 'var'), duration = preset.Duration; end
if ~exist('dt', 'var'), dt = 1; end
if ~exist('randomSeed', 'var'), randomSeed = 42; end

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
fprintf('Demo mode: %s\n', modeKey);
fprintf('Box size: [%.0f, %.0f, %.0f] m\n', boxSize(1), boxSize(2), boxSize(3));
fprintf('Targets total: %d\n', totalTargets);
fprintf('Output frames: %d\n', outputFrameCount);
fprintf('CSV saved: %s\n', csvFilename);
fprintf('MAT saved: %s\n', matFilename);
