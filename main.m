% main  Финальный сценарий запуска полной симуляции (ТЗ №7, №15.1).
%
% Режимы демонстрации:
%   demoMode = "small";   % 4 цели
%   demoMode = "medium";  % несколько целей каждого типа
%   demoMode = "full";    % больше целей
%
% Параметры можно переопределить перед запуском:
%   demoMode = "medium"; numFalse = 4; run('main.m')

setupRadarPaths();

%% Режим демонстрации
if ~exist('demoMode', 'var')
    demoMode = "small";
end

demoPresets = struct( ...
    'small', struct( ...
        'NumFalse', 1, ...
        'NumGround', 1, ...
        'NumAir', 2, ...
        'BoxSize', [1000, 1000, 300], ...
        'Duration', 300), ...
    'medium', struct( ...
        'NumFalse', 2, ...
        'NumGround', 2, ...
        'NumAir', 4, ...
        'BoxSize', [2000, 2000, 400], ...
        'Duration', 300), ...
    'full', struct( ...
        'NumFalse', 3, ...
        'NumGround', 3, ...
        'NumAir', 6, ...
        'BoxSize', [4000, 4000, 500], ...
        'Duration', 360));

modeKey = char(demoMode);
if ~isfield(demoPresets, modeKey)
    error('main:InvalidDemoMode', 'Unknown demoMode: %s', modeKey);
end
preset = demoPresets.(modeKey);

%% Пользовательские параметры
if ~exist('numFalse', 'var'), numFalse = preset.NumFalse; end
if ~exist('numGround', 'var'), numGround = preset.NumGround; end
if ~exist('numAir', 'var'), numAir = preset.NumAir; end
if ~exist('boxSize', 'var'), boxSize = preset.BoxSize; end
if ~exist('outputPeriod', 'var'), outputPeriod = 5; end
if ~exist('duration', 'var'), duration = preset.Duration; end
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
fprintf('Demo mode: %s\n', modeKey);
fprintf('Targets total: %d\n', totalTargets);
fprintf('Output frames: %d\n', outputFrameCount);
fprintf('CSV saved: %s\n', csvFilename);
fprintf('MAT saved: %s\n', matFilename);
