% demo_RadarTargetModel  Проверка базовой кинематической модели цели.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
setupRadarPaths();

environment = SimulationEnvironment.create( ...
    [10000, 10000, 5000], 0, 5000, 60, 1.0);

target = TargetFactory.createRandom(TargetType.AirplaneUAV, environment);

for step = 1:10
    target = target.update(1.0);
end

state = target.getState();

fprintf('ID: %d\n', state.ID);
fprintf('Type: %s\n', state.Type);
fprintf('Position: [%.2f, %.2f, %.2f] m\n', state.Position);
fprintf('Velocity: [%.2f, %.2f, %.2f] m/s\n', state.Velocity);
fprintf('Speed: %.2f m/s\n', state.Speed);
fprintf('History length: %d\n', size(target.HistoryPosition, 1));
