% demo_DecisionEngine  Проверка модуля принятия решений.

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
setupRadarPaths();

environment = SimulationEnvironment.create( ...
    [10000, 10000, 5000], ...
    0, ...
    5000, ...
    600, ...
    0.1);

engine = DecisionEngine();

typePlan = [
    TargetType.AirplaneUAV
    TargetType.Ground
    TargetType.Quadcopter
    TargetType.False
];

targets = cell(numel(typePlan), 1);
for k = 1:numel(typePlan)
    targets{k} = TargetFactory.createRandom(typePlan(k), environment);
end

fprintf('=== Decision Engine demo ===\n\n');

for k = 1:numel(targets)
    target = targets{k};

    for step = 1:40
        decision = engine.decide(target.toDecisionInput(), environment);
        target = target.applyDecision(decision);
        target = target.update(environment.TimeStep);
    end

    state = target.getState();
    fprintf('Type: %s\n', state.Type);
    fprintf('  Current state: %s\n', string(state.CurrentState));
    fprintf('  State time: %.2f s\n', state.StateTime);
    fprintf('  Position: [%.1f, %.1f, %.1f]\n\n', state.Position);
end

transitionMatrix = TransitionMatrixRegistry.getMatrix(TargetType.Ground);
fprintf('Ground matrix row sums: ');
fprintf('%.3f ', sum(transitionMatrix, 2));
fprintf('\n');

invalidMask = ~BehaviorStateCatalog.validStateMask(TargetType.Ground);
fprintf('Ground invalid columns max prob: %.3f\n', max(transitionMatrix(:, invalidMask), [], 'all'));
