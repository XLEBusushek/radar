# radar

Объектно-ориентированная модель радиолокационных целей на MATLAB.

## Структура проекта

```
radar/
├── enums/              % TargetType, TargetBehaviorState
├── models/             % RadarTargetModel, BehaviorCoefficients
├── profiles/           % TargetProfile, TargetProfileRegistry
├── factory/            % TargetFactory
├── trajectory/         % TrajectoryGenerator, motion models
├── simulation/         % SimulationEngine, PlotSimulationResult
├── integration/        % PhasedTargetAdapter
├── export/             % RadarOutputExporter
├── decision/           % DecisionEngine, матрицы переходов
├── examples/           % демо-скрипты
├── TestDecisionEngine.m
├── TestTargetProfiles.m
├── TestTrajectoryGenerator.m
└── setupRadarPaths.m
```

## Запуск демонстрации

```matlab
addpath('c:/path/to/radar');
main
```

Параметры задаются в начале `main.m` или перед вызовом `run('main.m')`.

```matlab
addpath('c:/path/to/radar');
setupRadarPaths();

environment = SimulationEnvironment.create([10000, 10000, 5000], 0, 5000, 300, 1);
target = TargetFactory.createRandom(TargetType.Quadcopter, environment);

engine = DecisionEngine();
decision = engine.decide(target.toDecisionInput(), environment);
target = TrajectoryGenerator.updateMotion(target, decision, environment, 1.0);

engine = SimulationEngine();
config.NumFalse = 5;
config.NumGround = 5;
config.NumAirplaneUAV = 3;
config.NumQuadcopter = 3;
config.BoxSize = [1000, 1000, 300];
config.Duration = 300;
config.Dt = 1;
config.OutputPeriod = 5;
config.RandomSeed = 42;
result = engine.run(config);
PlotSimulationResult(result);
```

## Тесты

```matlab
setupRadarPaths();
TestTargetProfiles         % валидация профилей целей (ТЗ 2.2)
TestDecisionEngine         % валидация Decision Engine (ТЗ 2.1)
TestTrajectoryGenerator    % валидация TrajectoryGenerator (ТЗ 3)
TestSimulationEngine       % валидация SimulationEngine (ТЗ 4)
TestPhasedTargetAdapter    % валидация PhasedTargetAdapter (ТЗ 5, требует Phased Array Toolbox)
TestRadarOutputExporter    % валидация RadarOutputExporter (ТЗ 6)
TestMainScenario           % валидация main.m (ТЗ 7)
```

## Реализованные этапы

| Этап | Описание |
|------|----------|
| ТЗ №1 | `RadarTargetModel` — кинематическая модель цели |
| ТЗ №2 | `DecisionEngine` — модуль принятия решений |
| ТЗ №2.1 | `TestDecisionEngine` — валидация Decision Engine |
| ТЗ №2.2 | `TargetProfileRegistry`, `TargetFactory` — профили и фабрика целей |
| ТЗ №3 | `TrajectoryGenerator` — модуль генерации траекторий |
| ТЗ №4 | `SimulationEngine` — полная симуляция целей |
| ТЗ №5 | `PhasedTargetAdapter` — интеграция с Phased Array Toolbox |
| ТЗ №6 | `RadarOutputExporter` — экспорт радиолокационных данных |
| ТЗ №7 | `main.m`, `PlotFlightMap` — финальный сценарий и карта полётов |

## Типы целей

- `False` — птица / ложная цель
- `Ground` — наземная цель
- `AirplaneUAV` — самолёт / БПЛА
- `Quadcopter` — квадрокоптер
