# radar

Объектно-ориентированная модель радиолокационных целей на MATLAB.

## Структура проекта

```
radar/
├── enums/              % TargetType, TargetBehaviorState
├── models/             % RadarTargetModel, BehaviorCoefficients
├── profiles/           % TargetProfile, TargetProfileRegistry
├── factory/            % TargetFactory
├── decision/           % DecisionEngine, матрицы переходов
├── examples/           % демо-скрипты
├── TestDecisionEngine.m
├── TestTargetProfiles.m
└── setupRadarPaths.m
```

## Быстрый старт

```matlab
addpath('c:/path/to/radar');
setupRadarPaths();

environment = SimulationEnvironment.create([10000, 10000, 5000], 0, 5000, 300, 1);
target = TargetFactory.createRandom(TargetType.Quadcopter, environment);

engine = DecisionEngine();
decision = engine.decide(target.toDecisionInput(), environment);
target = target.applyDecision(decision);
target = target.update(1.0);
```

## Тесты

```matlab
setupRadarPaths();
TestTargetProfiles      % валидация профилей целей (ТЗ 2.2)
TestDecisionEngine      % валидация Decision Engine (ТЗ 2.1)
```

## Реализованные этапы

| Этап | Описание |
|------|----------|
| ТЗ №1 | `RadarTargetModel` — кинематическая модель цели |
| ТЗ №2 | `DecisionEngine` — модуль принятия решений |
| ТЗ №2.1 | `TestDecisionEngine` — валидация Decision Engine |
| ТЗ №2.2 | `TargetProfileRegistry`, `TargetFactory` — профили и фабрика целей |

## Типы целей

- `False` — птица / ложная цель
- `Ground` — наземная цель
- `AirplaneUAV` — самолёт / БПЛА
- `Quadcopter` — квадрокоптер
