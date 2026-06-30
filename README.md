# radar

Объектно-ориентированная модель радиолокационных целей на MATLAB. Проект предназначен для моделирования поведения и движения различных типов целей в заданной области, формирования периодических выходных данных и подготовки объектов для дальнейшей радиолокационной обработки в Phased Array Toolbox.

## Реализованный функционал

Проект реализует:

- генерацию ложных, наземных и воздушных целей;
- ИИ-модуль выбора поведения (`DecisionEngine`);
- марковские матрицы переходов между состояниями;
- физические модели движения для каждого типа цели;
- экспорт координат, скоростей и ЭПР;
- 3D-карту траекторий;
- адаптер для `phased.Platform`;
- адаптер для `phased.RadarTarget`.

## Архитектура

Симуляция построена по модульному принципу. Каждый этап выполняет одну задачу и не дублирует логику соседних модулей.

```
TargetFactory → SimulationEngine → RadarOutputExporter
                      ↓
              DecisionEngine (выбор состояния)
                      ↓
              TrajectoryGenerator (физика движения)
                      ↓
              PhasedTargetAdapter (интеграция с Phased Array Toolbox)
```

**Основные компоненты:**

| Модуль | Назначение |
|--------|------------|
| `RadarTargetModel` | хранение состояния цели и истории движения |
| `TargetProfileRegistry` | физические ограничения по типам целей |
| `TargetFactory` | создание целей со случайными параметрами в допустимых диапазонах |
| `DecisionEngine` | выбор следующего состояния поведения на основе матриц переходов |
| `TrajectoryGenerator` | исполнение решения и обновление координат, скорости, курса |
| `SimulationEngine` | полный цикл симуляции и формирование выходных кадров |
| `RadarOutputExporter` | экспорт данных в структуру, таблицу, CSV и MAT |
| `PhasedTargetAdapter` | синхронизация с `phased.Platform` и `phased.RadarTarget` |

Поведение цели выбирается **марковским процессом**: для каждого типа цели задана матрица переходов, вероятности корректируются с учётом среды и индивидуальных коэффициентов, после чего выполняется случайная выборка следующего состояния.

## Структура папок

```
radar/
├── enums/                  % TargetType, TargetBehaviorState
├── models/                 % RadarTargetModel, BehaviorCoefficients
├── profiles/               % TargetProfile, TargetProfileRegistry
├── factory/                % TargetFactory
├── decision/               % DecisionEngine, ProbabilityModifiers, матрицы переходов
│   └── matrices/           % Bird, Ground, Airplane, Quad transition matrices
├── trajectory/             % TrajectoryGenerator и модели движения
├── simulation/             % SimulationEngine, PlotFlightMap, PlotSimulationResult
├── integration/            % PhasedTargetAdapter
├── export/                 % RadarOutputExporter
├── examples/               % демонстрационные скрипты
├── main.m                  % финальный сценарий запуска
├── setupRadarPaths.m       % добавление путей проекта
├── RunAllTests.m           % запуск всех тестов
└── Test*.m                 % тесты валидации модулей
```

## Входные параметры

### Параметры `main.m`

| Параметр | Описание |
|----------|----------|
| `numFalse` | количество ложных целей (птиц) |
| `numGround` | количество наземных целей |
| `numAir` | количество воздушных целей (распределяется 50/50 между `AirplaneUAV` и `Quadcopter`) |
| `boxSize` | размер области моделирования `[X Y Z]`, м |
| `outputPeriod` | период выдачи данных, с |
| `duration` | длительность симуляции, с |
| `dt` | шаг моделирования, с |
| `randomSeed` | зерно генератора случайных чисел |

### Параметры `SimulationEngine` (`config`)

```matlab
config.NumFalse
config.NumGround
config.NumAirplaneUAV
config.NumQuadcopter
config.BoxSize          % [X Y Z]
config.Duration         % с
config.Dt               % с
config.OutputPeriod     % с
config.RandomSeed
```

## Выходные данные

### Файлы

- `radar_output.csv` — плоская таблица всех кадров;
- `radar_output.mat` — полная структура `radarOutputs`.

### Формат одной цели в кадре

```matlab
targetOutput.ID
targetOutput.Type
targetOutput.State
targetOutput.IsHidden
targetOutput.RCS
targetOutput.Time
targetOutput.Kinematics     % матрица 3×2: [x vx; y vy; z vz]
```

### Таблица (`RadarOutputExporter.toTable`)

Столбцы: `Time`, `ID`, `Type`, `State`, `IsHidden`, `X`, `Y`, `Z`, `Vx`, `Vy`, `Vz`, `RCS`.

### Результат `SimulationEngine`

```matlab
result.Targets         % финальные объекты RadarTargetModel
result.OutputFrames    % снимки состояния на каждый OutputPeriod
result.Config          % использованная конфигурация
result.Statistics      % статистика по типам и времени выполнения
```

Количество выходных кадров: `Duration / OutputPeriod + 1`.

## Типы целей

| Тип | Описание | Скорость, м/с | ЭПР, м² | Высота, м |
|-----|----------|---------------|---------|-----------|
| `False` | птица / ложная цель | 5–15 | 0.001–0.03 | 0–40 |
| `Ground` | наземная цель | 5–30 | 5–30 | 0–30 |
| `AirplaneUAV` | самолёт / БПЛА | 10–20 | 0.01–0.1 | 0–5000 |
| `Quadcopter` | квадрокоптер | 5–12 | 0.01–0.1 | 0–500 |

### Состояния поведения

`FlyStraight`, `TurnLeft`, `TurnRight`, `Climb`, `Descend`, `Hover`, `SpeedUp`, `SlowDown`, `Hidden`.

Не все состояния допустимы для каждого типа. Например, наземная цель не использует `Hover`, `Climb`, `Descend`.

## Запуск

### Быстрый старт

```matlab
addpath('c:/path/to/radar');
main
```

### Настройка параметров перед запуском

```matlab
addpath('c:/path/to/radar');
numFalse = 3;
numGround = 3;
numAir = 4;
boxSize = [1000, 1000, 300];
outputPeriod = 5;
duration = 120;
dt = 1;
randomSeed = 42;
main
```

### Программный запуск

```matlab
setupRadarPaths();

config.NumFalse = 5;
config.NumGround = 5;
config.NumAirplaneUAV = 3;
config.NumQuadcopter = 3;
config.BoxSize = [1000, 1000, 300];
config.Duration = 300;
config.Dt = 1;
config.OutputPeriod = 5;
config.RandomSeed = 42;

result = SimulationEngine().run(config);
outputs = RadarOutputExporter.exportSimulation(result);
PlotFlightMap(result);
```

## Запуск тестов

### Все тесты

```matlab
addpath('c:/path/to/radar');
RunAllTests
```

### Отдельные тесты

```matlab
setupRadarPaths();

TestTargetProfiles
TestDecisionEngine
TestTrajectoryGenerator
TestSimulationEngine
TestRadarOutputExporter
TestPhasedTargetAdapter      % требует Phased Array Toolbox
TestMainScenario
```

При отсутствии Phased Array Toolbox тест `TestPhasedTargetAdapter` пропускается без ошибки.

Успешное прохождение всех тестов:

```
ALL TESTS PASSED
```

## Ограничения текущей версии

- радарный сигнал и обработка сигналов **не моделируются**;
- для скрытых целей (`Hidden`) ЭПР **не обнуляется** — фильтрация будет добавлена позже;
- `PhasedTargetAdapter` требует Phased Array System Toolbox;
- воздушные цели из `numAir` распределяются между `AirplaneUAV` и `Quadcopter` в пропорции 50/50;
- взаимодействие между целями не моделируется;
- полный прогон `TestDecisionEngine` и `RunAllTests` может занимать несколько минут из-за длительности симуляции (300 с).

## Возможные направления развития

- моделирование радарного канала и отражённого сигнала;
- фильтрация скрытых целей на этапе радиолокационного вывода;
- добавление новых типов целей через профили и матрицы переходов;
- визуализация состояний поведения во времени;
- интеграция с полной цепочкой `phased.RadarTarget` → обработка → обнаружение;
- параллельная симуляция большого числа целей;
- загрузка сценариев из внешних конфигурационных файлов.

## Требования

- MATLAB R2019b или новее (рекомендуется);
- Phased Array System Toolbox — опционально, для `PhasedTargetAdapter`.

## Лицензия

Учебный / исследовательский проект.
