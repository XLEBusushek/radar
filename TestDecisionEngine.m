% TestDecisionEngine  Валидация RadarTargetModel и DecisionEngine (ТЗ №2.1).
%
% Запуск:
%   addpath('путь/к/radar'); setupRadarPaths(); TestDecisionEngine

function TestDecisionEngine()
    rng(42);

    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    results = initValidationResults();
    simDuration = 300;
    dt = 1;

    environment = SimulationEnvironment.create( ...
        [10000, 10000, 5000], 0, 5000, simDuration, dt);

    engine = DecisionEngine();
    targets = createValidationTargets(environment);

    results = validateTransitionMatrices(results);
    results = validateArchitecture(results, projectRoot);

    numTargets = numel(targets);
    timeAxis = (0:(simDuration - 1)) * dt;
    stateTimeline = zeros(numTargets, simDuration);
    changeCounts = zeros(numTargets, 1);

    fprintf('=== Decision Engine Validation ===\n');
    fprintf('Objects: %d | Duration: %d s | dt: %d s\n\n', numTargets, simDuration, dt);

    for targetIdx = 1:numTargets
        target = targets{targetIdx};
        stateTimeline(targetIdx, 1) = BehaviorStateCatalog.index(target.CurrentState);
        results = recordPass(results, sprintf('Target %d initialized (Type=%s).', target.ID, target.Type));
    end

    for step = 1:simDuration
        for targetIdx = 1:numTargets
            target = targets{targetIdx};
            targetInput = target.toDecisionInput();

            stateBefore = target.CurrentState;
            stateTimeBefore = target.StateTime;
            minDuration = BehaviorStateCatalog.minDuration(stateBefore);

            results = validateCurrentStateForType(results, target, step);

            decision = engine.decide(targetInput, environment);
            results = validateDecisionOutput(results, target, decision, step);
            results = validateMinimumStateDuration(results, target, decision, stateTimeBefore, step);
            results = validateTypeRestrictions(results, target, decision.NextState, step);

            if stateTimeBefore >= minDuration
                results = validateProbabilities(results, targetInput, environment, stateBefore, decision, step);
            else
                results = validateStateHeldDuringMinimumTime(results, stateBefore, decision, step, target.ID);
            end

            if decision.NextState ~= stateBefore
                changeCounts(targetIdx) = changeCounts(targetIdx) + 1;
            end

            target = target.applyDecision(decision);
            target.StateTime = target.StateTime + dt;
            targets{targetIdx} = target;

            stateTimeline(targetIdx, step) = BehaviorStateCatalog.index(target.CurrentState);
        end
    end

    for targetIdx = 1:numTargets
        results = validateMinimumDurationEligibility(results, targets{targetIdx}, stateTimeline(targetIdx, :), dt);
    end

    printTargetStatistics(targets, stateTimeline, changeCounts, dt);
    plotStateTimeline(timeAxis, stateTimeline, targets);
    printValidationReport(results);
end

%% --- Scenario setup ---

function targets = createValidationTargets(environment)
    typePlan = [
        TargetType.False
        TargetType.False
        TargetType.False
        TargetType.Ground
        TargetType.Ground
        TargetType.Ground
        TargetType.AirplaneUAV
        TargetType.AirplaneUAV
        TargetType.Quadcopter
        TargetType.Quadcopter
    ];

    targets = cell(numel(typePlan), 1);

    for k = 1:numel(typePlan)
        targets{k} = TargetFactory.createRandom(typePlan(k), environment);
    end
end

%% --- Matrix and architecture validation ---

function results = validateTransitionMatrices(results)
    tolerance = 1e-6;
    targetTypes = TargetType.allValues();

    for k = 1:numel(targetTypes)
        targetType = targetTypes(k);
        transitionMatrix = TransitionMatrixRegistry.getMatrix(targetType);
        rowSums = sum(transitionMatrix, 2);

        if any(abs(rowSums - 1) > tolerance)
            results = recordError(results, ...
                sprintf('Matrix row sum deviation for type %s (max error %.3e).', targetType, max(abs(rowSums - 1))), ...
                'TransitionMatrixRegistry / matrices');
        else
            results = recordPass(results, sprintf('Matrix row sums valid for type %s.', targetType));
        end

        validMask = BehaviorStateCatalog.validStateMask(targetType);
        validRows = find(validMask);

        for row = validRows'
            forbiddenTransitions = transitionMatrix(row, ~validMask);
            if any(forbiddenTransitions > tolerance)
                results = recordError(results, ...
                    sprintf('Forbidden transition probability in base matrix for type %s, row %d.', targetType, row), ...
                    sprintf('%sTransitionMatrix', matrixNameForType(targetType)));
            end
        end
    end

    results = recordPass(results, 'Base transition matrices checked for all target types.');
end

function results = validateArchitecture(results, projectRoot)
    decisionEnginePath = fullfile(projectRoot, 'decision', 'DecisionEngine.m');
    engineSource = fileread(decisionEnginePath);

    behaviorPatterns = {
        'applyBoundaryProximity'
        'applyAltitudeLimits'
        'applyBehaviorCoefficients'
        'turnScale'
        'marginRatio'
    };

    for k = 1:numel(behaviorPatterns)
        if contains(engineSource, behaviorPatterns{k})
            results = recordError(results, ...
                sprintf('DecisionEngine contains behavior logic fragment: %s.', behaviorPatterns{k}), ...
                decisionEnginePath);
        end
    end

    if ~contains(engineSource, 'TransitionMatrixRegistry.getMatrix')
        results = recordError(results, 'DecisionEngine does not use TransitionMatrixRegistry.', decisionEnginePath);
    end

    if ~contains(engineSource, 'ProbabilityModifiers.apply')
        results = recordError(results, 'DecisionEngine does not use ProbabilityModifiers.', decisionEnginePath);
    end

    elseifCount = countToken(engineSource, 'elseif');
    if elseifCount > 0
        results = recordWarning(results, ...
            sprintf('DecisionEngine contains %d elseif branch(es). Expected only validation branches.', elseifCount), ...
            decisionEnginePath);
    else
        results = recordPass(results, 'DecisionEngine has no elseif branches for behavior selection.');
    end

    matrixFiles = {
        'BirdTransitionMatrix.m'
        'GroundTransitionMatrix.m'
        'AirplaneTransitionMatrix.m'
        'QuadTransitionMatrix.m'
    };

    for k = 1:numel(matrixFiles)
        matrixPath = fullfile(projectRoot, 'decision', 'matrices', matrixFiles{k});
        if ~isfile(matrixPath)
            results = recordError(results, sprintf('Missing transition matrix file: %s.', matrixFiles{k}), matrixPath);
        end
    end

    registryPath = fullfile(projectRoot, 'decision', 'matrices', 'TransitionMatrixRegistry.m');
    registrySource = fileread(registryPath);
    if ~contains(registrySource, 'matrixFunctionMap')
        results = recordError(results, 'TransitionMatrixRegistry lacks extensible type map.', registryPath);
    else
        results = recordPass(results, 'Transition matrices are separated from decision algorithm.');
    end

    modifierPath = fullfile(projectRoot, 'decision', 'ProbabilityModifiers.m');
    if isfile(modifierPath)
        results = recordPass(results, 'Probability modifiers stored in dedicated module.');
    end

    results = recordPass(results, 'Architecture separation checks completed.');
end

%% --- Runtime validation ---

function results = validateCurrentStateForType(results, target, step)
    if ~isStateAllowedForType(target.Type, target.CurrentState)
        results = recordError(results, ...
            sprintf('Target %d entered forbidden state %s at step %d.', ...
            target.ID, string(target.CurrentState), step), ...
            'Simulation loop / CurrentState');
    else
        results = recordPass(results, sprintf('Target %d state valid at step %d.', target.ID, step));
    end
end

function results = validateDecisionOutput(results, target, decision, step)
    requiredFields = {
        'NextState'
        'DesiredHeadingChange'
        'DesiredPitchChange'
        'DesiredSpeed'
        'DesiredAltitude'
    };

    for k = 1:numel(requiredFields)
        if ~isfield(decision, requiredFields{k})
            results = recordError(results, ...
                sprintf('Target %d decision missing field %s at step %d.', ...
                target.ID, requiredFields{k}, step), ...
                'DecisionEngine.decide');
        end
    end

    results = recordPass(results, sprintf('Target %d decision structure valid at step %d.', target.ID, step));
end

function results = validateMinimumStateDuration(results, target, decision, stateTimeBefore, step)
    minDuration = BehaviorStateCatalog.minDuration(target.CurrentState);

    if stateTimeBefore < minDuration && decision.NextState ~= target.CurrentState
        results = recordError(results, ...
            sprintf(['Target %d changed state from %s to %s at step %d ' ...
            'before minimum duration (%.2f < %.2f s).'], ...
            target.ID, string(target.CurrentState), string(decision.NextState), ...
            step, stateTimeBefore, minDuration), ...
            'DecisionEngine.decide / StateTime');
    else
        results = recordPass(results, sprintf('Target %d minimum duration respected at step %d.', target.ID, step));
    end
end

function results = validateStateHeldDuringMinimumTime(results, stateBefore, decision, step, targetId)
    if decision.NextState == stateBefore
        results = recordPass(results, ...
            sprintf('Target %d correctly held state during minimum duration at step %d.', targetId, step));
    end
end

function results = validateTypeRestrictions(results, target, nextState, step)
    forbiddenStates = forbiddenStatesForType(target.Type);

    for state = forbiddenStates
        if nextState == state
            results = recordError(results, ...
                sprintf('Target %d (Type=%s) selected forbidden state %s at step %d.', ...
                target.ID, target.Type, string(state), step), ...
                'DecisionEngine.decide / type restrictions');
        end
    end

    results = recordPass(results, sprintf('Target %d type restrictions respected at step %d.', target.ID, step));
end

function results = validateProbabilities(results, targetInput, environment, stateBefore, decision, step)
    tolerance = 1e-6;
    transitionMatrix = TransitionMatrixRegistry.getMatrix(targetInput.Type);
    currentIndex = BehaviorStateCatalog.index(stateBefore);
    baseRow = transitionMatrix(currentIndex, :);

    context = DecisionContext.create(targetInput, environment, baseRow);
    modifiedProbabilities = ProbabilityModifiers.apply(baseRow, context);

    if abs(sum(modifiedProbabilities) - 1) > tolerance
        results = recordError(results, ...
            sprintf('Modified probabilities not normalized at step %d (sum=%.9f).', ...
            step, sum(modifiedProbabilities)), ...
            'ProbabilityModifiers.apply');
    else
        results = recordPass(results, sprintf('Probability normalization valid at step %d.', step));
    end

    validMask = BehaviorStateCatalog.validStateMask(targetInput.Type);
    if any(modifiedProbabilities(~validMask) > tolerance)
        results = recordError(results, ...
            sprintf('Positive probability for forbidden state at step %d.', step), ...
            'ProbabilityModifiers.apply');
    end

    if decision.NextState ~= stateBefore
        nextIndex = BehaviorStateCatalog.index(decision.NextState);
        if modifiedProbabilities(nextIndex) <= tolerance
            results = recordError(results, ...
                sprintf(['Target type %s transitioned to state %s with zero ' ...
                'probability at step %d.'], ...
                targetInput.Type, string(decision.NextState), step), ...
                'DecisionEngine.decide / random sampling');
        else
            results = recordPass(results, sprintf('Non-zero transition probability confirmed at step %d.', step));
        end
    end
end

function results = validateMinimumDurationEligibility(results, target, stateIndices, dt)
    totalTime = numel(stateIndices) * dt;

    if numel(unique(stateIndices)) > 1
        results = recordPass(results, ...
            sprintf('Target %d changed state after minimum durations were satisfied.', target.ID));
    else
        results = recordWarning(results, ...
            sprintf('Target %d never changed state during %d s simulation.', target.ID, totalTime), ...
            'Simulation loop');
    end
end

%% --- Statistics and visualization ---

function printTargetStatistics(targets, stateTimeline, changeCounts, dt)
    states = BehaviorStateCatalog.orderedStates();
    totalTime = size(stateTimeline, 2) * dt;

    fprintf('\n=== Per-object statistics ===\n');

    for targetIdx = 1:numel(targets)
        target = targets{targetIdx};
        timeline = stateTimeline(targetIdx, :);

        fprintf('\nTarget ID %d (%s):\n', target.ID, target.Type);
        fprintf('  State changes: %d\n', changeCounts(targetIdx));

        for stateIdx = 1:numel(states)
            state = states(stateIdx);
            stateMask = timeline == BehaviorStateCatalog.index(state);
            timeInState = sum(stateMask) * dt;
            percentTime = 100 * timeInState / totalTime;
            avgDuration = averageEpisodeDuration(stateMask, dt);

            fprintf('  %-12s: %6.2f%% time | avg episode %.2f s\n', ...
                char(string(state)), percentTime, avgDuration);
        end
    end
end

function avgDuration = averageEpisodeDuration(stateMask, dt)
    if ~any(stateMask)
        avgDuration = 0;
        return;
    end

    padded = [false, stateMask, false];
    starts = find(diff(padded) == 1);
    ends = find(diff(padded) == -1) - 1;
    episodeLengths = (ends - starts + 1) * dt;
    avgDuration = mean(episodeLengths);
end

function plotStateTimeline(timeAxis, stateTimeline, targets)
    figure('Name', 'Decision Engine State Timeline', 'NumberTitle', 'off');
    hold on;

    colors = lines(numel(targets));

    for targetIdx = 1:numel(targets)
        plot(timeAxis, stateTimeline(targetIdx, :), ...
            'LineWidth', 1.2, ...
            'Color', colors(targetIdx, :), ...
            'DisplayName', sprintf('ID %d (%s)', targets{targetIdx}.ID, targets{targetIdx}.Type));
    end

    stateCount = BehaviorStateCatalog.count();
    yticks(1:stateCount);
    yticklabels(arrayfun(@char, BehaviorStateCatalog.orderedStates(), 'UniformOutput', false));
    xlabel('Time, s');
    ylabel('State index');
    title('Behavior state timeline (Decision Engine validation)');
    grid on;
    legend('Location', 'eastoutside');
    hold off;
end

%% --- Validation report ---

function printValidationReport(results)
    fprintf('\n=== Validation report ===\n');
    fprintf('Passed checks : %d\n', results.passed);
    fprintf('Warnings      : %d\n', results.warnings);
    fprintf('Errors        : %d\n', results.errors);

    if ~isempty(results.warningDetails)
        fprintf('\nWarnings:\n');
        for k = 1:numel(results.warningDetails)
            fprintf('  [%d] %s\n    Location: %s\n', ...
                k, results.warningDetails{k}.message, results.warningDetails{k}.location);
        end
    end

    if ~isempty(results.errorDetails)
        fprintf('\nErrors:\n');
        for k = 1:numel(results.errorDetails)
            fprintf('  [%d] %s\n    Location: %s\n', ...
                k, results.errorDetails{k}.message, results.errorDetails{k}.location);
        end
    end

    fprintf('\n');
    if results.errors == 0
        fprintf('Decision Engine Validation PASSED\n');
    else
        fprintf('Decision Engine Validation FAILED\n');
    end
end

%% --- Result tracking helpers ---

function results = initValidationResults()
    results = struct();
    results.passed = 0;
    results.warnings = 0;
    results.errors = 0;
    results.warningDetails = {};
    results.errorDetails = {};
end

function results = recordPass(results, ~)
    results.passed = results.passed + 1;
end

function results = recordWarning(results, message, location)
    results.warnings = results.warnings + 1;
    results.warningDetails{end + 1} = struct('message', message, 'location', location); %#ok<AGROW>
    fprintf('WARNING: %s (%s)\n', message, location);
end

function results = recordError(results, message, location)
    results.errors = results.errors + 1;
    results.errorDetails{end + 1} = struct('message', message, 'location', location); %#ok<AGROW>
    fprintf('ERROR: %s (%s)\n', message, location);
end

%% --- Utility helpers ---

function tf = isStateAllowedForType(targetType, state)
    validMask = BehaviorStateCatalog.validStateMask(targetType);
    stateIndex = BehaviorStateCatalog.index(state);
    tf = validMask(stateIndex);
end

function forbiddenStates = forbiddenStatesForType(targetType)
    forbiddenStates = BehaviorStateCatalog.invalidStates(targetType);
end

function name = matrixNameForType(targetType)
    switch char(targetType)
        case char(TargetType.False)
            name = 'Bird';
        case char(TargetType.Ground)
            name = 'Ground';
        case char(TargetType.AirplaneUAV)
            name = 'Airplane';
        case char(TargetType.Quadcopter)
            name = 'Quad';
        otherwise
            name = 'Unknown';
    end
end

function count = countToken(text, token)
    matches = regexp(text, ['\<' token '\>'], 'match');
    count = numel(matches);
end
