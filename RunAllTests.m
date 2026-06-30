% RunAllTests  Запуск всех тестов проекта (ТЗ №8).

function RunAllTests()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    testPlan = {
        'TestTargetProfiles', false
        'TestDecisionEngine', false
        'TestBehaviorPlanner', false
        'TestTrajectoryGenerator', false
        'TestSimulationEngine', false
        'TestRadarOutputExporter', false
        'TestMotionLogic', false
        'TestPhasedTargetAdapter', true
        'TestMainScenario', false
    };

    fprintf('=== Running All Tests ===\n\n');

    failedCount = 0;
    skippedCount = 0;
    passedCount = 0;

    for testIdx = 1:size(testPlan, 1)
        testName = testPlan{testIdx, 1};
        allowSkip = testPlan{testIdx, 2};

        fprintf('[%d/%d] %s\n', testIdx, size(testPlan, 1), testName);

        [status, outputText] = runSingleTest(testName);

        if ~isempty(outputText)
            fprintf('%s', outputText);
            if ~endsWith(strtrim(outputText), newline)
                fprintf('\n');
            end
        end

        switch status
            case 'passed'
                passedCount = passedCount + 1;
                fprintf('  -> PASSED\n\n');
            case 'skipped'
                skippedCount = skippedCount + 1;
                fprintf('  -> SKIPPED\n\n');
            otherwise
                failedCount = failedCount + 1;
                fprintf('  -> FAILED\n\n');
        end
    end

    fprintf('=== Test Summary ===\n');
    fprintf('Passed : %d\n', passedCount);
    fprintf('Skipped: %d\n', skippedCount);
    fprintf('Failed : %d\n', failedCount);
    fprintf('\n');

    if failedCount == 0
        fprintf('ALL TESTS PASSED\n');
    else
        fprintf('SOME TESTS FAILED\n');
    end
end

function [status, outputText] = runSingleTest(testName)
    status = 'failed';
    outputText = '';

    try
        outputText = evalc(testName);

        if contains(outputText, 'FAILED', 'IgnoreCase', true)
            return;
        end

        if contains(outputText, 'Skipping', 'IgnoreCase', true) || ...
                contains(outputText, 'not available', 'IgnoreCase', true)
            status = 'skipped';
            return;
        end

        if contains(outputText, 'PASSED', 'IgnoreCase', true)
            status = 'passed';
            return;
        end

    catch runError
        outputText = sprintf('%s\nERROR: %s\n', outputText, runError.message);
    end
end
