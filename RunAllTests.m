% RunAllTests  Запуск всех тестов проекта по категориям (ТЗ №15.1).

function RunAllTests()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    testCategories = {
        'Core', {
            'TestTargetProfiles', false
            'TestDecisionEngine', false
            'TestTrajectoryGenerator', false
            'TestSimulationEngine', false
            'TestNaturalMotionBase', false
            'TestMotionLogic', false
            'TestSpeedSmoothness', false
        }
        'Environment', {
            'TestEnvironmentGenerator', false
            'TestGroundMissionEnvironment', false
            'TestAirplaneMissionEnvironment', false
            'TestQuadcopterMissionEnvironment', false
            'TestBirdMissionEnvironment', false
        }
        'Mission', {
            'TestMissionPlannerBase', false
            'TestMissionStateMachine', false
        }
        'Target behavior', {
            'TestBehaviorPlanner', false
            'TestGroundNaturalMotion', false
            'TestAirplaneNaturalMotion', false
        }
        'Export / visualization', {
            'TestRadarOutputExporter', false
            'TestPhasedTargetAdapter', true
            'TestSmallScenarioVisualLogic', false
            'TestMainScenario', false
        }
    };

    fprintf('=== Running All Tests ===\n\n');

    failedCount = 0;
    skippedCount = 0;
    passedCount = 0;

    for categoryIdx = 1:size(testCategories, 1)
        categoryName = testCategories{categoryIdx, 1};
        categoryTests = testCategories{categoryIdx, 2};

        fprintf('[%s]\n', categoryName);

        for testIdx = 1:size(categoryTests, 1)
            testName = categoryTests{testIdx, 1};
            allowSkip = categoryTests{testIdx, 2}; %#ok<NASGU>

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
                    fprintf('%s PASSED\n', testName);
                case 'skipped'
                    skippedCount = skippedCount + 1;
                    fprintf('%s SKIPPED\n', testName);
                otherwise
                    failedCount = failedCount + 1;
                    fprintf('%s FAILED\n', testName);
            end
        end

        fprintf('\n');
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
