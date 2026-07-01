% TestBoxSizePropagation  Validation of boxSize propagation (TZ 15.4).

function TestBoxSizePropagation()
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(projectRoot);
    setupRadarPaths();

    fprintf('=== BoxSize Propagation Validation ===\n\n');

    boxSize = [2000, 2000, 500];
    config = struct( ...
        'NumFalse', 1, ...
        'NumGround', 1, ...
        'NumAirplaneUAV', 1, ...
        'NumQuadcopter', 1, ...
        'BoxSize', boxSize, ...
        'Duration', 10, ...
        'Dt', 1, ...
        'OutputPeriod', 5, ...
        'RandomSeed', 42);

    errors = 0;
    result = SimulationEngine().run(config);

    if ~isequal(result.Config.BoxSize, boxSize)
        fprintf('ERROR: result.Config.BoxSize mismatch.\n');
        errors = errors + 1;
    end

    if ~isequal(result.Environment.BoxSize, boxSize)
        fprintf('ERROR: result.Environment.BoxSize mismatch.\n');
        errors = errors + 1;
    end

    if ~isequal(result.Environment.XLimits, [-1000, 1000])
        fprintf('ERROR: Environment.XLimits mismatch.\n');
        errors = errors + 1;
    end

    if ~isequal(result.Environment.YLimits, [-1000, 1000])
        fprintf('ERROR: Environment.YLimits mismatch.\n');
        errors = errors + 1;
    end

    try
        PlotFlightMap(result);
        fig = findobj(0, 'Type', 'figure', 'Name', 'Flight Map');
        ax3d = findAxisByTitle(fig, '3D trajectories');
        axTop = findAxisByTitle(fig, 'Top view (X-Y)');
        axAlt = findAxisByTitle(fig, 'Altitude vs time');

        if isempty(ax3d) || isempty(axTop) || isempty(axAlt)
            fprintf('ERROR: PlotFlightMap missing expected axes.\n');
            errors = errors + 1;
        else
            errors = errors + validateLimits(xlim(ax3d), [-1000, 1000], '3D X');
            errors = errors + validateLimits(ylim(ax3d), [-1000, 1000], '3D Y');
            errors = errors + validateLimits(zlim(ax3d), [0, 500], '3D Z');
            errors = errors + validateLimits(xlim(axTop), [-1000, 1000], 'Top X');
            errors = errors + validateLimits(ylim(axTop), [-1000, 1000], 'Top Y');
            errors = errors + validateLimits(ylim(axAlt), [0, 500], 'Altitude Y');
        end

        if ~isempty(fig)
            close(fig);
        end
    catch plotError
        fprintf('ERROR: PlotFlightMap failed: %s\n', plotError.message);
        errors = errors + 1;
    end

    fprintf('\nErrors: %d\n', errors);
    if errors == 0
        fprintf('BoxSize Propagation Validation PASSED\n');
    else
        fprintf('BoxSize Propagation Validation FAILED\n');
    end
end

function errors = validateLimits(actual, expected, label)
    errors = 0;
    if numel(actual) ~= 2 || any(abs(actual - expected) > 1e-6)
        fprintf('ERROR: %s limits [%.1f %.1f] expected [%.1f %.1f].\n', ...
            label, actual, expected);
        errors = 1;
    end
end

function ax = findAxisByTitle(fig, titleText)
    ax = [];
    axesHandles = findobj(fig, 'Type', 'axes');
    for k = 1:numel(axesHandles)
        titleHandle = get(axesHandles(k), 'Title');
        if isprop(titleHandle, 'String') && strcmp(char(titleHandle.String), titleText)
            ax = axesHandles(k);
            return;
        end
    end
end
