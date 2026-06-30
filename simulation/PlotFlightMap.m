function PlotFlightMap(result)
    % PlotFlightMap  3D-карта полётов всех целей.

    arguments
        result (1, 1) struct
    end

    if ~isfield(result, 'Targets')
        error('PlotFlightMap:InvalidResult', 'Result must contain Targets.');
    end

    targets = result.Targets;
    typeOrder = TargetType.allValues();
    colors = lines(numel(typeOrder));
    typeColors = containers.Map(cellstr(typeOrder), num2cell(colors, 2));

    figure('Name', 'Flight Map', 'NumberTitle', 'off');
    hold on;
    grid on;

    drawSimulationBounds(result);

    legendHandles = gobjects(numel(typeOrder), 1);
    legendLabels = strings(numel(typeOrder), 1);

    for typeIdx = 1:numel(typeOrder)
        targetType = typeOrder(typeIdx);
        typeTargets = targets(cellfun(@(t) t.Type == targetType, targets));

        if isempty(typeTargets)
            continue;
        end

        color = typeColors(char(targetType));
        legendHandles(typeIdx) = plot3(nan, nan, nan, '-', 'Color', color, 'LineWidth', 1.5);
        legendLabels(typeIdx) = char(targetType);

        for k = 1:numel(typeTargets)
            trajectory = typeTargets{k}.HistoryPosition;
            if size(trajectory, 1) < 2
                plot3(trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), 'o', ...
                    'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
                continue;
            end

            plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
                'Color', color, 'LineWidth', 1.0);

            scatter3(trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), 36, color, 'o', 'filled');
            scatter3(trajectory(end, 1), trajectory(end, 2), trajectory(end, 3), 36, color, 's', 'filled');
        end
    end

    boundsLegend = plot3(nan, nan, nan, '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

    validTypeLegend = isgraphics(legendHandles);
    allHandles = [legendHandles(validTypeLegend); boundsLegend];
    allLabels = [legendLabels(validTypeLegend); "Simulation bounds"];

    legend(allHandles, allLabels, 'Location', 'eastoutside');
    xlabel('X, m');
    ylabel('Y, m');
    zlabel('Z, m');
    title('Flight map');
    hold off;
end

function drawSimulationBounds(result)
    if isfield(result, 'Statistics') && isfield(result.Statistics, 'Environment')
        environment = result.Statistics.Environment;
        xLimits = environment.XLimits;
        yLimits = environment.YLimits;
        zLimits = environment.ZLimits;
    elseif isfield(result, 'Config') && isfield(result.Config, 'BoxSize')
        halfXY = result.Config.BoxSize(1:2) / 2;
        xLimits = [-halfXY(1), halfXY(1)];
        yLimits = [-halfXY(2), halfXY(2)];
        zLimits = [0, result.Config.BoxSize(3)];
    else
        return;
    end

    cornerX = [xLimits(1), xLimits(2), xLimits(2), xLimits(1), xLimits(1)];
    cornerY = [yLimits(1), yLimits(1), yLimits(2), yLimits(2), yLimits(1)];

    plot3(cornerX, cornerY, [zLimits(1), zLimits(1), zLimits(1), zLimits(1), zLimits(1)], ...
        '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
    plot3(cornerX, cornerY, [zLimits(2), zLimits(2), zLimits(2), zLimits(2), zLimits(2)], ...
        '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

    for k = 1:4
        plot3([cornerX(k), cornerX(k)], [cornerY(k), cornerY(k)], zLimits, ...
            '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
    end
end
