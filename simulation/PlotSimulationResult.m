function PlotSimulationResult(result)
    % PlotSimulationResult  3D-карта траекторий по результатам симуляции.

    arguments
        result (1, 1) struct
    end

    if ~isfield(result, 'Targets')
        error('PlotSimulationResult:InvalidResult', 'Result must contain Targets.');
    end

    targets = result.Targets;
    typeOrder = TargetType.allValues();
    colors = lines(numel(typeOrder));
    typeColors = containers.Map(cellstr(typeOrder), num2cell(colors, 2));

    figure('Name', 'Simulation 3D Map', 'NumberTitle', 'off');
    hold on;
    grid on;

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
                scatter3(trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), 24, color, 'filled');
            else
                plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
                    'Color', color, 'LineWidth', 1.0);
            end
        end
    end

    validLegend = isgraphics(legendHandles);
    legend(legendHandles(validLegend), legendLabels(validLegend), 'Location', 'eastoutside');
    xlabel('X, m');
    ylabel('Y, m');
    zlabel('Z, m');
    title('Simulation trajectories by target type');
    hold off;
end
