function PlotFlightMap(result)
    % PlotFlightMap  3D-карта полётов и дополнительные проекции.

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
    timeAxis = buildTimeAxis(targets, result);

    figure('Name', 'Flight Map', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 900]);
    tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax3d = nexttile;
    hold(ax3d, 'on');
    grid(ax3d, 'on');
    drawSimulationBounds(result, ax3d);
    [legendHandles, legendLabels] = plotTrajectories(targets, typeOrder, typeColors, ax3d, 'plot3');
    boundsLegend = plot3(ax3d, nan, nan, nan, '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
    validLegend = isgraphics(legendHandles);
    legend(ax3d, [legendHandles(validLegend); boundsLegend], ...
        [legendLabels(validLegend); "Simulation bounds"], 'Location', 'eastoutside');
    xlabel(ax3d, 'X, m');
    ylabel(ax3d, 'Y, m');
    zlabel(ax3d, 'Z, m');
    title(ax3d, '3D trajectories');
    view(ax3d, 3);
    axis(ax3d, 'vis3d');
    if isfield(result, 'Config') && isfield(result.Config, 'BoxSize')
        zlim(ax3d, [0, result.Config.BoxSize(3)]);
    end
    hold(ax3d, 'off');

    axTop = nexttile;
    hold(axTop, 'on');
    grid(axTop, 'on');
    plotTrajectories(targets, typeOrder, typeColors, axTop, 'xy');
    xlabel(axTop, 'X, m');
    ylabel(axTop, 'Y, m');
    title(axTop, 'Top view (X-Y)');
    axis(axTop, 'equal');
    hold(axTop, 'off');

    axAlt = nexttile;
    hold(axAlt, 'on');
    grid(axAlt, 'on');
    plotTimeSeries(targets, timeAxis, typeColors, axAlt, 'altitude');
    xlabel(axAlt, 'Time, s');
    ylabel(axAlt, 'Z, m');
    title(axAlt, 'Altitude vs time');
    hold(axAlt, 'off');

    axSpeed = nexttile;
    hold(axSpeed, 'on');
    grid(axSpeed, 'on');
    plotTimeSeries(targets, timeAxis, typeColors, axSpeed, 'speed');
    xlabel(axSpeed, 'Time, s');
    ylabel(axSpeed, 'Speed, m/s');
    title(axSpeed, 'Speed vs time');
    hold(axSpeed, 'off');
end

function timeAxis = buildTimeAxis(targets, result)
    if ~isempty(targets) && ~isempty(targets{1}.HistorySpeed)
        timeAxis = (0:(numel(targets{1}.HistorySpeed) - 1)) * result.Config.Dt;
    else
        timeAxis = 0;
    end
end

function [legendHandles, legendLabels] = plotTrajectories(targets, typeOrder, typeColors, ax, mode)
    legendHandles = gobjects(numel(typeOrder), 1);
    legendLabels = strings(numel(typeOrder), 1);

    for typeIdx = 1:numel(typeOrder)
        targetType = typeOrder(typeIdx);
        typeTargets = targets(cellfun(@(t) t.Type == targetType, targets));
        if isempty(typeTargets)
            continue;
        end

        color = typeColors(char(targetType));
        switch mode
            case 'plot3'
                legendHandles(typeIdx) = plot3(ax, nan, nan, nan, '-', 'Color', color, 'LineWidth', 1.5);
            case 'xy'
                legendHandles(typeIdx) = plot(ax, nan, nan, '-', 'Color', color, 'LineWidth', 1.5);
        end
        legendLabels(typeIdx) = char(targetType);

        for k = 1:numel(typeTargets)
            trajectory = typeTargets{k}.HistoryPosition;
            if size(trajectory, 1) < 2
                if strcmp(mode, 'plot3')
                    scatter3(ax, trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), 24, color, 'filled');
                else
                    scatter(ax, trajectory(1, 1), trajectory(1, 2), 24, color, 'filled');
                end
                continue;
            end

            switch mode
                case 'plot3'
                    plot3(ax, trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
                        'Color', color, 'LineWidth', 1.0);
                    scatter3(ax, trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), 36, color, 'o', 'filled');
                    scatter3(ax, trajectory(end, 1), trajectory(end, 2), trajectory(end, 3), 36, color, 's', 'filled');
                    plotTargetWaypoints(typeTargets{k}, ax, color, 'plot3');
                case 'xy'
                    plot(ax, trajectory(:, 1), trajectory(:, 2), 'Color', color, 'LineWidth', 1.0);
                    scatter(ax, trajectory(1, 1), trajectory(1, 2), 36, color, 'o', 'filled');
                    scatter(ax, trajectory(end, 1), trajectory(end, 2), 36, color, 's', 'filled');
                    plotTargetWaypoints(typeTargets{k}, ax, color, 'xy');
            end
        end
    end
end

function plotTimeSeries(targets, timeAxis, typeColors, ax, seriesType)
    for k = 1:numel(targets)
        target = targets{k};
        color = typeColors(char(target.Type));
        seriesLength = min(numel(timeAxis), numel(target.HistorySpeed));

        switch seriesType
            case 'altitude'
                values = target.HistoryPosition(1:seriesLength, 3);
            case 'speed'
                values = target.HistorySpeed(1:seriesLength);
        end

        plot(ax, timeAxis(1:seriesLength), values, 'Color', color, 'LineWidth', 1.0);
    end
end

function plotTargetWaypoints(target, ax, color, mode)
    if isempty(target.HistoryWaypoint)
        if all(isfinite(target.TargetWaypoint))
            waypoints = target.TargetWaypoint;
        else
            return;
        end
    else
        waypoints = target.HistoryWaypoint;
        if all(isfinite(target.TargetWaypoint))
            waypoints = [waypoints; target.TargetWaypoint];
        end
    end

    switch mode
        case 'plot3'
            scatter3(ax, waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), ...
                28, color, 'd', 'LineWidth', 0.8);
        case 'xy'
            scatter(ax, waypoints(:, 1), waypoints(:, 2), ...
                28, color, 'd', 'LineWidth', 0.8);
    end
end

function drawSimulationBounds(result, ax)
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

    plot3(ax, cornerX, cornerY, repmat(zLimits(1), 1, 5), '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
    plot3(ax, cornerX, cornerY, repmat(zLimits(2), 1, 5), '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

    for k = 1:4
        plot3(ax, [cornerX(k), cornerX(k)], [cornerY(k), cornerY(k)], zLimits, ...
            '--', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
    end
end
