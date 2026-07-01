function PlotFlightMap(result, plotOptions)
    % PlotFlightMap  3D-карта полётов и дополнительные проекции.

    arguments
        result (1, 1) struct
        plotOptions.showMissionWaypoints (1, 1) logical = true
        plotOptions.showRoads (1, 1) logical = true
        plotOptions.showTreeZones (1, 1) logical = true
        plotOptions.showInspectionZones (1, 1) logical = true
        plotOptions.showPatrolZones (1, 1) logical = true
        plotOptions.showBounds (1, 1) logical = true
    end

    showMissionWaypoints = plotOptions.showMissionWaypoints;
    environment = resolveEnvironment(result);

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
    if plotOptions.showBounds
        drawSimulationBounds(result, ax3d);
    end
    drawEnvironmentLayers(environment, ax3d, plotOptions, 'plot3');
    [legendHandles, legendLabels] = plotTrajectories(targets, typeOrder, typeColors, ax3d, 'plot3', showMissionWaypoints);
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
    drawEnvironmentLayers(environment, axTop, plotOptions, 'xy');
    plotTrajectories(targets, typeOrder, typeColors, axTop, 'xy', showMissionWaypoints);
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

function [legendHandles, legendLabels] = plotTrajectories(targets, typeOrder, typeColors, ax, mode, showMissionWaypoints)
    if nargin < 6
        showMissionWaypoints = true;
    end
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
                    if showMissionWaypoints
                        plotMissionRoute(typeTargets{k}, ax, color, 'plot3');
                    end
                case 'xy'
                    plot(ax, trajectory(:, 1), trajectory(:, 2), 'Color', color, 'LineWidth', 1.0);
                    scatter(ax, trajectory(1, 1), trajectory(1, 2), 36, color, 'o', 'filled');
                    scatter(ax, trajectory(end, 1), trajectory(end, 2), 36, color, 's', 'filled');
                    plotTargetWaypoints(typeTargets{k}, ax, color, 'xy');
                    if showMissionWaypoints
                        plotMissionRoute(typeTargets{k}, ax, color, 'xy');
                    end
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

function plotMissionRoute(target, ax, color, mode)
    if ~isfield(target, 'MissionCommand') || isempty(target.MissionCommand) || ...
            ~isfield(target.MissionCommand, 'Status') || ...
            target.MissionCommand.Status ~= MissionStatus.Executing
        return;
    end

    route = target.MissionRoute;
    if isempty(route) || size(route, 1) < 2 || any(~isfinite(route(:)))
        return;
    end

    currentWaypoint = target.MissionCommand.CurrentWaypoint;

    switch mode
        case 'plot3'
            plot3(ax, route(:, 1), route(:, 2), route(:, 3), '--', ...
                'Color', color, 'LineWidth', 1.0);
            scatter3(ax, route(:, 1), route(:, 2), route(:, 3), ...
                40, color, 'p', 'LineWidth', 0.8);
            if all(isfinite(currentWaypoint))
                scatter3(ax, currentWaypoint(1), currentWaypoint(2), currentWaypoint(3), ...
                    90, color, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
            end
        case 'xy'
            plot(ax, route(:, 1), route(:, 2), '--', ...
                'Color', color, 'LineWidth', 1.0);
            scatter(ax, route(:, 1), route(:, 2), ...
                40, color, 'p', 'LineWidth', 0.8);
            if all(isfinite(currentWaypoint))
                scatter(ax, currentWaypoint(1), currentWaypoint(2), ...
                    90, color, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
            end
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

function environment = resolveEnvironment(result)
    environment = [];
    if isfield(result, 'Environment') && isstruct(result.Environment)
        environment = result.Environment;
    end
end

function drawEnvironmentLayers(environment, ax, plotOptions, mode)
    if isempty(environment)
        return;
    end

    if plotOptions.showRoads && isfield(environment, 'RoadNetwork')
        drawRoadNetwork(environment.RoadNetwork, environment.Terrain, ax, mode);
    end

    if plotOptions.showTreeZones && isfield(environment, 'TreeZones')
        drawRadialZones(environment.TreeZones, ax, mode, [0.2, 0.55, 0.2], 'Tree zone');
    end

    if plotOptions.showInspectionZones && isfield(environment, 'InspectionZones')
        drawRadialZones(environment.InspectionZones, ax, mode, [0.85, 0.45, 0.1], 'Inspection zone');
    end

    if plotOptions.showPatrolZones && isfield(environment, 'PatrolZones')
        drawPatrolZones(environment.PatrolZones, environment.Terrain, ax, mode);
    end
end

function drawRoadNetwork(roadNetwork, terrain, ax, mode)
    segments = roadNetwork.Segments;
    for segmentIdx = 1:size(segments, 1)
        segment = segments(segmentIdx, :);
        switch mode
            case 'plot3'
                z1 = terrain.Height(segment(1), segment(2));
                z2 = terrain.Height(segment(3), segment(4));
                plot3(ax, [segment(1), segment(3)], [segment(2), segment(4)], [z1, z2], ...
                    '-', 'Color', [0.35, 0.35, 0.35], 'LineWidth', 1.2);
            case 'xy'
                plot(ax, [segment(1), segment(3)], [segment(2), segment(4)], ...
                    '-', 'Color', [0.35, 0.35, 0.35], 'LineWidth', 1.2);
        end
    end
end

function drawRadialZones(zones, ax, mode, color, ~)
    theta = linspace(0, 2 * pi, 48);
    for zoneIdx = 1:numel(zones)
        zone = zones(zoneIdx);
        circleX = zone.Center(1) + zone.Radius * cos(theta);
        circleY = zone.Center(2) + zone.Radius * sin(theta);
        switch mode
            case 'plot3'
                plot3(ax, circleX, circleY, zeros(size(circleX)), '--', ...
                    'Color', color, 'LineWidth', 0.9);
            case 'xy'
                plot(ax, circleX, circleY, '--', 'Color', color, 'LineWidth', 0.9);
        end
    end
end

function drawPatrolZones(patrolZones, terrain, ax, mode)
    color = [0.2, 0.35, 0.75];
    for zoneIdx = 1:numel(patrolZones)
        polygon = patrolZones(zoneIdx).Polygon;
        closedPolygon = [polygon; polygon(1, :)];
        switch mode
            case 'plot3'
                z = terrain.Height(mean(polygon(:, 1)), mean(polygon(:, 2)));
                plot3(ax, closedPolygon(:, 1), closedPolygon(:, 2), repmat(z, size(closedPolygon, 1), 1), ...
                    '--', 'Color', color, 'LineWidth', 1.0);
            case 'xy'
                plot(ax, closedPolygon(:, 1), closedPolygon(:, 2), '--', ...
                    'Color', color, 'LineWidth', 1.0);
        end
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
