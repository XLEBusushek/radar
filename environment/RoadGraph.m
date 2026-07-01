classdef RoadGraph
    % RoadGraph  Граф дорожной сети Environment для маршрутизации наземных целей.

    properties (Constant)
        ROAD_WIDTH = 6
    end

    methods (Static)
        function graph = fromEnvironment(environment)
            segments = environment.RoadNetwork.Segments;
            [nodes, nodeKeys] = RoadGraph.extractNodes(segments);
            edges = RoadGraph.buildEdges(segments, nodes, nodeKeys);
            adjacency = RoadGraph.buildAdjacency(numel(nodes), edges);

            graph = struct();
            graph.Nodes = nodes;
            graph.NodeKeys = nodeKeys;
            graph.Edges = edges;
            graph.Adjacency = adjacency;
            graph.Segments = segments;
            graph.IntersectionNodes = RoadGraph.markIntersections(adjacency);
        end

        function info = locatePosition(graph, position, environment)
            if nargin < 3
                environment = struct('RoadNetwork', struct('Segments', graph.Segments));
            end

            roadInfo = Environment.findNearestRoad(environment, position);
            bestSegmentIndex = roadInfo.SegmentIndex;
            bestPoint = roadInfo.Point(1:2);
            bestDistance = roadInfo.Distance;
            segments = graph.Segments;

            startNode = RoadGraph.nearestNode(graph.Nodes, segments(bestSegmentIndex, 1:2));
            endNode = RoadGraph.nearestNode(graph.Nodes, segments(bestSegmentIndex, 3:4));
            distToStart = norm(bestPoint - graph.Nodes(startNode, :));
            distToEnd = norm(bestPoint - graph.Nodes(endNode, :));
            if distToEnd < distToStart
                bestNode = endNode;
            else
                bestNode = startNode;
            end

            info = struct( ...
                'SegmentIndex', bestSegmentIndex, ...
                'NodeIndex', bestNode, ...
                'Point', bestPoint, ...
                'Distance', bestDistance);
        end

        function destinationNode = selectDestination(graph, startNode, stream, minDistance)
            if nargin < 4
                minDistance = 150;
            end

            nodeCount = size(graph.Nodes, 1);
            startPoint = graph.Nodes(startNode, :);
            candidates = find(vecnorm(graph.Nodes - startPoint, 2, 2) >= minDistance)';

            if isempty(candidates)
                candidates = setdiff(1:nodeCount, startNode);
            end

            if isempty(candidates)
                destinationNode = startNode;
                return;
            end

            destinationNode = candidates(stream.randi(numel(candidates)));
        end

        function nodePath = findPath(graph, startNode, goalNode)
            if startNode == goalNode
                nodePath = startNode;
                return;
            end

            nodeCount = size(graph.Nodes, 1);
            openSet = startNode;
            cameFrom = zeros(nodeCount, 1);
            gScore = inf(nodeCount, 1);
            gScore(startNode) = 0;
            fScore = inf(nodeCount, 1);
            fScore(startNode) = norm(graph.Nodes(startNode, :) - graph.Nodes(goalNode, :));

            while ~isempty(openSet)
                [~, bestIdx] = min(fScore(openSet));
                current = openSet(bestIdx);
                openSet(bestIdx) = [];

                if current == goalNode
                    nodePath = RoadGraph.reconstructPath(cameFrom, current);
                    return;
                end

                for edgeIdx = graph.Adjacency{current}
                    edge = graph.Edges(edgeIdx);
                    neighbor = edge.To;
                    tentativeG = gScore(current) + edge.Length;

                    if tentativeG < gScore(neighbor)
                        cameFrom(neighbor) = current;
                        gScore(neighbor) = tentativeG;
                        fScore(neighbor) = tentativeG + ...
                            norm(graph.Nodes(neighbor, :) - graph.Nodes(goalNode, :));
                        if ~any(openSet == neighbor)
                            openSet(end + 1) = neighbor; %#ok<AGROW>
                        end
                    end
                end
            end

            nodePath = [startNode, goalNode];
        end

        function route = buildMissionRoute(graph, nodePath, environment, laneOffset)
            if numel(nodePath) < 2
                point = graph.Nodes(nodePath(1), :);
                route = RoadGraph.pointToWaypoint(point, 0, environment, laneOffset);
                return;
            end

            route = zeros(0, 3);
            sampleSpacing = 60;

            for stepIdx = 1:(numel(nodePath) - 1)
                fromNode = nodePath(stepIdx);
                toNode = nodePath(stepIdx + 1);
                edge = RoadGraph.edgeBetween(graph, fromNode, toNode);
                segment = graph.Segments(edge.SegmentIndex, :);
                heading = edge.Heading;
                startPoint = graph.Nodes(fromNode, :);
                endPoint = graph.Nodes(toNode, :);
                segmentLength = norm(endPoint - startPoint);
                numSamples = max(1, floor(segmentLength / sampleSpacing));

                for sampleIdx = 1:numSamples
                    t = sampleIdx / numSamples;
                    point2d = startPoint + t * (endPoint - startPoint);
                    route(end + 1, :) = RoadGraph.pointToWaypoint(point2d, heading, environment, laneOffset); %#ok<AGROW>
                end

                route(end + 1, :) = RoadGraph.pointToWaypoint(endPoint, heading, environment, laneOffset);
            end

            route = RoadGraph.dedupeRoute(route);
        end

        function laneOffset = pickLaneOffset(stream)
            if stream.rand() > 0.5
                laneOffset = 2;
            else
                laneOffset = -2;
            end
        end

        function waypoint = pointToWaypoint(point2d, heading, environment, laneOffset)
            if heading == 0 && nargin >= 2
                heading = 0;
            end

            if heading ~= 0 || laneOffset ~= 0
                offsetPoint = RoadGraph.applyLaneOffset(point2d, heading, laneOffset);
            else
                offsetPoint = point2d;
            end

            terrainHeight = environment.Terrain.Height(offsetPoint(1), offsetPoint(2));
            waypoint = [offsetPoint, terrainHeight + 0.8];
        end

        function offsetPoint = applyLaneOffset(point2d, heading, laneOffset)
            normal = [-sin(heading), cos(heading)];
            offsetPoint = point2d + laneOffset * normal;
        end

        function finalOffset = clampFinalLaneOffset(baseLaneOffset, laneOffsetNoise, roadWidth)
            if nargin < 3
                roadWidth = RoadGraph.ROAD_WIDTH;
            end

            halfWidth = roadWidth / 2;
            laneOffsetNoise = min(max(laneOffsetNoise, -0.7), 0.7);
            finalOffset = min(max(baseLaneOffset + laneOffsetNoise, -halfWidth), halfWidth);
        end

        function lateralOffset = lateralOffsetFromRoad(environment, position)
            roadInfo = Environment.findNearestRoad(environment, position);
            heading = roadInfo.Heading;
            delta = position(1:2) - roadInfo.Point(1:2);
            normal = [-sin(heading), cos(heading)];
            lateralOffset = dot(delta, normal);
        end

        function dist = distanceToNearestIntersection(graph, position)
            intersectionNodes = find(graph.IntersectionNodes);
            if isempty(intersectionNodes)
                dist = inf;
                return;
            end

            dists = vecnorm(graph.Nodes(intersectionNodes, :) - position(1:2), 2, 2);
            dist = min(dists);
        end

        function tf = isOnRoadNetwork(environment, position, tolerance)
            if nargin < 3
                tolerance = 8;
            end

            roadInfo = Environment.findNearestRoad(environment, position);
            tf = roadInfo.Distance <= tolerance;
        end

        function tf = isRouteOnRoads(environment, route, tolerance)
            if nargin < 3
                tolerance = 6;
            end

            tf = true;
            for pointIdx = 1:size(route, 1)
                if ~RoadGraph.isOnRoadNetwork(environment, route(pointIdx, :), tolerance)
                    tf = false;
                    return;
                end
            end
        end

        function tf = isRouteConnected(environment, route, maxStep)
            if nargin < 3
                maxStep = 80;
            end

            graph = RoadGraph.fromEnvironment(environment);
            tf = true;

            for pointIdx = 2:size(route, 1)
                prevPoint = route(pointIdx - 1, 1:2);
                currentPoint = route(pointIdx, 1:2);
                stepDistance = norm(currentPoint - prevPoint);

                if stepDistance > maxStep
                    tf = false;
                    return;
                end

                prevInfo = RoadGraph.locatePosition(graph, [prevPoint, 0], environment);
                currentInfo = RoadGraph.locatePosition(graph, [currentPoint, 0], environment);
                if prevInfo.SegmentIndex ~= currentInfo.SegmentIndex
                    prevNode = prevInfo.NodeIndex;
                    currentNode = currentInfo.NodeIndex;
                    if prevNode ~= currentNode && ~RoadGraph.nodesConnected(graph, prevNode, currentNode)
                        tf = false;
                        return;
                    end
                end
            end
        end

        function intersectionNodes = intersectionIndices(graph)
            intersectionNodes = find(graph.IntersectionNodes);
        end
    end

    methods (Static, Access = private)
        function [nodes, nodeKeys] = extractNodes(segments)
            tolerance = 1.0;
            nodes = zeros(0, 2);
            nodeKeys = {};

            for segmentIdx = 1:size(segments, 1)
                segment = segments(segmentIdx, :);
                endpoints = [segment(1:2); segment(3:4)];

                for endpointIdx = 1:2
                    point = endpoints(endpointIdx, :);
                    nodeIndex = RoadGraph.findOrCreateNode(nodes, nodeKeys, point, tolerance);
                    if nodeIndex == 0
                        nodes(end + 1, :) = point; %#ok<AGROW>
                        nodeKeys{end + 1} = RoadGraph.pointKey(point); %#ok<AGROW>
                    end
                end
            end
        end

        function nodeIndex = findOrCreateNode(nodes, nodeKeys, point, tolerance)
            nodeIndex = 0;
            if isempty(nodes)
                return;
            end

            distances = vecnorm(nodes - point, 2, 2);
            [minDistance, nearestIdx] = min(distances);
            if minDistance <= tolerance
                nodeIndex = nearestIdx;
            end
        end

        function key = pointKey(point)
            key = sprintf('%.1f_%.1f', point(1), point(2));
        end

        function edges = buildEdges(segments, nodes, ~)
            edgeCount = size(segments, 1);
            edges = repmat(RoadGraph.emptyEdge(), edgeCount * 2, 1);
            edgeIdx = 0;
            tolerance = 1.0;

            for segmentIdx = 1:edgeCount
                segment = segments(segmentIdx, :);
                startNode = RoadGraph.nearestNode(nodes, segment(1:2), tolerance);
                endNode = RoadGraph.nearestNode(nodes, segment(3:4), tolerance);
                lengthValue = norm(segment(3:4) - segment(1:2));
                heading = atan2(segment(4) - segment(2), segment(3) - segment(1));

                edgeIdx = edgeIdx + 1;
                edges(edgeIdx) = RoadGraph.makeEdge(startNode, endNode, segmentIdx, lengthValue, heading);
                edgeIdx = edgeIdx + 1;
                edges(edgeIdx) = RoadGraph.makeEdge(endNode, startNode, segmentIdx, lengthValue, heading + pi);
            end
        end

        function edge = makeEdge(fromNode, toNode, segmentIndex, lengthValue, heading)
            edge = struct( ...
                'From', fromNode, ...
                'To', toNode, ...
                'SegmentIndex', segmentIndex, ...
                'Length', lengthValue, ...
                'Heading', heading);
        end

        function edge = emptyEdge()
            edge = struct('From', 0, 'To', 0, 'SegmentIndex', 0, 'Length', 0, 'Heading', 0);
        end

        function adjacency = buildAdjacency(nodeCount, edges)
            adjacency = cell(nodeCount, 1);
            for edgeIdx = 1:numel(edges)
                fromNode = edges(edgeIdx).From;
                adjacency{fromNode}(end + 1) = edgeIdx; %#ok<AGROW>
            end
        end

        function intersectionMask = markIntersections(adjacency)
            intersectionMask = false(numel(adjacency), 1);
            for nodeIdx = 1:numel(adjacency)
                intersectionMask(nodeIdx) = numel(adjacency{nodeIdx}) >= 3;
            end
        end

        function nodeIndex = nearestNode(nodes, point, tolerance)
            if nargin < 3
                tolerance = 1.0;
            end

            distances = vecnorm(nodes - point, 2, 2);
            [minDistance, nodeIndex] = min(distances);
            if minDistance > tolerance
                nodeIndex = 1;
            end
        end

        function edge = edgeBetween(graph, fromNode, toNode)
            for edgeIdx = graph.Adjacency{fromNode}
                edge = graph.Edges(edgeIdx);
                if edge.To == toNode
                    return;
                end
            end

            edge = graph.Edges(1);
        end

        function nodePath = reconstructPath(cameFrom, current)
            nodePath = current;
            while cameFrom(current) ~= 0
                current = cameFrom(current);
                nodePath = [current, nodePath]; %#ok<AGROW>
            end
        end

        function route = dedupeRoute(route)
            if size(route, 1) < 2
                return;
            end

            keepMask = true(size(route, 1), 1);
            for pointIdx = 2:size(route, 1)
                if norm(route(pointIdx, 1:2) - route(pointIdx - 1, 1:2)) < 1.0
                    keepMask(pointIdx) = false;
                end
            end
            route = route(keepMask, :);
        end

        function tf = nodesConnected(graph, nodeA, nodeB)
            path = RoadGraph.findPath(graph, nodeA, nodeB);
            tf = ~isempty(path);
        end
    end
end
