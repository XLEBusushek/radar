classdef GroundMissionPlanner
    % GroundMissionPlanner  Миссии наземных целей по RoadNetwork из Environment.

    methods (Static)
        function command = createMission(target, environment)
            graph = RoadGraph.fromEnvironment(environment);
            stream = RandStream('mt19937ar', 'Seed', round(environment.RandomSeed + target.ID));

            locate = RoadGraph.locatePosition(graph, target.Position, environment);
            destinationNode = RoadGraph.selectDestination(graph, locate.NodeIndex, stream);
            nodePath = RoadGraph.findPath(graph, locate.NodeIndex, destinationNode);
            laneOffset = RoadGraph.pickLaneOffset(stream);
            route = RoadGraph.buildMissionRoute(graph, nodePath, environment, laneOffset);

            if size(route, 1) < 2
                route = [route; route(end, :)];
            end

            holdTime = MissionTypeCatalog.minimumHoldTime(target.Type);
            cruiseSpeed = 15 + 10 * stream.rand();

            command = MissionCommand.create( ...
                'MissionType', MissionType.FollowRoadRoute, ...
                'MissionRoute', route, ...
                'DesiredMissionSpeed', cruiseSpeed, ...
                'DesiredMissionAltitude', route(1, 3), ...
                'MissionHoldTime', holdTime, ...
                'MissionPriority', 1, ...
                'MissionReason', 'Ground road graph route', ...
                'MissionStartTime', target.MissionTime);

            command.GroundLaneOffset = laneOffset;
            command.CurrentRoadSegmentIndex = locate.SegmentIndex;
            command.DestinationNodeIndex = destinationNode;
            command.SourceNodeIndex = locate.NodeIndex;
            command.RoadNodePath = nodePath;
        end
    end
end
