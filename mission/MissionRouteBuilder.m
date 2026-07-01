classdef MissionRouteBuilder
    % MissionRouteBuilder  Построение базовых маршрутов миссий.

    methods (Static)
        function route = birdRoute(target, environment)
            spanX = diff(environment.XLimits);
            spanY = diff(environment.YLimits);
            center = [mean(environment.XLimits), mean(environment.YLimits)];
            numPoints = 3 + randi(2);

            route = zeros(numPoints, 3);
            for k = 1:numPoints
                route(k, 1) = center(1) + (-0.35 + 0.7 * rand()) * spanX;
                route(k, 2) = center(2) + (-0.35 + 0.7 * rand()) * spanY;
                route(k, 3) = 5 + 35 * rand();
            end
            route(1, :) = target.Position;
        end

        function route = groundRoute(target, environment)
            heading = RoadNetwork.nearestHeading(target.Heading);
            distance = 100 + 80 * rand();
            numPoints = 4;
            route = zeros(numPoints, 3);
            route(1, :) = target.Position;

            for k = 2:numPoints
                route(k, :) = RoadNetwork.nextRoadWaypoint( ...
                    route(k - 1, :), heading, distance, target.Position(3));
                route(k, 1) = min(max(route(k, 1), environment.XLimits(1)), environment.XLimits(2));
                route(k, 2) = min(max(route(k, 2), environment.YLimits(1)), environment.YLimits(2));
                heading = RoadNetwork.turnHeading(heading, rand() > 0.5);
            end
        end

        function route = airplaneRoute(target, environment)
            numPoints = 3 + randi(3);
            route = zeros(numPoints, 3);
            route(1, :) = target.Position;
            heading = target.Heading;

            for k = 2:numPoints
                segmentLength = 200 + 200 * rand();
                heading = heading + deg2rad(-15 + 30 * rand());
                route(k, 1) = route(k - 1, 1) + segmentLength * cos(heading);
                route(k, 2) = route(k - 1, 2) + segmentLength * sin(heading);
                route(k, 3) = 100 + 150 * rand();
                route(k, 1) = min(max(route(k, 1), environment.XLimits(1)), environment.XLimits(2));
                route(k, 2) = min(max(route(k, 2), environment.YLimits(1)), environment.YLimits(2));
            end
        end

        function route = quadcopterRoute(target, environment)
            numPoints = 3 + randi(2);
            route = zeros(numPoints, 3);
            route(1, :) = target.Position;
            heading = target.Heading;

            for k = 2:numPoints
                segmentLength = 50 + 50 * rand();
                heading = heading + deg2rad(-40 + 80 * rand());
                route(k, 1) = route(k - 1, 1) + segmentLength * cos(heading);
                route(k, 2) = route(k - 1, 2) + segmentLength * sin(heading);
                route(k, 3) = 40 + 80 * rand();
            end
        end

        function route = returnToAreaRoute(target, environment)
            center = [
                mean(environment.XLimits), ...
                mean(environment.YLimits), ...
                target.Position(3)
            ];
            route = [target.Position; center];
        end
    end
end
