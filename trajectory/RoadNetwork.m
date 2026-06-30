classdef RoadNetwork
    % RoadNetwork  Упрощенная дорожная сеть для наземных целей.

    methods (Static)
        function headings = cardinalHeadings()
            headings = [0, pi / 2, pi, -pi / 2];
        end

        function heading = nearestHeading(angle)
            headings = RoadNetwork.cardinalHeadings();
            deltas = arrayfun(@(h) abs(MotionKinematics.wrapAngle(angle - h)), headings);
            [~, idx] = min(deltas);
            heading = headings(idx);
        end

        function heading = turnHeading(currentHeading, turnLeft)
            headings = RoadNetwork.cardinalHeadings();
            currentHeading = RoadNetwork.nearestHeading(currentHeading);
            idx = find(headings == currentHeading, 1, 'first');

            if turnLeft
                idx = idx + 1;
            else
                idx = idx - 1;
            end

            idx = mod(idx - 1, numel(headings)) + 1;
            heading = headings(idx);
        end

        function tf = isCardinalHeading(heading, toleranceDeg)
            if nargin < 2
                toleranceDeg = 10;
            end

            tolerance = deg2rad(toleranceDeg);
            headings = RoadNetwork.cardinalHeadings();
            deltas = arrayfun(@(h) abs(MotionKinematics.wrapAngle(heading - h)), headings);
            tf = any(deltas <= tolerance);
        end

        function point = pointAlongRoad(position, heading, distance)
            roadHeading = RoadNetwork.nearestHeading(heading);
            point = position;
            point(1) = position(1) + distance * cos(roadHeading);
            point(2) = position(2) + distance * sin(roadHeading);
        end

        function point = snapToRoad(position, heading)
            point = RoadNetwork.pointAlongRoad(position, heading, 0);
            point(3) = position(3);
        end

        function tf = isOnRoad(position, heading, tolerance)
            if nargin < 3
                tolerance = 5;
            end

            roadHeading = RoadNetwork.nearestHeading(heading);
            projected = RoadNetwork.pointAlongRoad(position, roadHeading, 0);
            tf = norm(position(1:2) - projected(1:2)) <= tolerance;
        end

        function waypoint = nextRoadWaypoint(position, heading, distance, altitude)
            waypoint = RoadNetwork.pointAlongRoad(position, heading, distance);
            waypoint(3) = altitude;
        end
    end
end
