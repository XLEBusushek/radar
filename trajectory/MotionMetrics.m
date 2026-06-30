classdef MotionMetrics
    % MotionMetrics  Метрики реализма движения цели.

    methods (Static)
        function metrics = compute(target, dt)
            positions = target.HistoryPosition;
            speeds = target.HistorySpeed;
            headings = target.HistoryHeading;
            states = target.HistoryState;

            if size(positions, 1) < 2
                metrics = MotionMetrics.emptyMetrics(target);
                return;
            end

            deltas = diff(positions, 1, 1);
            stepDistances = sqrt(sum(deltas.^2, 2));
            horizontalDeltas = deltas(:, 1:2);
            horizontalSteps = sqrt(sum(horizontalDeltas.^2, 2));
            verticalSteps = abs(deltas(:, 3));

            metrics = struct();
            metrics.ID = target.ID;
            metrics.Type = target.Type;
            metrics.TotalDistance = sum(stepDistances);
            metrics.HorizontalDistance = sum(horizontalSteps);
            metrics.VerticalDistance = sum(verticalSteps);
            metrics.MeanSpeed = mean(speeds);
            metrics.MaxSpeed = max(speeds);
            metrics.MeanAltitude = mean(positions(:, 3));
            metrics.AltitudeRange = max(positions(:, 3)) - min(positions(:, 3));

            if numel(headings) > 1
                headingRates = abs(MotionKinematics.wrapAngle(diff(headings))) / dt;
                metrics.MeanTurnRate = mean(headingRates);
                metrics.MaxTurnRate = max(headingRates);
            else
                metrics.MeanTurnRate = 0;
                metrics.MaxTurnRate = 0;
            end

            metrics.HoverTime = MotionMetrics.stateDuration(states, TargetBehaviorState.Hover, dt);
            metrics.HiddenCount = sum(arrayfun(@(s) s == TargetBehaviorState.Hidden, states));
            metrics.StraightSegmentLengthMean = MotionMetrics.meanStraightSegmentLength( ...
                positions, headings, stepDistances);
            metrics.ForbiddenStateCount = MotionMetrics.countForbiddenStates(target.Type, states);
            metrics.MeanVerticalSpeed = mean(verticalSteps / dt);
            metrics.LowAltitudeHiddenCount = MotionMetrics.hiddenCountBelowAltitude( ...
                target, positions, states, 15);
            metrics.MaxSpeedStep = max(abs(diff(speeds)));
            if size(positions, 1) > 1
                metrics.MaxAltitudeStep = max(abs(diff(positions(:, 3))));
            else
                metrics.MaxAltitudeStep = 0;
            end
        end
    end

    methods (Static, Access = private)
        function metrics = emptyMetrics(target)
            metrics = struct( ...
                'ID', target.ID, ...
                'Type', target.Type, ...
                'TotalDistance', 0, ...
                'HorizontalDistance', 0, ...
                'VerticalDistance', 0, ...
                'MeanSpeed', 0, ...
                'MaxSpeed', 0, ...
                'MeanAltitude', 0, ...
                'AltitudeRange', 0, ...
                'MeanTurnRate', 0, ...
                'MaxTurnRate', 0, ...
                'HoverTime', 0, ...
                'HiddenCount', 0, ...
                'StraightSegmentLengthMean', 0, ...
                'ForbiddenStateCount', 0, ...
                'MeanVerticalSpeed', 0, ...
                'LowAltitudeHiddenCount', 0, ...
                'MaxSpeedStep', 0, ...
                'MaxAltitudeStep', 0);
        end

        function duration = stateDuration(states, targetState, dt)
            duration = sum(arrayfun(@(s) s == targetState, states)) * dt;
        end

        function count = countForbiddenStates(targetType, states)
            forbidden = MotionMetrics.forbiddenStates(targetType);
            if isempty(forbidden)
                count = 0;
                return;
            end
            count = sum(ismember(states, forbidden));
        end

        function forbidden = forbiddenStates(targetType)
            switch char(targetType)
                case char(TargetType.Ground)
                    forbidden = [TargetBehaviorState.Hover, TargetBehaviorState.Climb, TargetBehaviorState.Descend];
                case char(TargetType.AirplaneUAV)
                    forbidden = TargetBehaviorState.Hover;
                case char(TargetType.False)
                    forbidden = TargetBehaviorState.Hover;
                otherwise
                    forbidden = TargetBehaviorState.empty(0, 1);
            end
        end

        function meanLength = meanStraightSegmentLength(positions, headings, stepDistances)
            if isempty(stepDistances)
                meanLength = 0;
                return;
            end

            segmentLengths = [];
            currentLength = stepDistances(1);
            referenceHeading = headings(1);

            for k = 2:numel(stepDistances)
                headingDelta = abs(MotionKinematics.wrapAngle(headings(k) - referenceHeading));
                if headingDelta > deg2rad(15)
                    segmentLengths(end + 1) = currentLength; %#ok<AGROW>
                    currentLength = stepDistances(k);
                    referenceHeading = headings(k);
                else
                    currentLength = currentLength + stepDistances(k);
                end
            end

            segmentLengths(end + 1) = currentLength;

            if isempty(segmentLengths)
                meanLength = 0;
            else
                meanLength = mean(segmentLengths);
            end
        end

        function count = hiddenCountBelowAltitude(target, positions, states, altitudeThreshold)
            hiddenMask = arrayfun(@(s) s == TargetBehaviorState.Hidden, states);
            count = sum(hiddenMask & positions(:, 3) <= altitudeThreshold);
        end
    end
end
