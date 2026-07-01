classdef MissionStateMachine
    % MissionStateMachine  Управление жизненным циклом миссии.

    methods (Static)
        function [target, missionCommand] = update(target, missionCommand, environment, dt)
            arguments
                target (1, 1) RadarTargetModel
                missionCommand (1, 1) struct
                environment (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            missionCommand.StatusTime = missionCommand.StatusTime + dt;
            missionCommand = MissionStateMachine.refreshMetrics(missionCommand, target.Position);

            if target.IsHidden && missionCommand.Status == MissionStatus.Executing
                if ~MissionStateMachine.isInspectionMission(missionCommand)
                    missionCommand = MissionStateMachine.transition( ...
                        missionCommand, MissionStatus.Paused, target.MissionTime);
                    target = target.updateMissionCommand(missionCommand);
                    return;
                end
            end

            if ~target.IsHidden && missionCommand.Status == MissionStatus.Paused
                missionCommand = MissionStateMachine.transition( ...
                    missionCommand, MissionStatus.Executing, target.MissionTime);
                target = target.updateMissionCommand(missionCommand);
                return;
            end

            switch missionCommand.Status
                case MissionStatus.Created
                    missionCommand = MissionStateMachine.transition( ...
                        missionCommand, MissionStatus.Planning, target.MissionTime);

                case MissionStatus.Planning
                    if MissionStateMachine.isInvalidRoute(missionCommand)
                        missionCommand = MissionStateMachine.transition( ...
                            missionCommand, MissionStatus.Cancelled, target.MissionTime, ...
                            'CancelReason', 'Invalid mission route');
                    else
                        missionCommand = MissionStateMachine.transition( ...
                            missionCommand, MissionStatus.Executing, target.MissionTime);
                    end

                case MissionStatus.Executing
                    if MissionStateMachine.isBirdMission(missionCommand)
                        missionCommand = MissionStateMachine.updateBirdMission( ...
                            missionCommand, target, environment, dt);
                    elseif MissionStateMachine.isInspectionMission(missionCommand)
                        missionCommand = MissionStateMachine.updateInspectionMission( ...
                            missionCommand, target, dt);
                    else
                        threshold = MissionStateMachine.waypointThreshold(target.Type, missionCommand);
                        missionCommand = MissionStateMachine.tryAdvanceWaypoint( ...
                            missionCommand, target.Position, threshold);
                    end
                    missionCommand = MissionStateMachine.refreshMetrics(missionCommand, target.Position);

                    if missionCommand.IsMissionComplete
                        missionCommand = MissionStateMachine.transition( ...
                            missionCommand, MissionStatus.Completed, target.MissionTime, ...
                            'CompletionReason', 'All waypoints reached');
                    end

                case {MissionStatus.Paused, MissionStatus.Completed, MissionStatus.Cancelled}
                    % Handled by MissionPlanner or hidden-state logic.
            end

            target = target.updateMissionCommand(missionCommand);
        end

        function tf = isTerminal(missionCommand)
            tf = isstruct(missionCommand) && ...
                isfield(missionCommand, 'Status') && ...
                (missionCommand.Status == MissionStatus.Completed || ...
                missionCommand.Status == MissionStatus.Cancelled);
        end

        function tf = isOperational(missionCommand)
            tf = isstruct(missionCommand) && ...
                isfield(missionCommand, 'Status') && ...
                (missionCommand.Status == MissionStatus.Executing || ...
                missionCommand.Status == MissionStatus.Paused);
        end
    end

    methods (Static, Access = private)
        function missionCommand = transition(missionCommand, newStatus, missionTime, varargin)
            parser = inputParser;
            parser.addParameter('CompletionReason', missionCommand.CompletionReason);
            parser.addParameter('CancelReason', missionCommand.CancelReason);
            parser.parse(varargin{:});

            if missionCommand.Status == newStatus
                return;
            end

            if ~MissionStatusCatalog.isTransitionAllowed(missionCommand.Status, newStatus)
                warning('MissionStateMachine:InvalidTransition', ...
                    'Ignored mission transition %s -> %s.', ...
                    char(missionCommand.Status), char(newStatus));
                return;
            end

            missionCommand.PreviousStatus = missionCommand.Status;
            missionCommand.Status = newStatus;
            missionCommand.StatusStartTime = missionTime;
            missionCommand.StatusTime = 0;
            missionCommand.CompletionReason = parser.Results.CompletionReason;
            missionCommand.CancelReason = parser.Results.CancelReason;

            if newStatus == MissionStatus.Completed
                missionCommand.IsMissionComplete = true;
                missionCommand.Progress = 1;
                missionCommand.CompletionReason = parser.Results.CompletionReason;
            end

            if newStatus == MissionStatus.Cancelled
                missionCommand.IsMissionComplete = true;
                missionCommand.CancelReason = parser.Results.CancelReason;
            end
        end

        function missionCommand = refreshMetrics(missionCommand, position)
            if ~isfield(missionCommand, 'CurrentWaypoint')
                return;
            end

            if all(isfinite(missionCommand.CurrentWaypoint))
                missionCommand.DistanceToCurrentWaypoint = norm( ...
                    position(1:2) - missionCommand.CurrentWaypoint(1:2));
            else
                missionCommand.DistanceToCurrentWaypoint = inf;
            end

            totalCount = missionCommand.TotalWaypointCount;
            if totalCount <= 0
                missionCommand.Progress = 0;
                return;
            end

            if missionCommand.Status == MissionStatus.Completed
                missionCommand.Progress = 1;
                return;
            end

            reached = missionCommand.ReachedWaypointCount;
            if isfield(missionCommand, 'IsCyclic') && missionCommand.IsCyclic
                missionCommand.Progress = mod(reached, max(totalCount, 1)) / max(totalCount, 1);
            else
                missionCommand.Progress = min(1, reached / totalCount);
            end
        end

        function missionCommand = tryAdvanceWaypoint(missionCommand, position, threshold)
            if missionCommand.Status ~= MissionStatus.Executing
                return;
            end

            if missionCommand.IsMissionComplete
                return;
            end

            if norm(position(1:2) - missionCommand.CurrentWaypoint(1:2)) > threshold
                return;
            end

            missionCommand.ReachedWaypointCount = missionCommand.ReachedWaypointCount + 1;
            nextIndex = missionCommand.CurrentWaypointIndex + 1;

            if nextIndex > size(missionCommand.MissionRoute, 1)
                if MissionStateMachine.isCyclicMission(missionCommand)
                    nextIndex = 1;
                else
                    missionCommand.IsMissionComplete = true;
                    missionCommand.NextWaypoint = [nan, nan, nan];
                    missionCommand.Progress = 1;
                    return;
                end
            end

            missionCommand.CurrentWaypointIndex = nextIndex;
            missionCommand.CurrentWaypoint = missionCommand.MissionRoute(nextIndex, :);
            missionCommand.NextWaypoint = MissionCommand.nextWaypointAt( ...
                missionCommand.MissionRoute, nextIndex);
            missionCommand.Progress = min(1, missionCommand.ReachedWaypointCount / max(missionCommand.TotalWaypointCount, 1));
        end

        function tf = isInspectionMission(missionCommand)
            tf = isstruct(missionCommand) && ...
                isfield(missionCommand, 'MissionType') && ...
                missionCommand.MissionType == MissionType.InspectArea && ...
                isfield(missionCommand, 'InspectionPhase');
        end

        function tf = isBirdMission(missionCommand)
            tf = isstruct(missionCommand) && ...
                isfield(missionCommand, 'MissionType') && ...
                missionCommand.MissionType == MissionType.MoveBetweenZones && ...
                isfield(missionCommand, 'BirdPhase');
        end

        function missionCommand = updateBirdMission(missionCommand, target, environment, dt)
            if missionCommand.IsMissionComplete
                return;
            end

            switch missionCommand.BirdPhase
                case BirdPhase.MoveToWaypoint
                    threshold = MissionStateMachine.waypointThreshold(target.Type, missionCommand);
                    if norm(target.Position(1:2) - missionCommand.CurrentWaypoint(1:2)) <= threshold
                        missionCommand = MissionStateMachine.beginBirdLowAltitudeHide( ...
                            missionCommand, target, environment);
                    end

                case BirdPhase.LowAltitudeHide
                    missionCommand.BirdHideElapsed = missionCommand.BirdHideElapsed + dt;
                    if missionCommand.BirdHideElapsed >= missionCommand.BirdHideDuration
                        missionCommand = MissionStateMachine.advanceBirdWaypoint(missionCommand);
                    end
            end
        end

        function missionCommand = beginBirdLowAltitudeHide(missionCommand, target, environment)
            waypointIndex = missionCommand.CurrentWaypointIndex;
            shouldHide = false;
            if isfield(missionCommand, 'BirdLowAltitudeHideFlags') && ...
                    waypointIndex <= numel(missionCommand.BirdLowAltitudeHideFlags)
                shouldHide = missionCommand.BirdLowAltitudeHideFlags(waypointIndex);
            end

            if ~shouldHide && target.Position(3) > 12
                missionCommand = MissionStateMachine.advanceBirdWaypoint(missionCommand);
                return;
            end

            stream = RandStream('mt19937ar', 'Seed', ...
                round(target.ID * 97 + waypointIndex + environment.RandomSeed));
            missionCommand.BirdPhase = BirdPhase.LowAltitudeHide;
            missionCommand.BirdHideDuration = 4 + 6 * stream.rand();
            missionCommand.BirdHideElapsed = 0;
        end

        function missionCommand = advanceBirdWaypoint(missionCommand)
            missionCommand.ReachedWaypointCount = missionCommand.ReachedWaypointCount + 1;
            nextIndex = missionCommand.CurrentWaypointIndex + 1;

            if nextIndex > size(missionCommand.MissionRoute, 1)
                missionCommand.IsMissionComplete = true;
                missionCommand.NextWaypoint = [nan, nan, nan];
                missionCommand.Progress = 1;
                return;
            end

            missionCommand.CurrentWaypointIndex = nextIndex;
            missionCommand.CurrentWaypoint = missionCommand.MissionRoute(nextIndex, :);
            missionCommand.NextWaypoint = MissionCommand.nextWaypointAt( ...
                missionCommand.MissionRoute, nextIndex);
            missionCommand.BirdPhase = BirdPhase.MoveToWaypoint;
            missionCommand.BirdHideElapsed = 0;
            missionCommand.BirdHideDuration = 0;

            legIdx = min(nextIndex - 1, numel(missionCommand.BirdSegmentSpeeds));
            if legIdx >= 1
                missionCommand.DesiredMissionSpeed = missionCommand.BirdSegmentSpeeds(legIdx);
            end
            missionCommand.DesiredMissionAltitude = missionCommand.CurrentWaypoint(3);
            missionCommand.Progress = min(1, missionCommand.ReachedWaypointCount / ...
                max(missionCommand.TotalWaypointCount, 1));
        end

        function missionCommand = updateInspectionMission(missionCommand, target, dt)
            if missionCommand.IsMissionComplete
                return;
            end

            switch missionCommand.InspectionPhase
                case InspectionPhase.MoveToPoint
                    threshold = MissionStateMachine.waypointThreshold(target.Type, missionCommand);
                    altitudeTolerance = 5;
                    xyReached = norm(target.Position(1:2) - missionCommand.CurrentWaypoint(1:2)) <= threshold;
                    zReached = abs(target.Position(3) - missionCommand.CurrentWaypoint(3)) <= altitudeTolerance;
                    if xyReached && zReached
                        missionCommand = MissionStateMachine.beginHoverObserve(missionCommand, target);
                    end

                case InspectionPhase.HoverObserve
                    missionCommand.InspectionHoverElapsed = ...
                        missionCommand.InspectionHoverElapsed + dt;
                    if missionCommand.InspectionHoverElapsed >= missionCommand.InspectionHoverTime
                        missionCommand = MissionStateMachine.finishHoverObserve(missionCommand, target);
                    end

                case InspectionPhase.AltitudeAdjust
                    if ~isfield(missionCommand, 'InspectionPhaseElapsed')
                        missionCommand.InspectionPhaseElapsed = 0;
                    end
                    missionCommand.InspectionPhaseElapsed = ...
                        missionCommand.InspectionPhaseElapsed + dt;
                    if abs(target.Position(3) - missionCommand.InspectionPhaseAltitude) <= 3 || ...
                            missionCommand.InspectionPhaseElapsed >= 25
                        missionCommand = MissionStateMachine.advanceInspectionWaypoint(missionCommand);
                    end

                case InspectionPhase.MoveToNextPoint
                    missionCommand = MissionStateMachine.advanceInspectionWaypoint(missionCommand);
            end
        end

        function missionCommand = beginHoverObserve(missionCommand, target)
            stream = RandStream('mt19937ar', 'Seed', ...
                round(target.ID * 131 + missionCommand.CurrentWaypointIndex));

            missionCommand.InspectionPhase = InspectionPhase.HoverObserve;
            missionCommand.InspectionHoverTime = 3 + 9 * stream.rand();
            missionCommand.InspectionHoverElapsed = 0;
            missionCommand.InspectionHoverSpeed = stream.rand();
            missionCommand.InspectionHoverAltitude = target.Position(3) + (-2 + 4 * stream.rand());
            missionCommand.InspectionHoverPosition = target.Position;
        end

        function missionCommand = finishHoverObserve(missionCommand, target)
            legIdx = missionCommand.CurrentWaypointIndex;
            shouldAdjust = legIdx < missionCommand.TotalWaypointCount && ...
                MissionStateMachine.shouldAdjustAltitude(missionCommand, legIdx);

            if shouldAdjust
                delta = missionCommand.InspectionAdjustDelta(min(legIdx, numel(missionCommand.InspectionAdjustDelta)));
                missionCommand.InspectionPhaseAltitude = target.Position(3) + delta;
                missionCommand.InspectionPhase = InspectionPhase.AltitudeAdjust;
                missionCommand.InspectionPhaseElapsed = 0;
                return;
            end

            missionCommand.InspectionPhase = InspectionPhase.MoveToNextPoint;
            missionCommand = MissionStateMachine.advanceInspectionWaypoint(missionCommand);
        end

        function tf = shouldAdjustAltitude(missionCommand, legIdx)
            if ~isfield(missionCommand, 'InspectionAdjustFlags') || ...
                    isempty(missionCommand.InspectionAdjustFlags)
                tf = false;
                return;
            end

            flagIdx = min(legIdx, numel(missionCommand.InspectionAdjustFlags));
            tf = missionCommand.InspectionAdjustFlags(flagIdx);
        end

        function missionCommand = advanceInspectionWaypoint(missionCommand)
            missionCommand.ReachedWaypointCount = missionCommand.ReachedWaypointCount + 1;
            nextIndex = missionCommand.CurrentWaypointIndex + 1;

            if nextIndex > size(missionCommand.MissionRoute, 1)
                missionCommand.IsMissionComplete = true;
                missionCommand.NextWaypoint = [nan, nan, nan];
                missionCommand.Progress = 1;
                return;
            end

            missionCommand.CurrentWaypointIndex = nextIndex;
            missionCommand.CurrentWaypoint = missionCommand.MissionRoute(nextIndex, :);
            missionCommand.NextWaypoint = MissionCommand.nextWaypointAt( ...
                missionCommand.MissionRoute, nextIndex);
            missionCommand.InspectionPhase = InspectionPhase.MoveToPoint;
            missionCommand.InspectionHoverElapsed = 0;
            missionCommand.InspectionHoverTime = 0;
            missionCommand.Progress = min(1, missionCommand.ReachedWaypointCount / ...
                max(missionCommand.TotalWaypointCount, 1));
        end

        function tf = isCyclicMission(missionCommand)
            tf = isfield(missionCommand, 'IsCyclic') && missionCommand.IsCyclic && ...
                missionCommand.MissionType == MissionType.PatrolRoute;
        end

        function tf = isInvalidRoute(missionCommand)
            route = missionCommand.MissionRoute;
            if missionCommand.MissionType == MissionType.InspectArea
                tf = size(route, 1) < 3 || any(~isfinite(route(:)));
                return;
            end

            if missionCommand.MissionType == MissionType.MoveBetweenZones && ...
                    isfield(missionCommand, 'BirdPhase')
                tf = size(route, 1) < 3 || any(~isfinite(route(:)));
                return;
            end

            tf = size(route, 1) < 2 || any(~isfinite(route(:)));
        end

        function threshold = waypointThreshold(targetType, missionCommand)
            if nargin < 2
                missionCommand = struct();
            end

            switch char(targetType)
                case char(TargetType.False)
                    threshold = 18;
                case char(TargetType.Ground)
                    threshold = 25;
                case char(TargetType.AirplaneUAV)
                    if isfield(missionCommand, 'WaypointTolerance') && ...
                            isfinite(missionCommand.WaypointTolerance) && ...
                            missionCommand.WaypointTolerance > 0
                        threshold = missionCommand.WaypointTolerance;
                    else
                        threshold = 65;
                    end
                case char(TargetType.Quadcopter)
                    threshold = 15;
                otherwise
                    threshold = 20;
            end
        end
    end
end
