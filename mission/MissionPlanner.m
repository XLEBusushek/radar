classdef MissionPlanner
    % MissionPlanner  Стратегический планировщик долгосрочных миссий.

    methods (Static)
        function [target, missionCommand] = plan(target, environment, dt)
            arguments
                target (1, 1) RadarTargetModel
                environment (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            target = target.tickMissionTime(dt);

            if ~target.isMissionActive()
                returnCommand = [];
                if MissionCommand.isActive(target.MissionCommand) && ...
                        MissionStateMachine.isTerminal(target.MissionCommand)
                    returnCommand = MissionPlanner.buildReturnToAreaMission(target, environment);
                end

                if ~isempty(returnCommand)
                    target = target.setMissionCommand(returnCommand);
                    [target, missionCommand] = MissionStateMachine.update( ...
                        target, target.MissionCommand, environment, dt);
                    return;
                end

                missionCommand = MissionPlanner.createMissionForType(target, environment);
                target = target.setMissionCommand(missionCommand);
                [target, missionCommand] = MissionStateMachine.update( ...
                    target, target.MissionCommand, environment, dt);
                return;
            end

            returnCommand = MissionPlanner.buildReturnToAreaMission(target, environment);
            if ~isempty(returnCommand)
                target = target.setMissionCommand(returnCommand);
                [target, missionCommand] = MissionStateMachine.update( ...
                    target, target.MissionCommand, environment, dt);
                return;
            end

            [target, missionCommand] = MissionStateMachine.update( ...
                target, target.MissionCommand, environment, dt);

            if MissionStateMachine.isTerminal(missionCommand) && ...
                    target.MissionTime >= missionCommand.MissionHoldTime
                missionCommand = MissionPlanner.createMissionForType(target, environment);
                target = target.setMissionCommand(missionCommand);
                [target, missionCommand] = MissionStateMachine.update( ...
                    target, target.MissionCommand, environment, dt);
            end
        end
    end

    methods (Static, Access = private)
        function command = createMissionForType(target, environment)
            switch target.Type
                case TargetType.False
                    command = BirdMissionPlanner.createMission(target, environment);
                case TargetType.Ground
                    command = GroundMissionPlanner.createMission(target, environment);
                case TargetType.AirplaneUAV
                    command = AirplaneMissionPlanner.createMission(target, environment);
                case TargetType.Quadcopter
                    command = QuadcopterMissionPlanner.createMission(target, environment);
                otherwise
                    command = MissionCommand.create( ...
                        'MissionType', MissionType.Idle, ...
                        'MissionRoute', target.Position, ...
                        'DesiredMissionSpeed', target.Speed, ...
                        'DesiredMissionAltitude', target.Position(3), ...
                        'MissionHoldTime', 10, ...
                        'MissionReason', 'Idle mission');
            end
        end

        function command = buildReturnToAreaMission(target, environment)
            command = [];
            marginRatio = 0.10;
            position = target.Position;

            xSpan = diff(environment.XLimits);
            ySpan = diff(environment.YLimits);
            xMargin = marginRatio * xSpan;
            yMargin = marginRatio * ySpan;

            nearBoundary = ...
                (position(1) - environment.XLimits(1)) < xMargin || ...
                (environment.XLimits(2) - position(1)) < xMargin || ...
                (position(2) - environment.YLimits(1)) < yMargin || ...
                (environment.YLimits(2) - position(2)) < yMargin;

            if ~nearBoundary
                return;
            end

            if MissionCommand.isActive(target.MissionCommand) && ...
                    target.MissionCommand.MissionType == MissionType.InspectArea && ...
                    (ismember(target.MissionCommand.Status, [MissionStatus.Executing, MissionStatus.Paused, MissionStatus.Planning]) || ...
                    (MissionStateMachine.isTerminal(target.MissionCommand) && ...
                    target.MissionTime < target.MissionCommand.MissionHoldTime))
                return;
            end

            if MissionCommand.isActive(target.MissionCommand) && ...
                    target.MissionCommand.MissionType == MissionType.ReturnToArea && ...
                    ~MissionStateMachine.isTerminal(target.MissionCommand)
                return;
            end

            route = MissionRouteBuilder.returnToAreaRoute(target, environment);
            command = MissionCommand.create( ...
                'MissionType', MissionType.ReturnToArea, ...
                'MissionRoute', route, ...
                'DesiredMissionSpeed', target.Speed, ...
                'DesiredMissionAltitude', target.Position(3), ...
                'MissionHoldTime', max(10, MissionTypeCatalog.minimumHoldTime(target.Type)), ...
                'MissionPriority', 10, ...
                'MissionReason', 'Return inside simulation area', ...
                'MissionStartTime', target.MissionTime, ...
                'Status', MissionStatus.Created);
        end
    end
end
