classdef RadarOutputExporter
    % RadarOutputExporter  Экспорт результатов симуляции в формат радиолокационного вывода.

    methods (Static)
        function output = exportFrame(frame)
            arguments
                frame (1, 1) struct
            end

            RadarOutputExporter.validateInputFrame(frame);

            output = struct();
            output.Time = frame.Time;
            output.Targets = cell(numel(frame.Targets), 1);

            for targetIdx = 1:numel(frame.Targets)
                output.Targets{targetIdx} = RadarOutputExporter.exportTarget(frame.Targets{targetIdx});
            end
        end

        function outputs = exportSimulation(result)
            arguments
                result (1, 1) struct
            end

            if ~isfield(result, 'OutputFrames')
                error('RadarOutputExporter:InvalidResult', ...
                    'Result must contain OutputFrames field.');
            end

            frames = result.OutputFrames;
            outputs = repmat(RadarOutputExporter.emptyOutputFrame(), numel(frames), 1);

            for frameIdx = 1:numel(frames)
                outputs(frameIdx) = RadarOutputExporter.exportFrame(frames(frameIdx));
            end
        end

        function tableData = toTable(outputs)
            arguments
                outputs (:, 1) struct
            end

            rows = RadarOutputExporter.collectTableRows(outputs);
            tableData = struct2table(rows, 'AsArray', true);
        end

        function toCSV(outputs, filename)
            arguments
                outputs (:, 1) struct
                filename (1, :) char
            end

            tableData = RadarOutputExporter.toTable(outputs);
            writetable(tableData, filename);
        end

        function toMAT(outputs, filename)
            arguments
                outputs (:, 1) struct
                filename (1, :) char
            end

            radarOutputs = outputs; %#ok<NASGU>
            save(filename, 'radarOutputs');
        end
    end

    methods (Static, Access = private)
        function targetOutput = exportTarget(snapshot)
            kinematics = [
                snapshot.Position(1), snapshot.Velocity(1)
                snapshot.Position(2), snapshot.Velocity(2)
                snapshot.Position(3), snapshot.Velocity(3)
            ];

            targetOutput = struct( ...
                'ID', snapshot.ID, ...
                'Type', snapshot.Type, ...
                'State', snapshot.CurrentState, ...
                'IsHidden', snapshot.IsHidden, ...
                'RCS', snapshot.RCS, ...
                'Time', snapshot.Time, ...
                'BehaviorMode', snapshot.BehaviorMode, ...
                'DesiredHeading', snapshot.DesiredHeading, ...
                'DesiredSpeed', snapshot.DesiredSpeed, ...
                'DesiredAltitude', snapshot.DesiredAltitude, ...
                'BehaviorReason', snapshot.BehaviorReason, ...
                'HeadingNoise', RadarOutputExporter.optionalField(snapshot, 'HeadingNoise', 0), ...
                'SpeedNoise', RadarOutputExporter.optionalField(snapshot, 'SpeedNoise', 0), ...
                'AltitudeNoise', RadarOutputExporter.optionalField(snapshot, 'AltitudeNoise', 0), ...
                'PositionNoiseNorm', RadarOutputExporter.optionalField(snapshot, 'PositionNoiseNorm', 0), ...
                'MissionType', snapshot.MissionType, ...
                'MissionWaypointIndex', snapshot.MissionWaypointIndex, ...
                'MissionReason', snapshot.MissionReason, ...
                'MissionStatus', snapshot.MissionStatus, ...
                'MissionProgress', snapshot.MissionProgress, ...
                'MissionDistanceToWaypoint', snapshot.MissionDistanceToWaypoint, ...
                'MissionCompletionReason', snapshot.MissionCompletionReason, ...
                'MissionCancelReason', snapshot.MissionCancelReason, ...
                'Kinematics', kinematics);
        end

        function rows = collectTableRows(outputs)
            rowCount = 0;
            for frameIdx = 1:numel(outputs)
                rowCount = rowCount + numel(outputs(frameIdx).Targets);
            end

            rows = repmat(RadarOutputExporter.emptyTableRow(), rowCount, 1);
            rowIdx = 0;

            for frameIdx = 1:numel(outputs)
                frameOutput = outputs(frameIdx);

                for targetIdx = 1:numel(frameOutput.Targets)
                    targetOutput = frameOutput.Targets{targetIdx};
                    rowIdx = rowIdx + 1;

                    rows(rowIdx).Time = targetOutput.Time;
                    rows(rowIdx).ID = targetOutput.ID;
                    rows(rowIdx).Type = targetOutput.Type;
                    rows(rowIdx).State = string(targetOutput.State);
                    rows(rowIdx).IsHidden = targetOutput.IsHidden;
                    rows(rowIdx).X = targetOutput.Kinematics(1, 1);
                    rows(rowIdx).Y = targetOutput.Kinematics(2, 1);
                    rows(rowIdx).Z = targetOutput.Kinematics(3, 1);
                    rows(rowIdx).Vx = targetOutput.Kinematics(1, 2);
                    rows(rowIdx).Vy = targetOutput.Kinematics(2, 2);
                    rows(rowIdx).Vz = targetOutput.Kinematics(3, 2);
                    rows(rowIdx).RCS = targetOutput.RCS;
                    rows(rowIdx).BehaviorMode = string(targetOutput.BehaviorMode);
                    rows(rowIdx).DesiredHeading = targetOutput.DesiredHeading;
                    rows(rowIdx).DesiredSpeed = targetOutput.DesiredSpeed;
                    rows(rowIdx).DesiredAltitude = targetOutput.DesiredAltitude;
                    rows(rowIdx).BehaviorReason = string(targetOutput.BehaviorReason);
                    rows(rowIdx).MissionType = string(targetOutput.MissionType);
                    rows(rowIdx).MissionWaypointIndex = targetOutput.MissionWaypointIndex;
                    rows(rowIdx).MissionReason = string(targetOutput.MissionReason);
                    rows(rowIdx).MissionStatus = string(targetOutput.MissionStatus);
                    rows(rowIdx).MissionProgress = targetOutput.MissionProgress;
                    rows(rowIdx).MissionDistanceToWaypoint = targetOutput.MissionDistanceToWaypoint;
                    rows(rowIdx).MissionCompletionReason = string(targetOutput.MissionCompletionReason);
                    rows(rowIdx).MissionCancelReason = string(targetOutput.MissionCancelReason);
                    rows(rowIdx).HeadingNoise = targetOutput.HeadingNoise;
                    rows(rowIdx).SpeedNoise = targetOutput.SpeedNoise;
                    rows(rowIdx).AltitudeNoise = targetOutput.AltitudeNoise;
                    rows(rowIdx).PositionNoiseNorm = targetOutput.PositionNoiseNorm;
                end
            end
        end

        function output = emptyOutputFrame()
            output = struct('Time', 0, 'Targets', {{}});
        end

        function row = emptyTableRow()
            row = struct( ...
                'Time', 0, ...
                'ID', 0, ...
                'Type', "", ...
                'State', "", ...
                'IsHidden', false, ...
                'X', 0, ...
                'Y', 0, ...
                'Z', 0, ...
                'Vx', 0, ...
                'Vy', 0, ...
                'Vz', 0, ...
                'RCS', 0, ...
                'BehaviorMode', "", ...
                'DesiredHeading', 0, ...
                'DesiredSpeed', 0, ...
                'DesiredAltitude', 0, ...
                'BehaviorReason', "", ...
                'MissionType', "", ...
                'MissionWaypointIndex', 0, ...
                'MissionReason', "", ...
                'MissionStatus', "", ...
                'MissionProgress', 0, ...
                'MissionDistanceToWaypoint', 0, ...
                'MissionCompletionReason', "", ...
                'MissionCancelReason', "", ...
                'HeadingNoise', 0, ...
                'SpeedNoise', 0, ...
                'AltitudeNoise', 0, ...
                'PositionNoiseNorm', 0);
        end

        function value = optionalField(snapshot, fieldName, defaultValue)
            if isfield(snapshot, fieldName)
                value = snapshot.(fieldName);
            else
                value = defaultValue;
            end
        end

        function validateInputFrame(frame)
            if ~isfield(frame, 'Time') || ~isfield(frame, 'Targets')
                error('RadarOutputExporter:InvalidFrame', ...
                    'Frame must contain Time and Targets fields.');
            end

            requiredSnapshotFields = {
                'ID'
                'Type'
                'Position'
                'Velocity'
                'RCS'
                'CurrentState'
                'IsHidden'
                'BehaviorMode'
                'DesiredHeading'
                'DesiredSpeed'
                'DesiredAltitude'
                'BehaviorReason'
                'MissionType'
                'MissionWaypointIndex'
                'MissionReason'
                'MissionStatus'
                'MissionProgress'
                'MissionDistanceToWaypoint'
                'MissionCompletionReason'
                'MissionCancelReason'
                'Time'
            };

            for targetIdx = 1:numel(frame.Targets)
                snapshot = frame.Targets{targetIdx};
                for fieldIdx = 1:numel(requiredSnapshotFields)
                    fieldName = requiredSnapshotFields{fieldIdx};
                    if ~isfield(snapshot, fieldName)
                        error('RadarOutputExporter:InvalidFrame', ...
                            'Target snapshot missing field: %s', fieldName);
                    end
                end
            end
        end
    end
end
