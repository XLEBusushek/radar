classdef PhasedTargetAdapter < handle
    % PhasedTargetAdapter  Адаптер целей для Phased Array Toolbox.
    %
    % Преобразует состояние RadarTargetModel в phased.Platform и phased.RadarTarget.
    % Логику движения не выполняет — только синхронизацию данных.

    properties (SetAccess = private)
        Targets
        OperatingFrequency
        Platforms
        RadarTargets
        IsHidden
        NumTargets
    end

    methods
        function obj = PhasedTargetAdapter(targets, operatingFrequency)
            arguments
                targets (:, 1) cell
                operatingFrequency (1, 1) double {mustBePositive}
            end

            if isempty(targets)
                error('PhasedTargetAdapter:EmptyTargets', ...
                    'At least one target is required.');
            end

            obj.Targets = targets;
            obj.OperatingFrequency = operatingFrequency;
            obj.NumTargets = numel(targets);
            obj.Platforms = cell(obj.NumTargets, 1);
            obj.RadarTargets = cell(obj.NumTargets, 1);
            obj.IsHidden = false(obj.NumTargets, 1);
        end

        function obj = initialize(obj)
            PhasedTargetAdapter.assertToolboxAvailable();

            for targetIdx = 1:obj.NumTargets
                target = obj.Targets{targetIdx};
                obj.Platforms{targetIdx} = PhasedTargetAdapter.createPlatform(target);
                obj.RadarTargets{targetIdx} = PhasedTargetAdapter.createRadarTarget( ...
                    target, obj.OperatingFrequency);
                obj.IsHidden(targetIdx) = target.IsHidden;
            end
        end

        function obj = updateFromFrame(obj, frame)
            arguments
                obj (1, 1) PhasedTargetAdapter
                frame (1, 1) struct
            end

            PhasedTargetAdapter.validateFrame(frame, obj.NumTargets);

            for targetIdx = 1:obj.NumTargets
                snapshot = frame.Targets{targetIdx};
                obj.IsHidden(targetIdx) = snapshot.IsHidden;

                platform = obj.Platforms{targetIdx};
                PhasedTargetAdapter.updatePlatform(platform, snapshot.Position, snapshot.Velocity);

                radarTarget = obj.RadarTargets{targetIdx};
                PhasedTargetAdapter.updateRadarTarget(radarTarget, snapshot.RCS);
            end
        end

        function [positions, velocities, rcsValues] = getTargetStates(obj)
            positions = zeros(3, obj.NumTargets);
            velocities = zeros(3, obj.NumTargets);
            rcsValues = zeros(1, obj.NumTargets);

            for targetIdx = 1:obj.NumTargets
                platform = obj.Platforms{targetIdx};
                positions(:, targetIdx) = PhasedTargetAdapter.readColumnVector( ...
                    platform.InitialPosition, 3);
                velocities(:, targetIdx) = PhasedTargetAdapter.readColumnVector( ...
                    platform.Velocity, 3);
                rcsValues(targetIdx) = obj.RadarTargets{targetIdx}.MeanRCS;
            end
        end
    end

    methods (Static)
        function available = isToolboxAvailable()
            available = ~isempty(ver('phased')) && ...
                license('test', 'Phased_Array_System_Toolbox');
        end
    end

    methods (Static, Access = private)
        function assertToolboxAvailable()
            if ~PhasedTargetAdapter.isToolboxAvailable()
                error('PhasedTargetAdapter:ToolboxMissing', ...
                    'Phased Array System Toolbox is required for PhasedTargetAdapter.');
            end
        end

        function platform = createPlatform(target)
            platform = phased.Platform( ...
                'InitialPosition', target.Position(:), ...
                'Velocity', target.Velocity(:));
        end

        function radarTarget = createRadarTarget(target, operatingFrequency)
            radarTarget = phased.RadarTarget( ...
                'MeanRCS', target.RCS, ...
                'OperatingFrequency', operatingFrequency);
        end

        function updatePlatform(platform, position, velocity)
            if platform.isLocked
                release(platform);
            end

            platform.InitialPosition = position(:);
            platform.Velocity = velocity(:);
        end

        function updateRadarTarget(radarTarget, rcsValue)
            if radarTarget.isLocked
                release(radarTarget);
            end

            radarTarget.MeanRCS = rcsValue;
        end

        function validateFrame(frame, expectedTargetCount)
            if ~isfield(frame, 'Targets')
                error('PhasedTargetAdapter:InvalidFrame', ...
                    'Frame must contain Targets field.');
            end

            if numel(frame.Targets) ~= expectedTargetCount
                error('PhasedTargetAdapter:InvalidFrame', ...
                    'Frame target count does not match adapter target count.');
            end
        end

        function vector = readColumnVector(value, expectedLength)
            vector = reshape(value, [], 1);
            if numel(vector) ~= expectedLength
                error('PhasedTargetAdapter:InvalidVectorSize', ...
                    'Expected vector of length %d.', expectedLength);
            end
        end
    end
end
