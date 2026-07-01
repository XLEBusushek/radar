classdef MissionStatusCatalog
    % MissionStatusCatalog  Допустимые переходы состояний миссии.

    methods (Static)
        function tf = isTransitionAllowed(fromStatus, toStatus)
            if fromStatus == toStatus
                tf = true;
                return;
            end

            allowedPairs = {
                MissionStatus.Created, MissionStatus.Planning
                MissionStatus.Planning, MissionStatus.Executing
                MissionStatus.Planning, MissionStatus.Cancelled
                MissionStatus.Executing, MissionStatus.Paused
                MissionStatus.Paused, MissionStatus.Executing
                MissionStatus.Executing, MissionStatus.Completed
                MissionStatus.Executing, MissionStatus.Cancelled
                MissionStatus.Paused, MissionStatus.Cancelled
                MissionStatus.Completed, MissionStatus.Created
                MissionStatus.Cancelled, MissionStatus.Created
            };

            tf = false;
            for k = 1:size(allowedPairs, 1)
                if allowedPairs{k, 1} == fromStatus && allowedPairs{k, 2} == toStatus
                    tf = true;
                    return;
                end
            end
        end
    end
end
