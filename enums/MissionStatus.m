classdef MissionStatus
    % MissionStatus  Состояния жизненного цикла миссии.

    enumeration
        Created
        Planning
        Executing
        Paused
        Completed
        Cancelled
    end
end
