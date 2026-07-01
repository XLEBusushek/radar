classdef NaturalMotionLayer
    % NaturalMotionLayer  Естественные микровозмущения поведенческой команды.

    methods (Static)
        function [target, behaviorCommand] = apply(target, behaviorCommand, environment, dt)
            arguments
                target (1, 1) RadarTargetModel
                behaviorCommand (1, 1) struct
                environment (1, 1) struct
                dt (1, 1) double {mustBePositive}
            end

            if ~BehaviorCommand.isActive(behaviorCommand)
                target.NaturalMotionState.LastUpdateTime = target.NaturalMotionState.LastUpdateTime + dt;
                return;
            end

            motionProfile = NaturalMotionProfileRegistry.getProfile(target.Type);
            targetProfile = TargetProfileRegistry.getProfile(target.Type);
            state = target.NaturalMotionState;
            noiseStream = NaturalMotionLayer.noiseStream(state, target, environment);

            if target.Type == TargetType.Ground
                state = NaturalMotionLayer.updateGroundNoiseState(state, motionProfile, dt, noiseStream);
                behaviorCommand = NaturalMotionLayer.applyGroundNoiseToCommand( ...
                    target, behaviorCommand, state, motionProfile, targetProfile, environment);
            elseif target.Type == TargetType.AirplaneUAV
                state = NaturalMotionLayer.updateAirplaneNoiseState(state, motionProfile, dt, noiseStream);
                behaviorCommand = NaturalMotionLayer.applyAirplaneNoiseToCommand( ...
                    target, behaviorCommand, state, motionProfile, targetProfile, environment);
            elseif target.Type == TargetType.Quadcopter
                state = NaturalMotionLayer.updateQuadcopterNoiseState(state, motionProfile, dt, noiseStream);
                behaviorCommand = NaturalMotionLayer.applyQuadcopterNoiseToCommand( ...
                    target, behaviorCommand, state, motionProfile, targetProfile, environment);
            elseif NaturalMotionLayer.isBirdMission(target)
                motionProfile = NaturalMotionProfileRegistry.birdMissionProfile();
                state = NaturalMotionLayer.updateBirdNoiseState(state, motionProfile, dt, noiseStream);
                behaviorCommand = NaturalMotionLayer.applyBirdNoiseToCommand( ...
                    target, behaviorCommand, state, motionProfile, targetProfile, environment);
            else
                state = NaturalMotionLayer.updateGenericNoiseState(state, motionProfile, dt, noiseStream);
                state = NaturalMotionLayer.clampStateNoise(state, motionProfile);
                behaviorCommand = NaturalMotionLayer.applyNoiseToCommand( ...
                    behaviorCommand, state, motionProfile, targetProfile, environment, target.Type);
            end

            state.LastUpdateTime = state.LastUpdateTime + dt;
            state.NoiseStream = noiseStream;
            target.NaturalMotionState = state;
        end
    end

    methods (Static, Access = private)
        function state = updateGenericNoiseState(state, motionProfile, dt, noiseStream)
            state.HeadingNoise = SmoothNoiseProcess.update( ...
                state.HeadingNoise, motionProfile.HeadingSigma, motionProfile.HeadingTau, dt, noiseStream);
            state.SpeedNoise = SmoothNoiseProcess.update( ...
                state.SpeedNoise, motionProfile.SpeedSigma, motionProfile.SpeedTau, dt, noiseStream);
            state.AltitudeNoise = SmoothNoiseProcess.update( ...
                state.AltitudeNoise, motionProfile.AltitudeSigma, motionProfile.AltitudeTau, dt, noiseStream);
            state.PositionNoise = SmoothNoiseProcess.update( ...
                state.PositionNoise, motionProfile.PositionSigma, motionProfile.PositionTau, dt, noiseStream);
            state.LaneOffsetNoise = SmoothNoiseProcess.update( ...
                state.LaneOffsetNoise, motionProfile.PositionSigma * 0.35, motionProfile.PositionTau, dt, noiseStream);
            state.HoverDrift = SmoothNoiseProcess.update( ...
                state.HoverDrift, motionProfile.PositionSigma * 0.25, motionProfile.PositionTau, dt, noiseStream);
            state.WindDrift = SmoothNoiseProcess.update( ...
                state.WindDrift, motionProfile.PositionSigma * 0.20, motionProfile.PositionTau * 1.2, dt, noiseStream);
        end

        function state = updateQuadcopterNoiseState(state, motionProfile, dt, noiseStream)
            state.HeadingNoise = SmoothNoiseProcess.update( ...
                state.HeadingNoise, motionProfile.HeadingSigma, motionProfile.HeadingTau, dt, noiseStream);
            state.SpeedNoise = SmoothNoiseProcess.update( ...
                state.SpeedNoise, motionProfile.SpeedSigma, motionProfile.SpeedTau, dt, noiseStream);
            state.AltitudeNoise = SmoothNoiseProcess.update( ...
                state.AltitudeNoise, motionProfile.AltitudeSigma, motionProfile.AltitudeTau, dt, noiseStream);
            state.HoverDrift = SmoothNoiseProcess.update( ...
                state.HoverDrift, motionProfile.HoverDriftSigma, motionProfile.HoverDriftTau, dt, noiseStream);

            state.HeadingNoise = NaturalMotionLayer.clampNoise( ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            state.SpeedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            state.AltitudeNoise = NaturalMotionLayer.clampNoise( ...
                state.AltitudeNoise, motionProfile.MaxHoverAltitudeNoise);
            if numel(state.HoverDrift) < 2
                state.HoverDrift = [state.HoverDrift, 0];
            end
            state.HoverDrift(1:2) = NaturalMotionLayer.clampNoiseVector( ...
                state.HoverDrift(1:2), motionProfile.MaxHoverDrift);
        end

        function command = applyQuadcopterNoiseToCommand(target, command, state, motionProfile, targetProfile, environment)
            if ~isfield(command, 'BehaviorMode')
                return;
            end

            switch command.BehaviorMode
                case BehaviorMode.HoverObserve
                    anchor = NaturalMotionLayer.resolveHoverAnchor(target, command);
                    driftXY = state.HoverDrift(1:2);
                    command.DesiredPosition(1:2) = anchor(1:2) + driftXY;
                    command.DesiredPosition(3) = anchor(3) + state.AltitudeNoise;
                    command.DesiredPosition(1) = min(max(command.DesiredPosition(1), ...
                        environment.XLimits(1)), environment.XLimits(2));
                    command.DesiredPosition(2) = min(max(command.DesiredPosition(2), ...
                        environment.YLimits(1)), environment.YLimits(2));

                    if isfinite(command.DesiredHeading)
                        command.DesiredHeading = MotionKinematics.wrapAngle( ...
                            command.DesiredHeading + state.HeadingNoise);
                    end

                    baseAltitude = anchor(3);
                    if isfinite(command.DesiredAltitude)
                        baseAltitude = command.DesiredAltitude;
                    end
                    command.DesiredAltitude = baseAltitude + state.AltitudeNoise;
                    altitudeLimits = TargetFactory.resolveAltitudeLimits(targetProfile, environment);
                    command.DesiredAltitude = min(max(command.DesiredAltitude, ...
                        baseAltitude - motionProfile.MaxHoverAltitudeNoise), ...
                        baseAltitude + motionProfile.MaxHoverAltitudeNoise);
                    command.DesiredAltitude = min(max(command.DesiredAltitude, ...
                        altitudeLimits(1)), altitudeLimits(2));

                    hoverSpeedNoise = NaturalMotionLayer.clampNoise( ...
                        state.SpeedNoise, motionProfile.MaxHoverSpeedNoise);
                    if isfinite(command.DesiredSpeed)
                        command.DesiredSpeed = command.DesiredSpeed + hoverSpeedNoise;
                        command.DesiredSpeed = min(motionProfile.HoverSpeedMax, ...
                            max(0, command.DesiredSpeed));
                    end

                case BehaviorMode.MoveToPoint
                    if isfinite(command.DesiredSpeed)
                        moveSpeedNoise = NaturalMotionLayer.clampNoise( ...
                            state.SpeedNoise, motionProfile.MaxMoveSpeedNoise);
                        command.DesiredSpeed = command.DesiredSpeed + moveSpeedNoise;
                        command.DesiredSpeed = min(motionProfile.MoveSpeedMax, ...
                            max(motionProfile.MoveSpeedMin, command.DesiredSpeed));
                        command.DesiredSpeed = min(max(command.DesiredSpeed, targetProfile.SpeedMin), ...
                            targetProfile.SpeedMax);
                    end

                otherwise
            end
        end

        function anchor = resolveHoverAnchor(target, command)
            anchor = target.Position;
            missionCommand = target.MissionCommand;

            if MissionCommand.isActive(missionCommand) && ...
                    isfield(missionCommand, 'InspectionHoverPosition') && ...
                    all(isfinite(missionCommand.InspectionHoverPosition))
                anchor = missionCommand.InspectionHoverPosition;
            elseif all(isfinite(command.DesiredPosition))
                anchor = command.DesiredPosition;
            end

            if MissionCommand.isActive(missionCommand) && ...
                    isfield(missionCommand, 'InspectionHoverAltitude') && ...
                    isfinite(missionCommand.InspectionHoverAltitude)
                anchor(3) = missionCommand.InspectionHoverAltitude;
            elseif isfinite(command.DesiredAltitude)
                anchor(3) = command.DesiredAltitude;
            end
        end

        function state = updateAirplaneNoiseState(state, motionProfile, dt, noiseStream)
            state.HeadingNoise = SmoothNoiseProcess.update( ...
                state.HeadingNoise, motionProfile.HeadingSigma, motionProfile.HeadingTau, dt, noiseStream);
            state.SpeedNoise = SmoothNoiseProcess.update( ...
                state.SpeedNoise, motionProfile.SpeedSigma, motionProfile.SpeedTau, dt, noiseStream);
            state.AltitudeNoise = SmoothNoiseProcess.update( ...
                state.AltitudeNoise, motionProfile.AltitudeSigma, motionProfile.AltitudeTau, dt, noiseStream);

            state.HeadingNoise = NaturalMotionLayer.clampNoise( ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            state.SpeedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            state.AltitudeNoise = NaturalMotionLayer.clampNoise( ...
                state.AltitudeNoise, motionProfile.MaxAltitudeNoise);
        end

        function state = updateBirdNoiseState(state, motionProfile, dt, noiseStream)
            state.HeadingNoise = SmoothNoiseProcess.update( ...
                state.HeadingNoise, motionProfile.HeadingSigma, motionProfile.HeadingTau, dt, noiseStream);
            state.SpeedNoise = SmoothNoiseProcess.update( ...
                state.SpeedNoise, motionProfile.SpeedSigma, motionProfile.SpeedTau, dt, noiseStream);
            state.AltitudeNoise = SmoothNoiseProcess.update( ...
                state.AltitudeNoise, motionProfile.AltitudeSigma, motionProfile.AltitudeTau, dt, noiseStream);
            state.WindDrift = SmoothNoiseProcess.update( ...
                state.WindDrift, motionProfile.WindDriftSigma, motionProfile.WindDriftTau, dt, noiseStream);

            state.HeadingNoise = NaturalMotionLayer.clampNoise( ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            state.SpeedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            state.AltitudeNoise = NaturalMotionLayer.clampNoise( ...
                state.AltitudeNoise, motionProfile.MaxAltitudeNoise);
            if numel(state.WindDrift) < 2
                state.WindDrift = [state.WindDrift, 0];
            end
            state.WindDrift(1:2) = NaturalMotionLayer.clampNoiseVector( ...
                state.WindDrift(1:2), motionProfile.MaxWindDrift);
        end

        function command = applyBirdNoiseToCommand(target, command, state, motionProfile, targetProfile, environment)
            if isfinite(command.DesiredHeading)
                command.DesiredHeading = MotionKinematics.wrapAngle( ...
                    command.DesiredHeading + state.HeadingNoise);
            end

            if isfinite(command.DesiredSpeed)
                command.DesiredSpeed = command.DesiredSpeed + state.SpeedNoise;
                command.DesiredSpeed = min(motionProfile.SpeedMax, ...
                    max(motionProfile.SpeedMin, command.DesiredSpeed));
                command.DesiredSpeed = min(max(command.DesiredSpeed, targetProfile.SpeedMin), ...
                    targetProfile.SpeedMax);
            end

            if isfinite(command.DesiredAltitude)
                baseAltitude = command.DesiredAltitude;
                command.DesiredAltitude = baseAltitude + state.AltitudeNoise;
                command.DesiredAltitude = min(max(command.DesiredAltitude, ...
                    baseAltitude - motionProfile.MaxAltitudeNoise), ...
                    baseAltitude + motionProfile.MaxAltitudeNoise);

                if isfield(command, 'BirdPhase') && ...
                        command.BirdPhase == BirdPhase.LowAltitudeHide
                    command.DesiredAltitude = min(motionProfile.LowHideAltitudeMax, ...
                        max(0, command.DesiredAltitude));
                end

                altitudeLimits = TargetFactory.resolveAltitudeLimits(targetProfile, environment);
                command.DesiredAltitude = min(max(command.DesiredAltitude, altitudeLimits(1)), ...
                    altitudeLimits(2));
            end

            if all(isfinite(command.DesiredPosition))
                windXY = state.WindDrift(1:2);
                if isfinite(command.DesiredHeading)
                    normal = [-sin(command.DesiredHeading), cos(command.DesiredHeading)];
                    lateralDrift = dot(windXY, normal) * normal;
                    command.DesiredPosition(1:2) = command.DesiredPosition(1:2) + lateralDrift;
                else
                    command.DesiredPosition(1:2) = command.DesiredPosition(1:2) + windXY;
                end
                command.DesiredPosition(1) = min(max(command.DesiredPosition(1), ...
                    environment.XLimits(1)), environment.XLimits(2));
                command.DesiredPosition(2) = min(max(command.DesiredPosition(2), ...
                    environment.YLimits(1)), environment.YLimits(2));
                if isfinite(command.DesiredAltitude)
                    command.DesiredPosition(3) = command.DesiredAltitude;
                end
            end
        end

        function tf = isBirdMission(target)
            missionCommand = target.MissionCommand;
            tf = target.Type == TargetType.False && ...
                MissionCommand.isActive(missionCommand) && ...
                missionCommand.MissionType == MissionType.MoveBetweenZones && ...
                isfield(missionCommand, 'BirdPhase');
        end

        function command = applyAirplaneNoiseToCommand(target, command, state, motionProfile, targetProfile, environment)
            if isfinite(command.DesiredHeading)
                command.DesiredHeading = MotionKinematics.wrapAngle( ...
                    command.DesiredHeading + state.HeadingNoise);
            end

            if isfinite(command.DesiredSpeed)
                command.DesiredSpeed = command.DesiredSpeed + state.SpeedNoise;
                command.DesiredSpeed = min(max(command.DesiredSpeed, targetProfile.SpeedMin), ...
                    targetProfile.SpeedMax);
            end

            if isfinite(command.DesiredAltitude)
                baseAltitude = command.DesiredAltitude;
                command.DesiredAltitude = baseAltitude + state.AltitudeNoise;
                command.DesiredAltitude = min(max(command.DesiredAltitude, ...
                    baseAltitude - motionProfile.MaxAltitudeNoise), ...
                    baseAltitude + motionProfile.MaxAltitudeNoise);

                preferredAltitude = NaturalMotionLayer.resolvePatrolPreferredAltitude(target, environment);
                if isfinite(preferredAltitude)
                    spread = motionProfile.MissionAltitudeSpread + motionProfile.MaxAltitudeNoise;
                    offsetMax = motionProfile.MissionAltitudeOffsetMax + motionProfile.MaxAltitudeNoise;
                    command.DesiredAltitude = min(max(command.DesiredAltitude, ...
                        preferredAltitude - spread), preferredAltitude + offsetMax);
                end

                altitudeLimits = TargetFactory.resolveAltitudeLimits(targetProfile, environment);
                command.DesiredAltitude = min(max(command.DesiredAltitude, altitudeLimits(1)), ...
                    altitudeLimits(2));
            end
        end

        function preferredAltitude = resolvePatrolPreferredAltitude(target, environment)
            preferredAltitude = nan;

            if MissionCommand.isActive(target.MissionCommand) && ...
                    isfield(target.MissionCommand, 'PatrolZone') && ...
                    isstruct(target.MissionCommand.PatrolZone) && ...
                    isfield(target.MissionCommand.PatrolZone, 'PreferredAltitude')
                preferredAltitude = target.MissionCommand.PatrolZone.PreferredAltitude;
                return;
            end

            if MissionCommand.isActive(target.MissionCommand) && ...
                    isfield(target.MissionCommand, 'PatrolZoneIndex') && ...
                    isfield(environment, 'PatrolZones') && ...
                    target.MissionCommand.PatrolZoneIndex >= 1 && ...
                    target.MissionCommand.PatrolZoneIndex <= numel(environment.PatrolZones)
                preferredAltitude = environment.PatrolZones( ...
                    target.MissionCommand.PatrolZoneIndex).PreferredAltitude;
            end
        end

        function state = updateGroundNoiseState(state, motionProfile, dt, noiseStream)
            state.LaneOffsetNoise = SmoothNoiseProcess.update( ...
                state.LaneOffsetNoise, motionProfile.LaneOffsetSigma, motionProfile.LaneOffsetTau, dt, noiseStream);
            state.SpeedNoise = SmoothNoiseProcess.update( ...
                state.SpeedNoise, motionProfile.SpeedSigma, motionProfile.SpeedTau, dt, noiseStream);
            state.RoadHeightNoise = SmoothNoiseProcess.update( ...
                state.RoadHeightNoise, motionProfile.RoadHeightSigma, motionProfile.RoadHeightTau, dt, noiseStream);

            state.LaneOffsetNoise = NaturalMotionLayer.clampNoise( ...
                state.LaneOffsetNoise, motionProfile.MaxLaneOffsetNoise);
            state.SpeedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            state.RoadHeightNoise = NaturalMotionLayer.clampNoise( ...
                state.RoadHeightNoise, motionProfile.MaxRoadHeightNoise);
        end

        function command = applyGroundNoiseToCommand(target, command, state, motionProfile, targetProfile, environment)
            baseLaneOffset = 0;
            if isfield(target.MotionContext, 'LaneOffset')
                baseLaneOffset = target.MotionContext.LaneOffset;
            end

            laneOffsetNoise = state.LaneOffsetNoise;
            finalLaneOffset = RoadGraph.clampFinalLaneOffset( ...
                baseLaneOffset, laneOffsetNoise, motionProfile.RoadWidth);
            deltaOffset = finalLaneOffset - baseLaneOffset;

            if all(isfinite(command.DesiredPosition)) && isfield(environment, 'RoadNetwork') && ...
                    abs(deltaOffset) > 1e-9
                roadInfo = Environment.findNearestRoad(environment, command.DesiredPosition);
                normal = [-sin(roadInfo.Heading), cos(roadInfo.Heading)];
                command.DesiredPosition(1:2) = command.DesiredPosition(1:2) + deltaOffset * normal;
            end

            if isfinite(command.DesiredSpeed)
                applySpeedNoise = true;
                if isfield(command, 'BehaviorMode')
                    if command.BehaviorMode == BehaviorMode.ApproachIntersection || ...
                            command.BehaviorMode == BehaviorMode.TurnAtIntersection
                        applySpeedNoise = false;
                    end
                end
                if applySpeedNoise
                    command.DesiredSpeed = command.DesiredSpeed + state.SpeedNoise;
                end
                command.DesiredSpeed = min(max(command.DesiredSpeed, targetProfile.SpeedMin), ...
                    targetProfile.SpeedMax);
            end

            referenceXY = target.Position(1:2);
            if all(isfinite(command.DesiredPosition))
                referenceXY = command.DesiredPosition(1:2);
            end
            terrainHeight = environment.Terrain.Height(referenceXY(1), referenceXY(2));
            rideHeight = motionProfile.GroundRideHeight;
            tolerance = motionProfile.GroundRideHeightTolerance;
            desiredAltitude = terrainHeight + rideHeight + state.RoadHeightNoise;
            desiredAltitude = min(max(desiredAltitude, terrainHeight + rideHeight - tolerance), ...
                terrainHeight + rideHeight + tolerance);

            command.DesiredAltitude = desiredAltitude;
            if all(isfinite(command.DesiredPosition))
                command.DesiredPosition(3) = desiredAltitude;
            end
        end

        function command = applyNoiseToCommand(command, state, motionProfile, targetProfile, environment, targetType)
            if nargin < 6
                targetType = "";
            end

            isMissionCommand = isfield(command, 'Reason') && contains(command.Reason, 'Mission:');

            applyAltitudeNoise = ~isMissionCommand;
            applySpeedNoise = ~isMissionCommand;
            applyPositionNoise = ~isMissionCommand;
            if ~isMissionCommand && isfield(command, 'BehaviorMode')
                if command.BehaviorMode == BehaviorMode.HoverObserve || ...
                        command.BehaviorMode == BehaviorMode.AltitudeAdjust
                    applyAltitudeNoise = false;
                    applySpeedNoise = false;
                end
            end
            headingNoise = NaturalMotionLayer.clampNoise( ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            speedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            altitudeNoise = NaturalMotionLayer.clampNoise( ...
                state.AltitudeNoise, motionProfile.MaxAltitudeNoise);
            positionNoiseXY = NaturalMotionLayer.clampNoiseVector( ...
                state.PositionNoise(1:2), motionProfile.MaxPositionNoise);

            if isfinite(command.DesiredHeading)
                command.DesiredHeading = MotionKinematics.wrapAngle( ...
                    command.DesiredHeading + headingNoise);
            end

            if applySpeedNoise && isfinite(command.DesiredSpeed)
                command.DesiredSpeed = command.DesiredSpeed + speedNoise;
                command.DesiredSpeed = min(max(command.DesiredSpeed, targetProfile.SpeedMin), ...
                    targetProfile.SpeedMax);
            end

            if applyAltitudeNoise && isfinite(command.DesiredAltitude)
                command.DesiredAltitude = command.DesiredAltitude + altitudeNoise;
                altitudeLimits = TargetFactory.resolveAltitudeLimits(targetProfile, environment);
                command.DesiredAltitude = min(max(command.DesiredAltitude, altitudeLimits(1)), ...
                    altitudeLimits(2));
            end

            if applyPositionNoise && all(isfinite(command.DesiredPosition))
                command.DesiredPosition(1) = command.DesiredPosition(1) + positionNoiseXY(1);
                command.DesiredPosition(2) = command.DesiredPosition(2) + positionNoiseXY(2);
                command.DesiredPosition(1) = min(max(command.DesiredPosition(1), ...
                    environment.XLimits(1)), environment.XLimits(2));
                command.DesiredPosition(2) = min(max(command.DesiredPosition(2), ...
                    environment.YLimits(1)), environment.YLimits(2));
                if isfinite(command.DesiredPosition(3))
                    altitudeLimits = TargetFactory.resolveAltitudeLimits(targetProfile, environment);
                    command.DesiredPosition(3) = min(max(command.DesiredPosition(3), ...
                        altitudeLimits(1)), altitudeLimits(2));
                end
            end
        end

        function noiseValue = clampNoise(noiseValue, maxNoise)
            noiseValue = min(max(noiseValue, -maxNoise), maxNoise);
        end

        function noiseVector = clampNoiseVector(noiseVector, maxNoise)
            magnitude = norm(noiseVector);
            if magnitude > maxNoise && magnitude > 0
                noiseVector = noiseVector * (maxNoise / magnitude);
            end
        end

        function state = clampStateNoise(state, motionProfile)
            state.HeadingNoise = NaturalMotionLayer.clampNoise( ...
                state.HeadingNoise, motionProfile.MaxHeadingNoise);
            state.SpeedNoise = NaturalMotionLayer.clampNoise( ...
                state.SpeedNoise, motionProfile.MaxSpeedNoise);
            state.AltitudeNoise = NaturalMotionLayer.clampNoise( ...
                state.AltitudeNoise, motionProfile.MaxAltitudeNoise);
            state.PositionNoise(1:2) = NaturalMotionLayer.clampNoiseVector( ...
                state.PositionNoise(1:2), motionProfile.MaxPositionNoise);
            if numel(state.PositionNoise) < 3
                state.PositionNoise(3) = 0;
            else
                state.PositionNoise(3) = NaturalMotionLayer.clampNoise( ...
                    state.PositionNoise(3), motionProfile.MaxPositionNoise);
            end
        end

        function stream = noiseStream(state, target, environment)
            if isfield(state, 'NoiseStream') && ~isempty(state.NoiseStream)
                stream = state.NoiseStream;
                return;
            end

            seed = 17;
            if isfield(environment, 'RandomSeed')
                seed = round(environment.RandomSeed + target.ID * 131);
            else
                seed = round(target.ID * 131);
            end
            stream = RandStream('mt19937ar', 'Seed', seed);
        end
    end
end
