classdef BirdMotionModel < MotionModelBase
    % BirdMotionModel  Хаотичное трехмерное движение птицы.

    methods (Static)
        function target = update(target, decision, behaviorCommand, profile, environment, dt)
            headingAtStart = target.Heading;
            pitchAtStart = target.Pitch;
            speedAtStart = target.Speed;

            target = BirdMotionModel.ensureContext(target);
            target = MotionStateExecutor.applyState(target, decision.NextState, profile, dt);
            target = MotionBehaviorGuidance.apply(target, behaviorCommand, profile, environment, dt);

            if ~BehaviorCommand.isActive(behaviorCommand)
                target = BirdMotionModel.applyChaoticMotion(target, profile, dt);
            else
                target = BirdMotionModel.applyPitchWobble(target, profile, dt);
            end

            target = MotionKinematics.applyBoundarySteering(target, profile, environment, dt);
            target = MotionKinematics.clampSpeed(target, profile, decision.NextState);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);
            target = MotionKinematics.enforceRateLimits(target, headingAtStart, speedAtStart, profile, dt);

            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;
            target.Pitch = MotionKinematics.rotateToward(pitchAtStart, target.Pitch, maxPitchStep);

            target = MotionKinematics.updateVelocity(target);
            target = MotionKinematics.integratePosition(target, environment, dt);
            target = MotionKinematics.enforceAltitudeProfile(target, profile, environment);

            if size(target.HistoryPosition, 1) >= 1
                stepDistance = norm(target.Position - target.HistoryPosition(end, :));
            else
                stepDistance = target.Speed * dt;
            end
            target.MotionContext.StraightDistance = target.MotionContext.StraightDistance + stepDistance;
        end
    end

    methods (Static, Access = private)
        function target = ensureContext(target)
            if ~isfield(target.MotionContext, 'StraightDistance')
                target.MotionContext.StraightDistance = 0;
                target.MotionContext.NextTurnDistance = 20 + 30 * rand();
                target.MotionContext.PitchPhase = 2 * pi * rand();
            end
        end

        function target = applyPitchWobble(target, profile, dt)
            context = target.MotionContext;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;
            context.PitchPhase = context.PitchPhase + dt;
            pitchWobble = 0.25 * sin(context.PitchPhase) + 0.1 * sin(2.1 * context.PitchPhase);
            target.Pitch = target.Pitch + pitchWobble * maxPitchStep;
            target.Pitch = max(min(target.Pitch, deg2rad(12)), deg2rad(-12));
            target.MotionContext = context;
        end

        function target = applyChaoticMotion(target, profile, dt)
            context = target.MotionContext;
            maxTurnStep = deg2rad(profile.MaxTurnRate) * dt;
            maxPitchStep = deg2rad(profile.MaxPitchRate) * dt;

            if context.StraightDistance >= context.NextTurnDistance
                turnSign = 1 - 2 * (rand() > 0.5);
                turnDelta = turnSign * maxTurnStep * (1 + rand());
                target.Heading = MotionKinematics.wrapAngle(target.Heading + turnDelta);
                context.StraightDistance = 0;
                context.NextTurnDistance = 20 + 30 * rand();
            end

            target = BirdMotionModel.applyPitchWobble(target, profile, dt);

            altitudeLimits = [profile.AltitudeMin, profile.AltitudeMax];
            preferredAltitude = mean(altitudeLimits);
            altitudeError = preferredAltitude - target.Position(3);
            target.Pitch = target.Pitch + sign(altitudeError) * 0.2 * maxPitchStep;

            target.MotionContext = context;
        end
    end
end
