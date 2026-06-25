classdef ConveyorSyncController < handle

properties

    targetVelocity
    maxAcceleration

    active

    triggerTime
    triggerPosition
    triggerVelocity

    previousTime
    previousGap

end

methods

function obj = ConveyorSyncController(vTarget,maxAccel)

    obj.targetVelocity = vTarget;
    obj.maxAcceleration = maxAccel;

    obj.active = false;

    obj.triggerTime = NaN;
    obj.triggerPosition = NaN;
    obj.triggerVelocity = 0.0;

    obj.previousTime = 0;
    obj.previousGap = NaN;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,v,xRef,vRef,aRef] = ...
            update(obj,...
                   tNow,...
                   parcelX,...
                   parcelV,...
                   targetX)

    gapNow = parcelX - targetX;

    if isnan(obj.previousGap)

        obj.previousGap = gapNow;
        obj.previousTime = tNow;

    end

    %% Trigger Logic

    if ~obj.active

        reqGap = requiredGap( ...
                    parcelV,...
                    obj.targetVelocity,...
                    obj.maxAcceleration);

        crossed = ...
            obj.previousGap > reqGap && ...
            gapNow <= reqGap;

        if crossed

            tCross = interpolateCrossing( ...
                        obj.previousTime,...
                        tNow,...
                        obj.previousGap,...
                        gapNow,...
                        reqGap);

            obj.active = true;

            obj.triggerTime = tCross;
            obj.triggerPosition = parcelX;
            obj.triggerVelocity = parcelV;

        end
    end

    %% Evaluate planner

    if obj.active

        elapsed = ...
            max(tNow - obj.triggerTime,0);

        [vRef,aRef,xRel] = ...
            evalSmoothstepProfile( ...
                obj.triggerVelocity,...
                obj.targetVelocity,...
                obj.maxAcceleration,...
                elapsed);

        xRef = obj.triggerPosition + xRel;

        % Proportional feedback on position tracking error
        Kp = 8.0;
        v = vRef + Kp * (xRef - parcelX);
        x = xRef;

    else

        x = parcelX;
        v = parcelV;

        xRef = NaN;
        vRef = NaN;
        aRef = 0;

    end

    obj.previousGap = gapNow;
    obj.previousTime = tNow;

end

end
end