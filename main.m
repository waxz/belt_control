function directional_pid_simulation()

close all;
clc;

%% Configuration

cfg.targetX0 = -5.0;
cfg.targetV  = 2.5;

cfg.maxAccel = 3.0;

cfg.nominalDt = 0.01;
cfg.tFinal    = 10;

cfg.parcelV0  = 1.0; % Initial parcel velocity

%% Create Controller

ctrl = ConveyorSyncController( ...
    cfg.targetV,...
    cfg.maxAccel);

%% Runtime

parcelX = 0.0;
parcelV = cfg.parcelV0;

tClock = tic;
tPrev  = 0;

maxSamples = ceil(cfg.tFinal/cfg.nominalDt)*3 + 100;

logger = createLogger(maxSamples);
parcelVOut_last = parcelV;

while true

    tNow = toc(tClock);

    if tNow > cfg.tFinal
        break;
    end

    dt = tNow - tPrev;

    if dt <= 0
        continue;
    end

    targetX = cfg.targetX0 + cfg.targetV*tNow;
    % Update dynamic state for the next timestep (using parcelVOut with small noise)
    noise = 0.005 * randn(); % small noise added to velocity
    parcelV = parcelVOut_last + noise;
    parcelX = parcelX + parcelV * dt;

    [parcelXOut,...
        parcelVOut,...
        plannerX,...
        plannerV,...
        plannerA] = ...
        ctrl.update( ...
        tNow,...
        parcelX,...
        parcelV,...
        targetX);

    logger = logSample( ...
        logger,...
        tNow,...
        targetX,...
        cfg.targetV,...
        parcelX,...
        parcelV,...
        plannerX,...
        plannerV,...
        plannerA,...
        dt);

    parcelVOut_last = parcelVOut;




    tPrev = tNow;

    pause(cfg.nominalDt);

end

logger = finalizeLogger(logger);

plotResults(logger);

end