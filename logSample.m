function logger = logSample( ...
    logger,...
    t,...
    targetX,...
    targetV,...
    parcelX,...
    parcelV,...
    refX,...
    refV,...
    refA,...
    dt)

k = logger.count + 1;

logger.time(k) = t;

logger.xTarget(k) = targetX;
logger.vTarget(k) = targetV;

logger.xParcel(k) = parcelX;
logger.vParcel(k) = parcelV;

logger.xRef(k) = refX;
logger.vRef(k) = refV;

logger.aRef(k) = refA;

logger.posError(k) = targetX - parcelX;
logger.velError(k) = targetV - parcelV;

logger.dt(k) = dt;

logger.count = k;

end