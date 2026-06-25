function logger = createLogger(maxSamples)

logger.time      = nan(maxSamples,1);

logger.xTarget   = nan(maxSamples,1);
logger.vTarget   = nan(maxSamples,1);

logger.xParcel   = nan(maxSamples,1);
logger.vParcel   = nan(maxSamples,1);

logger.xRef      = nan(maxSamples,1);
logger.vRef      = nan(maxSamples,1);

logger.aRef      = nan(maxSamples,1);

logger.posError  = nan(maxSamples,1);
logger.velError  = nan(maxSamples,1);

logger.dt        = nan(maxSamples,1);

logger.count     = 0;

end