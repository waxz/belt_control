function logger = finalizeLogger(logger)

N = logger.count;

fields = fieldnames(logger);

for i = 1:length(fields)

    f = fields{i};

    if strcmp(f,'count')
        continue;
    end

    logger.(f) = logger.(f)(1:N);

end

end