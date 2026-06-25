function gap = requiredGap( ...
    v0,...
    vf,...
    aMax)

dv = vf - v0;

if dv <= 0

    gap = 0;
    return;

end

gap = 0.75*dv^2/aMax;

end