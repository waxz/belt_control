function [v,a,x] = ...
    evalSmoothstepProfile( ...
    v0,...
    vf,...
    aMax,...
    t)

dv = vf - v0;

if dv <= 0

    v = vf;
    a = 0;
    x = vf*max(t,0);

    return;

end

T = 1.5*dv/aMax;

if t <= 0

    v = v0;
    a = 0;
    x = 0;
    return;

end

if t >= T

    xRamp = (v0+vf)/2*T;

    v = vf;
    a = 0;

    x = xRamp + vf*(t-T);

    return;

end

s = t/T;

v = v0 + dv*(3*s^2 - 2*s^3);

a = dv/T*(6*s - 6*s^2);

x = ...
    v0*t + ...
    dv*t^3/T^2 - ...
    dv*t^4/(2*T^3);

end