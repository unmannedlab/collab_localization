function [out] = angErr(measurement, truth)
    err = measurement - truth;
    out = err;
    out(err < -pi) = -2*pi - err(err < -pi);
    out(err > pi) = -err(err > pi) + 2*pi;
end