function [out] = fmain(nSims)

    addpath('algorithms');
    addpath('input');
    addpath('output');
    addpath('src');

    runtime = 20;
    SF = -0;
    plt = false;
    sve = false;

    par; 
    preprocessor;

    for t=1:length(ticks)
        Truth;
        EKF;
        DCL;
        DC2;

    %     EKF_LMK;
    %     DCL_LMK;
    end

    postprocessor;

    out.EKF = EKF_m;
    out.DCL = DCL_m;
    out.DC2 = DC2_m;

end

