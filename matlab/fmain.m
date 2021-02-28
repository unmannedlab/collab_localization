function [out] = fmain(nSims, sensors)

    addpath('algorithms');
    addpath('input');
    addpath('output');
    addpath('src');

    runtime = 5;
    SF = -0;
    plt = false;
    sve = false;

    lone; 
    preprocessor;

    for t=1:length(ticks)
        Truth;
        EKF;
%         DCL;
%         DC2;
        CKF;

    %     EKF_LMK;
    %     DCL_LMK;
    end

    postprocessor;

    out.EKF = EKF_rmse;
%     out.DCL = DCL_rmse;
%     out.DC2 = DC2_rmse;
    out.CKF = CKF_rmse;

end

