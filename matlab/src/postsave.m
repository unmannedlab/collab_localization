fn = [datestr(now,'yyyy_mm_dd-HH_MM-'),in_name,'-',num2str(nSims),'.mat'];
ff = fullfile('output',fn);

save(ff, 'CKF_rmse', 'EKF_rmse', 'DCL_rmse');