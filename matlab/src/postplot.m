x_max = 0; x_min = 1;
y_max = 0; y_min = 1;
t_max = 0; t_min = 1;

if exist('EKF_rmse','var')
    x_max = max(x_max, 3*std(EKF_rmse(1,:)) + mean(EKF_rmse(1,:)));
    y_max = max(y_max, 3*std(EKF_rmse(2,:)) + mean(EKF_rmse(2,:)));
    t_max = max(t_max, 3*std(EKF_rmse(3,:)) + mean(EKF_rmse(3,:)));
    x_min = min(x_min, min(EKF_rmse(1,:)));
    y_min = min(y_min, min(EKF_rmse(2,:)));
    t_min = min(t_min, min(EKF_rmse(3,:)));
end

if exist('DCL_rmse','var')
    x_max = max(x_max, 3*std(DCL_rmse(1,:)) + mean(DCL_rmse(1,:)));
    y_max = max(y_max, 3*std(DCL_rmse(2,:)) + mean(DCL_rmse(2,:)));
    t_max = max(t_max, 3*std(DCL_rmse(3,:)) + mean(DCL_rmse(3,:)));
    x_min = min(x_min, min(DCL_rmse(1,:)));
    y_min = min(y_min, min(DCL_rmse(2,:)));
    t_min = min(t_min, min(DCL_rmse(3,:)));
end

if exist('CKF_rmse','var')
    x_max = max(x_max, 3*std(CKF_rmse(1,:)) + mean(CKF_rmse(1,:)));
    y_max = max(y_max, 3*std(CKF_rmse(2,:)) + mean(CKF_rmse(2,:)));
    t_max = max(t_max, 3*std(CKF_rmse(3,:)) + mean(CKF_rmse(3,:)));
    x_min = min(x_min, min(CKF_rmse(1,:)));
    y_min = min(y_min, min(CKF_rmse(2,:)));
    t_min = min(t_min, min(CKF_rmse(3,:)));
end

figure(1); clf;
nBins = 30;
xEdges = linspace(round(x_min, 2), round(x_max, 2), nBins+1);
yEdges = linspace(round(y_min, 2), round(y_max, 2), nBins+1);
tEdges = linspace(round(t_min, 2), round(t_max, 3), nBins+1);

if exist('EKF_rmse','var')
    subplot(1,3,1); hold on; histogram(EKF_rmse(1,:), xEdges, 'Normalization','probability', 'DisplayName', 'EKF'); hold off;
    subplot(1,3,2); hold on; histogram(EKF_rmse(2,:), yEdges, 'Normalization','probability','DisplayName', 'EKF'); hold off;
    subplot(1,3,3); hold on; histogram(EKF_rmse(3,:), tEdges, 'Normalization','probability', 'DisplayName', 'EKF'); hold off;
end

if exist('DCL_rmse','var')
    subplot(1,3,1); hold on; histogram(DCL_rmse(1,:), xEdges, 'Normalization','probability','DisplayName', 'DCL'); hold off;
    subplot(1,3,2); hold on; histogram(DCL_rmse(2,:), yEdges, 'Normalization','probability', 'DisplayName', 'DCL'); hold off;
    subplot(1,3,3); hold on; histogram(DCL_rmse(3,:), tEdges, 'Normalization','probability', 'DisplayName', 'DCL'); hold off;
end

if exist('CKF_rmse','var')
    subplot(1,3,1); hold on; histogram(CKF_rmse(1,:), xEdges, 'Normalization','probability', 'DisplayName', 'CKF'); hold off;
    subplot(1,3,2); hold on; histogram(CKF_rmse(2,:), yEdges, 'Normalization','probability', 'DisplayName', 'CKF'); hold off;
    subplot(1,3,3); hold on; histogram(CKF_rmse(3,:), tEdges, 'Normalization','probability', 'DisplayName', 'CKF'); hold off;
end

subplot(1,3,1); legend; xlabel('X Error');
subplot(1,3,2); legend; xlabel('Y Error');
subplot(1,3,3); legend; xlabel('Theta Error');
