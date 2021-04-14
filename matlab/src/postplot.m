
nBins = 40;
ea = 0.3;
fa = 0.8;

p_max = max([EKF_rmse(1,:), CKF_rmse(1,:), DCL_rmse(1,:)]);
p_min = min([EKF_rmse(1,:), CKF_rmse(1,:), DCL_rmse(1,:)]);
p_step = min([  range(EKF_rmse(1,:)), ...
                range(EKF_rmse(1,:)), ...
                range(EKF_rmse(1,:))] / nBins);
p_edges = p_min:p_step:p_max;
      
t_max = max([EKF_rmse(2,:), CKF_rmse(2,:), DCL_rmse(2,:)]);
t_min = min([EKF_rmse(2,:), CKF_rmse(2,:), DCL_rmse(2,:)]);
t_step = min([  range(EKF_rmse(2,:)), ...
                range(EKF_rmse(2,:)), ...
                range(EKF_rmse(2,:))] / nBins);
t_edges = t_min:t_step:t_max;

figure(1); clf; hold on; 
histogram(EKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF');
histogram(DCL_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL');
histogram(CKF_rmse(1,:), p_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'CCL');
hold off;

% figure(2); clf;hold on; 
% histogram(EKF_rmse(2,:), t_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'EKF');
% histogram(DCL_rmse(2,:), t_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'DCL');
% histogram(CKF_rmse(2,:), t_edges, 'Normalization','probability', 'EdgeAlpha', ea, 'FaceAlpha', fa, 'DisplayName', 'CCL');
% hold off;

figure(1); legend; xlabel('Position Error');
% figure(2); legend; xlabel('Heading Error');

clear nBins ea fa;
clear p_min p_max;
clear t_min t_max;
clear p_edges t_edges;

