clf; clear; close all;

n = 10000;
% err_ekf      = zeros(n,2);
% err_ekf_lmk  = zeros(n,2);
err_dcl      = zeros(n,2);
err_dc2      = zeros(n,2);
% err_dcl_lmk  = zeros(n,2);

tic
parfor i = 1:n
    if mod(i, 500) == 0
        disp(i)
    end
    
    Sim = Simulation('Parallel',false, [1, 0, 1, 1]);
    Sim.run_to(5, 0.1);
    
%     err_ekf(i,:)     = [RMSE(Sim.err_ekf{1}(:,1)),     ...
%                         RMSE(Sim.err_ekf{1}(:,2))];
                    
%     err_ekf_lmk(i,:) = [RMSE(Sim.err_ekf_lmk{1}(:,1)), ...
%                         RMSE(Sim.err_ekf_lmk{1}(:,2))];
    
    err_dcl(i,:)     = [RMSE(Sim.err_dcl{1}(:,1)),     ...
                        RMSE(Sim.err_dcl{1}(:,2))];
                    
    err_dc2(i,:)     = [RMSE(Sim.err_dc2{1}(:,1)),     ...
                        RMSE(Sim.err_dc2{1}(:,2))];
                    
%     err_dcl_lmk(i,:) = [RMSE(Sim.err_dcl_lmk{1}(:,1)), ...
%                         RMSE(Sim.err_dcl_lmk{1}(:,2))];

end
toc

%%
figure(1);
hold on;
edges1 = 0:0.1:4;
% histogram(err_ekf(:,1),'DisplayName','EKF');
histogram(err_dcl(:,1),'DisplayName','DCL');
histogram(err_dc2(:,1),'DisplayName','DC2');
% h3 = histogram(err_dcl(:,1),     edges1, 'Normalization','probability', 'DisplayName', 'DCL', 'FaceColor', [0, 0.4470, 0.7410]);
% h1 = histogram(err_ekf(:,1),     edges1, 'Normalization','probability', 'DisplayName', 'EKF', 'FaceColor', [0.9290, 0.6940, 0.1250]);
legend;
xlabel('RMSE [m]')
ylabel('Frequency')

% figure(2);
% hold on;
% edges2 = 0:0.0025:0.2;
% h2 = histogram(err_ekf_lmk(:,1), edges2, 'Normalization','probability', 'DisplayName', 'EKF - Landmark', 'FaceColor', 	[0.8500, 0.3250, 0.0980]);
% h4 = histogram(err_dcl_lmk(:,1), edges2, 'Normalization','probability', 'DisplayName', 'DCL - Landmark', 'FaceColor', 	[0.4940, 0.1840, 0.5560]);
% legend;
% xlabel('RMSE [m]')
% ylabel('Frequency')


%% Functions 
function err = RMSE(a)
    err = sqrt(mean(a(:).^2));
end