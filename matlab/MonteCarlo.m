clf; clear; close all;

n = 10000;
err_ekf      = zeros(n,2);
err_ekf_lmk  = zeros(n,2);
err_dcl      = zeros(n,2);
err_dcl_lmk  = zeros(n,2);

parfor i = 1:n
    if mod(i,500) == 0
        disp(i);
    end
    
    Sim = Simulation('Street_Cross',false, [1, 1, 0, 1]);
    Sim.run_to(60, 0.1);
    
    err_ekf(i,:)     = [RMSE(Sim.err_ekf{1}(1000:end,1)),     ...
                        RMSE(Sim.err_ekf{1}(1000:end,2))];
                    
    err_ekf_lmk(i,:) = [RMSE(Sim.err_ekf_lmk{1}(1000:end,1)), ...
                        RMSE(Sim.err_ekf_lmk{1}(1000:end,2))];
    
    err_dcl(i,:)     = [RMSE(Sim.err_dcl{1}(1000:end,1)),     ...
                        RMSE(Sim.err_dcl{1}(1000:end,2))];
                    
    err_dcl_lmk(i,:) = [RMSE(Sim.err_dcl_lmk{1}(1000:end,1)), ...
                        RMSE(Sim.err_dcl_lmk{1}(1000:end,2))];

end



%%
figure(1);
hold on;
edges = 0:0.01:0.5;
h1 = histogram(err_ekf(:,1),     edges, 'Normalization','probability', 'DisplayName', 'EKF');
h2 = histogram(err_ekf_lmk(:,1), edges, 'Normalization','probability', 'DisplayName', 'EKF - Landmark');
h3 = histogram(err_dcl(:,1),     edges, 'Normalization','probability', 'DisplayName', 'DCL');
h4 = histogram(err_dcl_lmk(:,1), edges, 'Normalization','probability', 'DisplayName', 'DCL - Landmark');
legend;
xlabel('RMSE [m]')
ylabel('Frequency')


%% Functions 
function err = RMSE(a)
    err = sqrt(mean(a(:).^2));
end