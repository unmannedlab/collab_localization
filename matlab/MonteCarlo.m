clf; clear; close all;

n = 10;
errors_kf_tun_lmk  = zeros(n,3);
errors_uwb_tun_lmk = zeros(n,3);

for i = 1:n
    
    Sim = Simulation('Parallel',false, [1, 1, 1, 1]);

    Sim.run_to(20, 0.1);
    errors_kf_tun_lmk(i,:) = [   RMSE(Sim.err_out{1}(:,1)), ...
                                RMSE(Sim.err_out{1}(:,2)), ...
                                RMSE(Sim.err_out{1}(:,3))];

end

% parfor i = 1:n
%     
%     Sim = Simulation('Tunnel',false, [1, 0, 1]);
%     
%     Sim.run_to(20, 0.1);
%     errors_uwb_sc_lmk(i,:) = [  RMSE(Sim.err_out{1}(:,1)), ...
%                                 RMSE(Sim.err_out{1}(:,2)), ...
%                                 RMSE(Sim.err_out{1}(:,3))];
% 
% end

%%
edges = 0:0.5:10;
h1 = histogram(errors_kf_tun_lmk(:,1)*10,edges,'Normalization','probability');
hold on;
h2 = histogram(errors_uwb_tun_lmk(:,1),edges,'Normalization','probability');
legend('KF - No UWB', 'KF - With UWB');
xlabel('RMSE [m]')
ylabel('Frequency')


%% Functions 
function err = RMSE(a)
    err = sqrt(mean(a(:).^2));
end