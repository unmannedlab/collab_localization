clf; clear; close all;

n = 10;
errors_kf_tun_lmk  = zeros(n,3);
errors_uwb_tun_lmk = zeros(n,3);

for i = 1:n
    
    Sim = Simulation('Tunnel',false, [1, 0, 0]);

    Sim.run_to(20, 0.1);
    errors_kf_tun_lmk(i,:) = [   RMSE(Sim.err_out{1}(:,1)), ...
                                RMSE(Sim.err_out{1}(:,2)), ...
                                RMSE(Sim.err_out{1}(:,3))];

end

for i = 1:n
    
    Sim = Simulation('Tunnel',false, [1, 0, 1]);
    
    Sim.run_to(20, 0.1);
    errors_uwb_tun_lmk(i,:) = [  RMSE(Sim.err_out{1}(:,1)), ...
                                RMSE(Sim.err_out{1}(:,2)), ...
                                RMSE(Sim.err_out{1}(:,3))];

end

%%

h1 = histogram(errors_kf_tun_lmk(:,1),50,'Normalization','probability','EdgeColor','none');
hold on;
h2 = histogram(errors_uwb_tun_lmk(:,1),50,'Normalization','probability','EdgeColor','none');
legend('KF - No UWB', 'KF - With UWB');
xlabel('RMSE [m]')
ylabel('Frequency')


%% Functions 
function err = RMSE(a)
    err = sqrt(mean(a(:).^2));
end