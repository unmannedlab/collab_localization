if t == 1
    EKF_x = zeros(6,nCars,nSimulations,nTicks);
    EKF_P = zeros(6,6,nCars,nSimulations,nTicks);
else
    
    
    % Predict Step at 400 Hz
    if mod(t, 3) == 0

    end

    % Pacmod Step at 30 Hz
    if mod(t, 40) == 0

    end

    % GPS step at 10 Hz
    if mod(t, 120) == 0

    end

    % UWB step at 3 Hz
    if mod(t, 400) == 0

    end
end