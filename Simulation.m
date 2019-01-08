%% Header
% This is the master function which handles the simulation


%% Simulation
if settings.simulate
    
    % Object for display of results
    animation = animate(core);  

    % Object simulating the system
    % system = dynamics(core);

    % Simulate the system
    % system.simulate(); 

    % Display results                                            
    animation.play();
end
                                            