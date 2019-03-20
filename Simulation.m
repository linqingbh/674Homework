%% Header
% This is the master function which handles the simulation

%% Simulation
if settings.simulate
    % Object simulating the system
    system = dynamics(core);

    % Simulate the system
    system.simulate(); 
    
    % Object for display of results
    animation = animate(core); 

    disp("Paused: Press any key to continue...")
    pause

    % Display results                                            
    animation.play();
end
                                            