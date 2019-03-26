%% Header
% This is the master function which handles the simulation

%% Simulation
if settings.simulate
    % Object for display of results
    [animation,core.param.animation_fig] = animate(core); 
    
    % Object simulating the system
    system = dynamics(core);

    % Simulate the system
    system.simulate(); 
    
%     disp("Paused: Press any key to continue...")
%     pause

    % Display results                                            
    animation.play();
end
                                            