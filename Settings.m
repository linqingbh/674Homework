%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
Parameters;

%% Initial Conditions
x_0 = [0;0;0];

%% Input Parameters
input       = 'line';  % Type of signal
period      = 100;        % Period of signal input
amplitude   = 0;        % Amplitude of signal input
offset      = 0;            % Offset from 0
phase_delay = 0;        % phase delay of function in rad
line = function_generator(input,period,amplitude,offset,phase_delay,t);
x = [line;line;line];

%% Simulation Parameters
settings.animation   = true;
settings.real_time   = false;
settings.loopshaping = true;
settings.simulate    = true;
settings.plot_names  = {["\theta - Pitch (rad)","\theta"];
                        ["\phi - Roll (rad)","\phi"];
                        ["\psi - Yaw (rad)","\psi"]};
                    
% Display warnings?
warning('off','all')

%% Publish Data
core.publish_list("x",x)
core.publish_list("x_hat",x)
core.publish_list("u",zeros(1,length(x)))
core.publish_list("r",x)
core.publish_list("t",t)
core.settings = settings;
core.param = param;
core.functions = functions;

%% Run Program
Simulation;
