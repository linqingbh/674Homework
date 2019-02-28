%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
Parameters;

%% Initial Conditions
% Trim
param.optimizer.name = "fminsearch"; % fminsearch gradiant
param.trim.V_a = 25;
param.trim.R = Inf;
param.trim.gamma = 0*pi/180;
param.trim.h_0 = 100;
[param.u_0,param.x_0,param.y_r_0] = functions.get_equilibrium('throw',param,functions);
y_m_0 = [param.x_0(1:3);param.trim.V_a;param.x_0(7:12);param.x_0(9);param.trim.V_a;0;0;0];

x_0s = zeros(size(param.x_names));
sensor_0s = zeros(size(param.sensor_names));
m_0s = zeros(size(param.m_names));
r_0s = zeros(size(param.r_names));
u_0s = zeros(size(param.u_names));

%% Model
assumed_param = param;
assumed_param.wind = wind(wind.steady,[0;0;0],param.trim.V_a);
[param.A,param.B] = get_linear_model(functions.eqs_motion,assumed_param);

%% Input Parameters
input       = 'line';  % Type of signal
period      = 10;        % Period of signal input
amplitude   = 1;        % Amplitude of signal input
offset      = 0;            % Offset from 0
phase_delay = 0;        % phase delay of function in rad
line = function_generator(input,period,amplitude,offset,phase_delay,t);
input       = 'square';  % Type of signal
period      = 40;        % Period of signal input
amplitude   = 0.5;        % Amplitude of signal input
offset      = 0.5;            % Offset from 0
phase_delay = 0;        % phase delay of function in rad
square1 = function_generator(input,period,amplitude,offset,phase_delay,t);
phase_delay = pi/3;        % phase delay of function in rad
square2 = function_generator(input,period,amplitude,offset,phase_delay,t);
phase_delay = 2*pi/3;        % phase delay of function in rad
square3 = function_generator(input,period,amplitude,offset,phase_delay,t);
% Commanded Inputs [chi,phi,h,theta,beta,V_a]
r = [square1*pi;line;square2*50+100;line;line;square3*10+param.trim.V_a];

%% Simulation Parameters
settings.animation   = true;
settings.plot        = true;
settings.real_time   = false;
settings.simulate    = true;
settings.plot_names  = {%["p_{n} - Longitude (m)","p_{n}"];
                        %["p_{e} - Latitude (m)","p_{e}"];
                        ["p_{d} - Altitude (m)","p_{d}","r_{p_{d}}"];
                        ["\theta - Pitch (rad)","\theta","r_{\theta}"];%,"y_{\theta}_{dot}"];
                        ["\chi - Course (rad)","y_r_{\chi}","r_{\chi}"];
                        ["\phi - Roll (rad)","\phi","r_{\phi}"];%,"y_{\phi}_{dot}"];
                        ["\beta - Sideslip (rad)","y_r_{\beta}","r_{\beta}"];
                        %["\psi - Yaw (rad)","\psi"]
                        ["V_a - Forward Velocity (m/s)","y_r_{V_a}","r_{V_a}"];
                        ["\delta_{a} - Ailorons (rad)","delta_a"];
                        ["\delta_{e} - Elevator (rad)","delta_e"];
                        ["\delta_{r} - Ruder (rad)","delta_r"];
                        ["\delta_{t} - Throttle (%)","delta_t"];
                        ["y_{r}_{dot} - Rate of Change","y_r_dot_{p_{d}}","y_r_dot_{\theta}","y_r_dot_{\chi}","y_r_dot_{\phi}","y_r_dot_{\beta}","y_r_dot_{V_a}"]
                        %["h_dot - Rate of Climb (m/s)","y_{h}_{dot}"]
                        };
settings.active_fig  = 2;
settings.show_hist   = true;
                    
% General Variable to implement uncertainty
settings.implement_uncertainty = false;
                    
% Display warnings?
warning('off','all')

%% Publish Data
core.publish_list("t",t)
core.publish_list("r",r)
core.publish("x",param.x_0)
core.publish("x_dot",x_0s)
core.publish("sensor_data",sensor_0s)
core.publish("y_m",y_m_0)
core.publish("x_hat",x_0s)
core.publish("d_hat",r_0s)
core.publish("u",u_0s)
core.publish("y_r",param.y_r_0)
core.publish("y_r_dot",r_0s)
core.settings = settings;
core.param = param;
core.functions = functions;

%% Parameters for Tuning
control.type = controllers.PID;

W_chi = 10;
zeta_chi = 0.707;
e_max_phi = 45*pi/180;
zeta_phi = 0.707;

e_max_beta = 45*pi/180;
zeta_beta = 0.707;

W_h = 25;
zeta_h = 0.707;
e_max_theta = 10*pi/180;
zeta_theta = 0.707;

W_V_2 = 10;
zeta_V_2 = 0.707;
t_r_V = 3;
zeta_V = 0.707;

%% Sensors
sense.exact = true;

sense.type = sensors.GPS;
sense.x_names = ["p_{n}";"p_{e}";"p_{d}"];
sense.sensor_names = ["GPS_n";"GPS_e";"GPS_h";"GPS_Vg";"GPS_chi"];
core.functions.sensors(1) = sensors(sense,param);

sense.type = sensors.Bar;
sense.x_names = "p_{d}";
sense.sensor_names = "Bar";
core.functions.sensors(2) = sensors(sense,param);

sense.type = sensors.Pito;
sense.x_names = "u";
sense.sensor_names = "Pito";
core.functions.sensors(3) = sensors(sense,param);

sense.type = sensors.Comp;
sense.x_names = "\psi";
sense.sensor_names = "Comp";
core.functions.sensors(4) = sensors(sense,param);

% sense.type = sensors.Accel;
% sense.x_names = ["u","v","w","\phi","\theta","p","q","r"];
% sense.sensor_names = ["Accel_x";"Accel_y";"Accel_z"];
% core.functions.sensors(5) = sensors(sense,param);

sense.type = sensors.Exact;
sense.x_names = ["\phi","\theta","\psi"];
sense.sensor_names = ["Accel_x";"Accel_y";"Accel_z"];
core.functions.sensors(5) = sensors(sense,param);

sense.type = sensors.RateGyro;
sense.x_names = ["p","q","r"];
sense.sensor_names = ["RateGyro_p";"RateGyro_q";"RateGyro_r"];
core.functions.sensors(6) = sensors(sense,param);

sense.type = sensors.Exact;
sense.x_names = ["u","v","w"];
sense.sensor_names = ["u";"v";"w"];
core.functions.sensors(7) = sensors(sense,param);

%% Observers
observe.L.sigma = 0.05;
observe.r_names = [];
observe.u_names = [];

observe.type = observers.exact;
observe.m_names = ["p_{n}";"p_{e}";"p_{d}";"u_w";"v_w";"w_w";"\phi";"\theta";"\psi";"p";"q";"r"];
observe.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi";"p";"q";"r"];
core.functions.observers(1) = observers(observe,core);

% observe.type = observers.dy;
% observe.m_names = ["p_{n}";"p_{e}";"p_{d}"];
% observe.x_names = ["u","v","w"];
% core.functions.observers(2) = observers(observe,core);

%% Controllers

[a_phi_1,a_phi_2,a_beta_1,a_beta_2,a_theta_1,a_theta_2,a_theta_3,a_V_1,a_V_2,a_V_3] = functions.get_tf_coefficents(param);
V_g = norm(param.x_0(4:6));
g = param.g;

w_n_phi = sqrt(abs(a_phi_2)*param.phi_sat_lim.high/e_max_phi);
w_n_chi = 1/W_chi*w_n_phi;
w_n_beta = (a_beta_1+a_beta_2*param.delta_r_sat_lim.high/e_max_beta)/(2*zeta_beta);
w_n_theta = sqrt(a_theta_2+param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3);
w_n_h = 1/W_h*w_n_theta;
w_n_V = 2.2/t_r_V;
w_n_V_2 = 1/W_V_2*w_n_theta;

K_theta_DC = param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3/(a_theta_2+param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3);

control.anti_windup = 'derivative'; % 'derivative', 'saturation', 'both', 'none'

switch control.type
    case controllers.OL
        control.plan = plan;
        control.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi";"p";"q";"r"];
        control.u_names = ["delta_a";"delta_e";"delta_r";"delta_t"];
        control.impose_sat = false;
        core.functions.controllers(1) = controllers(control,core);
    case controllers.PID
        % Course hold
        control.windup_limit = 0.01;
        control.sat_lim.high = param.phi_sat_lim.high;
        control.sat_lim.low = param.phi_sat_lim.low;
        control.K.P = 2*zeta_chi*w_n_chi*V_g/g;
        control.K.I = w_n_chi^2*V_g/g;
        control.K.D = 0;
        control.r_names = "\chi";
        control.u_names = "\phi";
        core.functions.controllers(1) = controllers(control,core);
        % Roll attitude hold
        control.sat_lim.high = param.delta_a_sat_lim.high;
        control.sat_lim.low = param.delta_a_sat_lim.low;
        control.K.P = control.sat_lim.high/e_max_phi;
        control.K.I = 0;
        control.K.D = (2*zeta_phi*w_n_phi-a_phi_1)/a_phi_2;
        control.r_names = "\phi";
        control.u_names = "delta_a";
        core.functions.controllers(1).cascade = controllers(control,core);
        % Sideslip mitigation
        control.windup_limit = 0.0001;
        control.sat_lim.high = param.delta_r_sat_lim.high;
        control.sat_lim.low = param.delta_r_sat_lim.low;
        control.K.P = control.sat_lim.high/e_max_beta;
        control.K.I = w_n_beta^2/a_beta_2;
        control.K.D = 0;
        control.r_names = "\beta";
        control.u_names = "delta_r";
        core.functions.controllers(2) = controllers(control,core);

        % Altitude hold
        control.windup_limit = 1;
        control.sat_lim.high = param.theta_sat_lim.high;
        control.sat_lim.low = param.theta_sat_lim.low;
        control.K.P = 2*zeta_h*w_n_h/(K_theta_DC*param.V_design);
        control.K.I = w_n_h^2/(K_theta_DC*param.V_design);
        control.K.D = 0;
        control.r_names = "p_{d}";
        control.u_names = "\theta";
        core.functions.controllers(3) = controllers(control,core);
        % Ptch attitude hold
        control.sat_lim.high = param.delta_e_sat_lim.high;
        control.sat_lim.low = param.delta_e_sat_lim.low;
        control.K.P = control.sat_lim.high/e_max_theta*sign(a_theta_3);
        control.K.I = 0;
        control.K.D = (2*zeta_theta*w_n_theta-a_theta_1)/a_theta_3;
        control.r_names = "\theta";
        control.u_names = "delta_e";
        core.functions.controllers(3).cascade = controllers(control,core);

        % Throttle airspeed hold
        control.windup_limit = 1;
        control.sat_lim.high = param.delta_t_sat_lim.high;
        control.sat_lim.low = param.delta_t_sat_lim.low;
        control.K.P = (2*zeta_V*w_n_V-a_V_1)/a_V_2;
        control.K.I = w_n_V^2/a_V_2;
        control.K.D = 0;
        control.r_names = "V_a";
        control.u_names = "delta_t";
        core.functions.controllers(4) = controllers(control,core);
        % Pitch airspeed hold
        control.sat_lim.high = param.delta_t_sat_lim.high;
        control.sat_lim.low = param.delta_t_sat_lim.low;
        control.K.P = (a_V_1-2*zeta_V_2*w_n_V_2)/(K_theta_DC*param.g);
        control.K.I = w_n_V_2^2/(K_theta_DC*param.g);
        control.K.D = 0;
        control.r_names = "V_a";
        control.u_names = "delta_t";
        core.functions.controllers(5) = controllers(control,core); 
end

%% Run Program
Simulation;

