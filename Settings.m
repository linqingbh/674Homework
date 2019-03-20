%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% profile on
rng('default')

% Add Path
Parameters;

%% Initial Conditions
% Trim
param.optimizer.name = "fminsearch"; % fminsearch gradiant
param.trim.V_a = 25;
param.trim.R = Inf;
param.trim.gamma = 0*pi/180;
param.trim.h_0 = 100;
core.functions = functions;
core.param = param;
[param.u_0,param.x_0,param.y_r_0] = functions.get_equilibrium('throw',core);
param.d_0 = [param.wind.base;0;0;0];
d_0s = zeros(size(param.d_names));
x_0s = zeros(size(param.x_names));
z_0s = zeros(size(param.z_names));
m_0s = zeros(size(param.m_names));
r_0s = zeros(size(param.r_names));
u_0s = zeros(size(param.u_names));

param.default.b = [0;0;-100];
param.default.rho = 200;
param.default.q = [0;0;1];
param.default.limits = [0,2*pi];

%% Model
assumed_param = param;
assumed_param.wind = wind(wind.steady,[0;0;0],param.aircraft.V_design);
[param.A,param.B] = get_linear_model(functions.eqs_motion,assumed_param);

%% Input Parameters
waypoints = [500,-500,500,-500;
             500,500,-500,-500;
             -100,-100,-100,-100;
             0,pi/2,pi,3*pi/2];
core.publish_list("W",waypoints);



%% Path Follower
core.param = param;% Make unessisary
follow.x_names = ["p_{n}";"p_{e}";"p_{d}"];
follow.r_names = ["\chi","h","V_a"];
follow.k.line = 0.01;
follow.k.orbit = 2;
follow.chi_inf = pi/3;
functions.followers(1) = path_follower(follow,core);

%% Path Manager
manage.type = path_manager.dubins;
manage.x_names = ["p_{n}";"p_{e}";"p_{d}";"\psi"];
manage.W = waypoints;
functions.manager(1) = path_manager(manage,core);

%% Simulation Parameters
settings.active_fig  = 1;
settings.animation   = true;
settings.plot        = true;
settings.simulate    = true;
settings.progress_update = true;
pause off
settings.plot_names  = {% ["p_{n} - Longitude (m)","p_{n}"];
%                         ["p_{e} - Latitude (m)","p_{e}"];

%                         ["p_{d} - Altitude (m)","y_r_hat_{h}","y_r_{h}","r_{h}"];
%                         ["\theta - Pitch (rad)","y_r_hat_{\theta}","y_r_{\theta}","r_{\theta}"];
%                         ["\chi - Course (rad)","y_r_hat_{\chi}","y_r_{\chi}","r_{\chi}"];
%                         ["\phi - Roll (rad)","y_r_hat_{\phi}","y_r_{\phi}","r_{\phi}"];
%                         ["\beta - Sideslip (rad)","y_r_hat_{\beta}","y_r_{\beta}","r_{\beta}"]
%                         ["V_a - Forward Velocity (m/s)","y_r_hat_{V_a}","y_r_{V_a}","r_{V_a}"];
                        
%                         ["\delta_{a} - Ailorons (rad)","delta_a"];
%                         ["\delta_{e} - Elevator (rad)","delta_e"];
%                         ["\delta_{r} - Ruder (rad)","delta_r"];
%                         ["\delta_{t} - Throttle (%)","delta_t"];
                        ["y_{r}_{dot} - Rate of Change","y_r_dot_{\chi}"]%,"y_r_dot_{\theta}","y_r_dot_{\chi}","y_r_dot_{\phi}","y_r_dot_{\beta}","y_r_dot_{V_a}"]

%                         ["p_{n} - Longitude (m)","p_{n}","z_hat_{GPS_n}","z_f_{GPS_n}"];
%                         ["p_{e} - Latitude (m)","p_{e}","z_hat_{GPS_e}","z_f_{GPS_e}"];
%                         ["h - Altitude (m)","y_r_{h}","z_hat_{GPS_h}","z_f_{GPS_h}"];
%                         ["\chi - Course (rad)","y_r_{\chi}","z_hat_{GPS_chi}","z_f_{GPS_chi}"];
%                         ["Bar - Barometer (Pa)","z_hat_{Bar}","z_f_{Bar}"];
%                         ["Pito - Pito Tube (Pa)","z_hat_{Pito}","z_f_{Pito}"];
%                         ["Comp - Compass Heading (rad)","\psi","z_hat_{Comp}","z_f_{Comp}"];
%                         ["a - Accelormeter (m\s^{2})","z_hat_{Accel_x}","z_hat_{Accel_y}","z_hat_{Accel_z}","z_f_{Accel_x}","z_f_{Accel_y}","z_f_{Accel_z}"];
%                         ["Omega_dot - Rate Gyro (rad/s)","z_hat_{RateGyro_p}","z_hat_{RateGyro_q}","z_hat_{RateGyro_r}","z_f_{RateGyro_p}","z_f_{RateGyro_q}","z_f_{RateGyro_r}"]
                          
%                         ["p_{n} - Longitude (m)","p_{n}","y_m_{p_{n}}"];
%                         ["p_{e} - Latitude (m)","p_{e}","y_m_{p_{e}}"];
%                         ["h - Altitude (m)","p_{d}","y_m_{p_{d}}"];
%                         ["u_a - Forward Velocity (m/s)","u","y_m_{u_a}"];
%                         ["\phi - Roll (rad)","\phi","y_m_{\phi}"];
%                         ["\theta - Pitch (rad)","\theta","y_m_{\theta}"];
%                         ["\psi - Yaw (rad)","\psi","y_m_{\psi}"];
%                         ["p - Roll Rate (rad\s)","p","y_m_{p}"];
%                         ["q - Pitch Rate (rad\s)","q","y_m_{q}"];
%                         ["r - Yaw Rate (rad\s)","r","y_m_{r}"];
%                         ["\chi - Course (rad)","y_r_{\chi}","y_m_{\chi}"]
%                         ["V_gh - Horizontal Velcoity (m/s)","y_r_{V_a}","y_m_{V_gh}"]

%                         ["p_{n} - Longitude (m)","p_{n}","y_m_hat_{p_{n}}"];
%                         ["p_{e} - Latitude (m)","p_{e}","y_m_{p_{e}}"];
%                         ["h - Altitude (m)","p_{d}","y_m_{p_{d}}"];
%                         ["u_a - Forward Velocity (m/s)","u","y_m_{u_a}"];
%                         ["\phi - Roll (rad)","\phi","y_m_{\phi}"];
%                         ["\theta - Pitch (rad)","\theta","y_m_{\theta}"];
%                         ["\psi - Yaw (rad)","\psi","y_m_{\psi}"];
%                         ["p - Roll Rate (rad\s)","p","y_m_{p}"];
%                         ["q - Pitch Rate (rad\s)","q","y_m_{q}"];
%                         ["r - Yaw Rate (rad\s)","r","y_m_{r}"];
%                         ["\chi - Course (rad)","y_r_{\chi}","y_m_{\chi}"]
%                         ["V_gh - Horizontal Velcoity (m/s)","y_r_{V_a}","y_m_{V_gh}"]

%                         ["w_n - North Wind (m/s)","w_n"]
%                         ["w_e - East Wind (m/s)","w_e"]
%                         ["w_d - Down Wind (m/s)","w_d"]
% 
%                           ["e - Position Error (m)";
%                           "x_hat_e_{p_{n}}";
%                           "x_hat_e_{p_{e}}";
%                           "x_hat_e_{p_{d}}"
%                           ].'
%                           
%                           ["e - Velocity Error (m/s)";
%                           "x_hat_e_{u}";
%                           "x_hat_e_{v}";
%                           "x_hat_e_{w}"
%                           ].'
%                           
%                           ["e - Angle Error (rad)";
%                           "x_hat_e_{\phi}";
%                           "x_hat_e_{\theta}";
%                           "x_hat_e_{\psi}"
%                           ].'
%                           
%                           ["e - Angular Rate Error (rad/s)";
%                           "x_hat_e_{p}";
%                           "x_hat_e_{q}";
%                           "x_hat_e_{r}"
%                           ].'
% 
%                           ["w_n - North Wind","w_n","d_hat_{w_n}"];
%                           ["w_e - East Wind","w_e","d_hat_{w_e}"];
%                           ["w_d - Down Wind","y_m_{w_d}","y_m_hat_{w_d}"];
                          
%                          ["a - Accelormeter (m\s^{2})","z_f_{Accel_x}","z_f_{Accel_y}","z_f_{Accel_z}"];
%                          ["Omega_dot - Rate Gyro (rad/s)","p","q","r"]
%                          ["u - Foward Velcoity (m)","u","x_hat_{u}"];
%                          ["v - Sideways Velcoity (m)","v","x_hat_{v}"];

%                             ["e_h - Altitude Error (m)","y_r_e_{h}"]
                            ["e_{\chi} - Course Error (m)","y_r_e_{\chi}"]
                        };
                    
% Display warnings?
warning('off','all')

%% Publish Data
core.publish_list("t",t)
core.publish("r",r_0s)
core.publish("d",param.d_0)
core.publish("x",param.x_0)
core.publish("x_dot",x_0s)
core.publish("z",z_0s)
core.publish("z_f",z_0s)
core.publish("z_hat",z_0s)
core.publish("y_m",m_0s)
core.publish("y_m_hat",m_0s)
core.publish("x_hat",param.x_0)
core.publish("d_hat",param.d_0)
core.publish("y_r",param.y_r_0)
core.publish("y_r_hat",r_0s)
core.publish("y_r_dot",r_0s)
core.publish("y_r_dot_hat",r_0s)
core.publish("u",param.u_0)
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
sense.perfect = false;
sense.d_names = ["w_n","w_e","w_d"];

sense.type = sensors.GPS;
sense.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi"];
sense.z_names = ["GPS_n";"GPS_e";"GPS_h";"GPS_Vg";"GPS_chi"];
core.functions.sensors(1) = sensors(sense,param);

sense.type = sensors.Bar;
sense.x_names = "p_{d}";
sense.z_names = "Bar";
core.functions.sensors(2) = sensors(sense,param);

sense.type = sensors.Pito;
sense.x_names = ["p_{d}","u","\phi","\theta","\psi"];
sense.z_names = "Pito";
core.functions.sensors(3) = sensors(sense,param);

sense.type = sensors.Comp;
sense.x_names = "\psi";
sense.z_names = "Comp";
core.functions.sensors(4) = sensors(sense,param);

sense.type = sensors.Accel;
sense.x_names = ["u","v","w","\phi","\theta","p","q","r"];
sense.z_names = ["Accel_x";"Accel_y";"Accel_z"];
core.functions.sensors(5) = sensors(sense,param);

sense.type = sensors.RateGyro;
sense.x_names = ["p","q","r"];
sense.z_names = ["RateGyro_p";"RateGyro_q";"RateGyro_r"];
core.functions.sensors(6) = sensors(sense,param);

% Measure
z_0 = zeros(size(param.z_names));
for j = 1:length(core.functions.sensors)
    [~,z_0(core.functions.sensors(j).z_indexes)] = core.functions.sensors(j).sense(param.x_0,functions.eqs_motion(0,param.x_0,param.u_0,param),param.wind.base,0);
end
y_m_0 = functions.get_y_m(z_0,param);
core.publish_specific("z",z_0,1)
core.publish_specific("z_hat",z_0,1)
core.publish_specific("y_m",y_m_0,1)
core.publish_specific("y_m_hat",y_m_0,1)

%% Filter
core.functions.filters = my_filter([0,... GPS_n
                                    0,... GPS_e
                                    0,... GPS_h
                                    10,... GPS_Vg
                                    10,... GPS_chi
                                    10,... Bar
                                    10,... Pito
                                    10,... Comp
                                    10,... Accel_x
                                    10,... Accel_y
                                    10,... Accel_z
                                    10,... RateGyro_p
                                    10,... RateGyro_q
                                    10],...RateGyro_r
                                    ...
                                    [0.5,... GPS_n
                                     0.5,... GPS_e
                                     0.5,...   GPS_h
                                     0.5,... GPS_Vg
                                     0.5,... GPS_chi
                                     0.5,...   Bar
                                     0.5,...   Pito
                                     0.5,...   Comp
                                     0.5,...   Accel_x
                                     0.5,...   Accel_y
                                     0.5,...   Accel_z
                                     0.4,...   RateGyro_p
                                     0.4,...   RateGyro_q
                                     0.4...    RateGyro_r
                                     ],z_0);
core.functions.filters.update_every_step = false;

%% State Observers

observe.type = observers.exact;
observe.x_names = param.x_names;
observe.r_names = [];
observe.m_names = param.m_names;%["p_{n}";"p_{e}";"p_{d}";"u_a";"Accel_x";"Accel_y";"Accel_z";"\psi";"p";"q";"r";"\chi";"V_gh"];
observe.u_names = param.u_names;
observe.d_names = ["w_n","w_e","w_d"];
        %p_n;p_e;p_d;   u;   u;   w; phi;theta;psi;   p;   q;   r;
gains = [1000,1000,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;    % P_n
         1000,1000,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;    % P_e
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % P_d
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % u
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % v
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % w
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % phi
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % theta
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % psi
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % p
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;  % q
         0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]; % r
d_gains = [0.01,0,0;
           0,0.01,0;
           0,0,0];
observe.L.Q = [(my_cov(@(state) functions.eqs_motion(0,state,param.u_0,param),param.x_0)).*gains,zeros(length(param.x_names),3);zeros(3,length(param.x_names)),d_gains];
core.functions.observers(1) = observers(observe,core);

% observe.type = observers.ekf;
% observe.x_names = [];
% observe.r_names = [];
% observe.u_names = [];
% observe.d_names = ["w_n","w_e","w_d"];
% observe.m_names = ["w_n","w_e","w_d"];
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

control.anti_windup = 'none'; % 'derivative', 'saturation', 'both', 'none'

control.d_names = [];

switch control.type
    case controllers.OL
        % General
        control.windup_limit = 0;
        control.sat_lim.high = Inf;
        control.sat_lim.low = -Inf;
        control.K.I = 0;
        
        % Ailorons
        control.r_names = "h";
        control.u_names = "delta_a";
        control.plan = line;
        control.t_vec = t;
        core.functions.controllers(1) = controllers(control,core);
        
        % Elevator
        control.r_names = "h";
        control.u_names = "delta_r";
        control.plan = line;
        control.t_vec = t;
        core.functions.controllers(2) = controllers(control,core);
        
        % Rudder
        control.r_names = "h";
        control.u_names = "delta_e";
        control.plan = line;
        control.t_vec = t;
        core.functions.controllers(3) = controllers(control,core);
        
        % Throttle
        control.r_names = "h";
        control.u_names = "delta_t";
        control.plan = line;
        control.t_vec = t;
        core.functions.controllers(4) = controllers(control,core);
    case controllers.PID
        % Course hold
        control.windup_limit = 0.3;
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
        control.K.P = 2*zeta_h*w_n_h/(K_theta_DC*param.aircraft.V_design);
        control.K.I = w_n_h^2/(K_theta_DC*param.aircraft.V_design);
        control.K.D = 0;
        control.r_names = "h";
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
        control.u_names = "delta_e";
        core.functions.controllers(5) = controllers(control,core); 
end

%% Run Program
Simulation;

% profile viewer

