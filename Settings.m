%% Header 

% Homework Specific Parameters

% Clear Previous Info
clear all
clc

% Add Path
Parameters;

% This is the version I want.

%% Initial Conditions
% Trim
param.optimizer.name = "gradiant"; % fminsearch gradiant
param.optimizer.epsilon = 0.000001;
param.optimizer.N = 1000;
param.optimizer.kappa = 0.00001;
param.trim.V_a = 25;
param.trim.R = Inf;
param.trim.gamma = 0*pi/180;
[param.u_0,param.x_0] = functions.u_e('throw',param);
x_0 = param.x_0;
x_0(3) = -100;
[y_r_0,y_r_dot_0] = functions.y_r(x_0,core);
param.u_0 = [0;-0.1251;0;0.3144];
param.x_0 = [0;0;-100;24.9686;0;1.2523;0;0.0501;0;0;0;0];

%% Model

Dispaly_Linear_Model = false; 

% Assume trimmed and no wind
V_g = norm(x_0(4:6));
V_a = param.trim.V_a;
alpha = atan2(x_0(6),x_0(4));
theta = x_0(8);
delta_e = param.u_0(2);
delta_t = param.u_0(4);

% Aircraft Parameters
g           = param.g;
rho         = param.rho;
S           = param.S_wing;
b           = param.b;
c           = param.c;
C_l_p       = param.C_l_p;
C_l_delta_a = param.C_l_delta_a;
C_n_p       = param.C_n_p;
C_n_delta_a = param.C_n_delta_a;
C_m_q       = param.C_m_q;
C_m_alpha   = param.C_m_alpha;
C_m_delta_e = param.C_m_delta_e;
C_D_0       = param.C_D_0;
C_D_alpha   = param.C_D_alpha;
C_D_delta_e = param.C_D_delta_e;
C_Y_beta    = param.C_Y_beta;
C_Y_delta_r = param.C_Y_delta_r;
S_prop      = param.S_prop;
C_prop      = param.S_prop;
m           = param.mass;
Jx          = param.Jx;
Jy          = param.Jy;
Jz          = param.Jz;
Jxz         = param.Jxz;
k_motor     = param.k_motor;

Gamma(1) = Jxz*(Jx-Jy+Jz)/(Jx*Jz-Jxz^2);
Gamma(2) = (Jz*(Jz-Jy)+Jxz^2)/(Jx*Jz-Jxz^2);
Gamma(3) = Jz/(Jx*Jz-Jxz^2);
Gamma(4) = Jxz/(Jx*Jz-Jxz^2);
Gamma(5) = (Jz-Jx)/Jy;
Gamma(6) = Jxz/Jy;
Gamma(7) = ((Jx-Jy)*Jx+Jxz^2)/(Jx*Jz-Jxz^2);
Gamma(8) = Jx/(Jx*Jz-Jxz^2);

C_p_p = Gamma(3)*C_l_p + Gamma(4)*C_n_p;
C_p_delta_a = Gamma(3)*C_l_delta_a + Gamma(4)*C_n_delta_a;

% Shorthand
a_phi_1     = 1/4*rho*V_a*S*b^2*C_p_p;
a_phi_2     = 1/2*rho*V_a^2*S*b*C_p_delta_a;
a_beta_1    = -rho*V_a*S/(2*m)*C_Y_beta; 
a_beta_2    = rho*V_a*S/(2*m)*C_Y_delta_r;
a_theta_1   = -rho*V_a*c^2*S/(4*Jy)*C_m_q;
a_theta_2   = -rho*V_a^2*c*S/(2*Jy)*C_m_alpha;
a_theta_3   = rho*V_a^2*c*S/(2*Jy)*C_m_delta_e;
a_V_1       = rho*V_a*S/m*(C_D_0+C_D_alpha*alpha+C_D_delta_e*delta_e) + rho*S_prop/m*C_prop*V_a;
a_V_2       = rho*S_prop/m*C_prop*k_motor^2*delta_t;
a_V_3       = g;

% Lateral
delta_a2phi     = tf(a_phi_2,[1,a_phi_1,0]);
% d_phi_22phi     = tf(1,[1,a_phi_1,0]);
% d_chi2chi       = tf(g/V_g,[1,0]);
phi2chi         = tf(g/V_g,[1,0]);
delta_a2chi     = minreal(delta_a2phi*phi2chi);

% Sideslip
delta_r2beta    = tf(a_beta_2,[1,a_beta_1]);
% d_phi_2         = tf(a_beta_2,[1,a_beta_1]);
delta_r2v       = minreal(V_a*delta_r2beta);

% Airspeed
delta_t2V_a     = tf(a_V_2,[1,a_V_1]);
theta2V_a       = tf(-a_V_3,[1,a_V_1]);
% d_V2V_a         = tf(1,[1,a_V_1]);

% Longitudinal
delta_e2theta   = tf(a_theta_3,[1,a_theta_1,a_theta_2]);
% d_theta_22theta = tf(1,[1,a_theta_1,a_theta_2]);
% d_theta_12theta = tf(-1,[1,0]);
% q2theta         = tf(1,[1,0]);
% d_h2h           = tf(1,[1,0]);
theta2h         = tf(V_a,[1,0]);
delta_e2h       = minreal(delta_e2theta*theta2h);
V_a2h           = tf(theta,[1,0]);

core.param = param;
core.param.W_s = [0;0;0];
core.param.gust_condition = 'steady';
jacobian = numerical_jacobian(@(state) functions.eqs_motion(1000,state,param.u_0,core),x_0);
A_lat = jacobian([5,7,9,10,12],[5,7,9,10,12]);
A_lon = jacobian([3,4,6,8,11],[3,4,6,8,11]);
jacobian = numerical_jacobian(@(input) functions.eqs_motion(1000,x_0,input,core),param.u_0);
B_lat = jacobian([5,7,9,10,12],[1,3]);
B_lon = jacobian([3,4,6,8,11],[2,4]);

e_lat = eig(A_lat);
e_lon = eig(A_lon);

short_period.poly  = [1,-(e_lon(1)+e_lon(2)),(e_lon(1)*e_lon(2))];
short_period.w_n = sqrt(short_period.poly(3));
short_period.zeta = short_period.poly(2)/(2*short_period.w_n);

phugoid.poly  = [1,-(e_lon(4)+e_lon(5)),(e_lon(4)*e_lon(5))];
phugoid.w_n = sqrt(phugoid.poly(3));
phugoid.zeta = phugoid.poly(2)/(2*phugoid.w_n);

if Dispaly_Linear_Model
    figure(5),clf
    subplot(2,1,1)
    hold on
    title("Lateral Eigonvalues")
    plot(e_lat(1:2),[0,0],'+b')
    plot(e_lat(3:4),'ob')
    plot(e_lat(5),0,'*b')
    legend("Spiral","Dutch-Roll","Roll")
    xlabel("Real")
    ylabel("Imaginary")
    grid on
    hold off
    subplot(2,1,2)
    hold on
    title("Longitudinal Eigonvalues")
    plot(e_lon(1:2),'ob')
    plot(e_lon(4:5),'*b')
    legend("Short Perid","Phugoid")
    xlabel("Real")
    ylabel("Imaginary")
    grid on
    hold off
end

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
settings.loopshaping = false;
settings.simulate    = true;
settings.plot_names  = {%["p_{n} - Longitude (m)","p_{n}"];
                        %["p_{e} - Latitude (m)","p_{e}"];
                        ["h - Altitude (m)","y_{h}","r_{h}"];
                        ["\theta - Pitch (rad)","y_{\theta}","r_{\theta}"];%,"y_{\theta}_{dot}"];
                        ["\chi - Course (rad)","y_{\chi}","r_{\chi}"];
                        ["\phi - Roll (rad)","y_{\phi}","r_{\phi}"];%,"y_{\phi}_{dot}"];
                        ["\beta - Sideslip (rad)","y_{\beta}","r_{\beta}"];
                        %["\psi - Yaw (rad)","\psi"]
                        ["V_a - Forward Velocity (m/s)","y_{V_a}","r_{V_a}"];
                        ["\delta_{a} - Ailorons (rad)","delta_a"];
                        ["\delta_{e} - Elevator (rad)","delta_e"];
                        ["\delta_{r} - Ruder (rad)","delta_r"];
                        ["\delta_{t} - Throttle (rad)","delta_t"];
                        ["y_{r}_{dot} - Rate of Change","y_{h}_{dot}","y_{\theta}_{dot}","y_{\chi}_{dot}","y_{\phi}_{dot}","y_{\beta}_{dot}","y_{V_a}_{dot}"]
                        %["h_dot - Rate of Climb (m/s)","y_{h}_{dot}"]
%                         ["GPS_n - Position north (m)","GPS_n","p_{n}"];
%                         ["GPS_e - Position east (m)","GPS_e","p_{e}"];
%                         ["GPS_h - Altitude (m)","GPS_h","y_{h}"];
%                         ["GPS_Vg - measured horizontal velocity (m/s)","GPS_Vg","u"];
%                         ["GPS_chi - measured course (rad)","GPS_chi","y_{\chi}"];
%                         ["Bar - measured static pressure (altitude) (Pa)","Bar"];
%                         ["Pito - measured dynamic pressure (velocity) (Pa)","Pito"];
%                         ["Comp - measured heading (rad)","Comp","\psi"];
%                         ["Accel - measured acceleration (m/s^2)","Accel_x","Accel_y","Accel_z"];
%                         ["RateGyro - measured rotational rate (rad/s)","RateGyro_p","p","RateGyro_q","q","RateGyro_r","r"]
                        };
settings.active_fig  = 2;
settings.show_hist   = true;
                    
% General Variable to implement uncertainty
settings.implement_uncertainty = false;
                    
% Display warnings?
warning('off','all')

%% Publish Data
core.publish("measurements",zeros(size(param.sensor_names)));
core.publish("x",x_0)
core.publish("x_hat",x_0)
core.publish("u",param.u_0)
core.publish_list("r",r)
core.publish("y_r",y_r_0)
core.publish("y_r_dot",y_r_dot_0)
core.publish_list("t",t)
core.publish("V_a",x_0(4:6));
core.publish("V_a_dot",[0;0;0]);
core.settings = settings;
core.param = param;
core.functions = functions;

%% Controller Parameters

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

w_n_phi = sqrt(abs(a_phi_2)*param.phi_sat_lim.high/e_max_phi);
w_n_chi = 1/W_chi*w_n_phi;
w_n_beta = (a_beta_1+a_beta_2*param.delta_r_sat_lim.high/e_max_beta)/(2*zeta_beta);
w_n_theta = sqrt(a_theta_2+param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3);
w_n_h = 1/W_h*w_n_theta;
w_n_V = 2.2/t_r_V;
w_n_V_2 = 1/W_V_2*w_n_theta;

K_theta_DC = param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3/(a_theta_2+param.delta_e_sat_lim.high/e_max_theta*sign(a_theta_3)*a_theta_3);

control.controller_type = controllers.PID;

observe.type = observers.e; % O,e,p,m
% observe.L = gains_O(t_r./5,zeta,p_d,param.A,param.B,param.C_m); % 0.05;
observe.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi";"p";"q";"r"];
observe.u_names = ["delta_a";"delta_e";"delta_r";"delta_t"];
core.functions.observers(1) = observers(observe,core);

control.anti_windup = 'derivative'; % 'derivative', 'saturation', 'both', 'none'
control.windup_limit = 0;

if strcmp(control.controller_type,controllers.OL)
    control.windup_limit = 0;
    control.sat_lim.high = Inf;
    control.sat_lim.low = -Inf;
    control.K.I = 0;
    control.y_r_names = "h";
    control.u_names = "delta_a";
    control.plan = line;
    control.t_vec = t;
    core.functions.controllers(1) = controllers(control,core);
    
    control.windup_limit = 0;
    control.sat_lim.high = Inf;
    control.sat_lim.low = -Inf;
    control.K.I = 0;
    control.y_r_names = "h";
    control.u_names = "delta_r";
    control.plan = line;
    control.t_vec = t;
    core.functions.controllers(2) = controllers(control,core);
    
    control.windup_limit = 0;
    control.sat_lim.high = Inf;
    control.sat_lim.low = -Inf;
    control.K.I = 0;
    control.y_r_names = "h";
    control.u_names = "delta_e";
    control.plan = line;
    control.t_vec = t;
    core.functions.controllers(3) = controllers(control,core);
    
    control.windup_limit = 0;
    control.sat_lim.high = Inf;
    control.sat_lim.low = -Inf;
    control.K.I = 0;
    control.y_r_names = "h";
    control.u_names = "delta_t";
    control.plan = line;
    control.t_vec = t;
    core.functions.controllers(4) = controllers(control,core);
elseif strcmp(control.controller_type,controllers.PID)
    % Course hold
    control.windup_limit = 0.01;
    control.sat_lim.high = param.phi_sat_lim.high;
    control.sat_lim.low = param.phi_sat_lim.low;
    control.K.P = 2*zeta_chi*w_n_chi*V_g/g;
    control.K.I = w_n_chi^2*V_g/g;
    control.K.D = 0;
    control.y_r_names = "\chi";
    control.u_names = "\phi";
    core.functions.controllers(1) = controllers(control,core);
    core.functions.controllers(1).wrapping = true;
    control.K
    % Roll attitude hold
    control.sat_lim.high = param.delta_a_sat_lim.high;
    control.sat_lim.low = param.delta_a_sat_lim.low;
    control.K.P = control.sat_lim.high/e_max_phi;
    control.K.I = 0;
    control.K.D = (2*zeta_phi*w_n_phi-a_phi_1)/a_phi_2;
    control.y_r_names = "\phi";
    control.u_names = "delta_a";
    core.functions.controllers(1).cascade = controllers(control,core);
    control.K
    % Sideslip mitigation
    control.windup_limit = 0.0001;
    control.sat_lim.high = param.delta_r_sat_lim.high;
    control.sat_lim.low = param.delta_r_sat_lim.low;
    control.K.P = control.sat_lim.high/e_max_beta;
    control.K.I = w_n_beta^2/a_beta_2;
    control.K.D = 0;
    control.y_r_names = "\beta";
    control.u_names = "delta_r";
    core.functions.controllers(2) = controllers(control,core);
    control.K
    
    % Altitude hold
    control.windup_limit = 1;
    control.sat_lim.high = param.theta_sat_lim.high;
    control.sat_lim.low = param.theta_sat_lim.low;
    control.K.P = 2*zeta_h*w_n_h/(K_theta_DC*param.V_design);
    control.K.I = w_n_h^2/(K_theta_DC*param.V_design);
    control.K.D = 0;
    control.y_r_names = "h";
    control.u_names = "\theta";
    core.functions.controllers(3) = controllers(control,core);
    control.K
    % Ptch attitude hold
    control.sat_lim.high = param.delta_e_sat_lim.high;
    control.sat_lim.low = param.delta_e_sat_lim.low;
    control.K.P = control.sat_lim.high/e_max_theta*sign(a_theta_3);
    control.K.I = 0;
    control.K.D = (2*zeta_theta*w_n_theta-a_theta_1)/a_theta_3;
    control.y_r_names = "\theta";
    control.u_names = "delta_e";
    core.functions.controllers(3).cascade = controllers(control,core);
    control.K
    
    % Throttle airspeed hold
    control.windup_limit = 1;
    control.sat_lim.high = param.delta_t_sat_lim.high;
    control.sat_lim.low = param.delta_t_sat_lim.low;
    control.K.P = (2*zeta_V*w_n_V-a_V_1)/a_V_2;
    control.K.I = w_n_V^2/a_V_2;
    control.K.D = 0;
    control.y_r_names = "V_a";
    control.u_names = "delta_t";
    core.functions.controllers(4) = controllers(control,core);
    control.K
    % Pitch airspeed hold
    control.sat_lim.high = param.delta_t_sat_lim.high;
    control.sat_lim.low = param.delta_t_sat_lim.low;
    control.K.P = (a_V_1-2*zeta_V_2*w_n_V_2)/(K_theta_DC*param.g);
    control.K.I = w_n_V_2^2/(K_theta_DC*param.g);
    control.K.D = 0;
    control.y_r_names = "V_a";
    control.u_names = "delta_t";
    core.functions.controllers(5) = controllers(control,core);
    control.K
    
end

%% Sensors
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

sense.type = sensors.Accel;
sense.x_names = ["u","v","w","\phi","\theta","p","q","r"];
sense.sensor_names = ["Accel_x";"Accel_y";"Accel_z"];
core.functions.sensors(5) = sensors(sense,param);

% sense.type = sensors.Exact;
% sense.x_names = ["\phi","\theta","\psi"];
% sense.sensor_names = ["Accel_x";"Accel_y";"Accel_z"];
% core.functions.sensors(5) = sensors(sense,param);

sense.type = sensors.RateGyro;
sense.x_names = ["p","q","r"];
sense.sensor_names = ["RateGyro_p";"RateGyro_q";"RateGyro_r"];
core.functions.sensors(6) = sensors(sense,param);

% sense.type = sensors.Exact;
% sense.x_names = ["u","v","w"];
% sense.sensor_names = ["u";"v";"w"];
% core.functions.sensors(7) = sensors(sense,param);


%% Run Program
Simulation;

