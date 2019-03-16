%% Header
% Parameters

% add path
addpath('generalized/')

% Object for passing data
core = piping();

%% Pysical Parameters

% Pysical
param.g = 9.81;
param = aircraft("aerosnode",param);

% System Limits
param.chi_sat_lim.high = 15*pi/180;
param.chi_sat_lim.low = -15*pi/180;

param.phi_sat_lim.high = 45*pi/180;
param.phi_sat_lim.low = -45*pi/180;

param.delta_a_sat_lim.high = 45*pi/180;
param.delta_a_sat_lim.low = -45*pi/180;

param.delta_r_sat_lim.high = 45*pi/180;
param.delta_r_sat_lim.low = -45*pi/180;

param.h_sat_lim.high = 3;
param.h_sat_lim.low = -3;

param.theta_sat_lim.high = 45*pi/180;
param.theta_sat_lim.low = -45*pi/180;

param.delta_e_sat_lim.high = 45*pi/180;
param.delta_e_sat_lim.low = -45*pi/180;

param.delta_t_sat_lim.high = 1;
param.delta_t_sat_lim.low = 0;

param.take_off_alt = 10;
param.take_off_pitch = 15*pi/180;

% State Description
param.d_names = ["w_n";"w_e";"w_d";"w_n_dot";"w_e_dot";"w_d_dot"];
param.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi";"p";"q";"r"];
param.z_names = ["GPS_n";"GPS_e";"GPS_h";"GPS_Vg";"GPS_chi";"Bar";"Pito";"Comp";"Accel_x";"Accel_y";"Accel_z";"RateGyro_p";"RateGyro_q";"RateGyro_r"];
param.m_names = ["p_{n}";"p_{e}";"p_{d}";"u_a";"Accel_x";"Accel_y";"Accel_z";"\psi";"p";"q";"r";"\chi";"V_gh";"w_n";"w_e";"w_d"];
param.r_names = ["\chi";"\phi";"h";"\theta";"\beta";"V_a"];
param.u_names = ["delta_a";"delta_e";"delta_r";"delta_t"];

param.x_is_angle = [false;false;false;false;false;false;true;true;true;false;false;false];
param.m_is_angle = [false;false;false;false;false;false;false;true;false;false;false;true;false;false;false;false];
param.r_is_angle = [true;true;false;true;true;false];

%% Uncertainty

% Parameter
param.uncertian_param = {};
param.D_in_param.random  = 0.2.*[];
param.D_in_param.bias    = 0.0.*[];

% Wind
base = [5;0;0];
gust = wind.steady;
param.wind = wind(gust,base,param.V_design);

% Magnetic
param.declination = 2.9*pi/180;

%% Simulation Dimensions

% Aircraft Dimensions
scale = 100;

% Wing
param.wing_w = 1.5*scale;
param.wing_l = 0.2*scale;
    
% Fusalage
param.fuse_w = 0.15*scale;
param.fuse_l1 = 0.2*scale;
param.fuse_l2 = 0.1*scale;
param.fuse_l3 = 0.8*scale;
param.fuse_h = 0.15*scale;
    
% Horizontal Stabalizer
param.tailwing_w = 0.2*scale;
param.tailwing_l = 0.12*scale;
    
% Vertical Stabalizer
param.tail_h = 0.1*scale;

% Simulation
settings.start       = 0;      % s
settings.step        = 0.02;   % s
settings.end         = 30;     % s
t = settings.start:settings.step:settings.end;

settings.playback_rate  = 1;
settings.window         = [-1000,1000,-1000,1000,0,200]; % m
settings.view           = [45,45];
settings.gph_per_img    = 4;
settings.labels         = ["p_{e} - Latitude (m)","p_{n} - Longitude (m)","h - Altitude (m)"];

%% Functions

functions.get_drawing = @get_drawing;
functions.get_equilibrium = @get_equilibrium;
functions.eqs_motion = @eqs_motion;
functions.controller_architecture = @controller_architecture; 
functions.get_y_r = @get_y_r;
functions.get_y_m = @get_y_m;
functions.get_tf_coefficents = @get_tf_coefficents;

% Controller
function [u,r] = controller_architecture(controllers,y_r,y_r_dot,r,d_hat,t,param)
    % Saturate
    r_sat = r;
    chi_error = controllers.get_error(y_r(1),r_sat(1),true);
    if chi_error > param.chi_sat_lim.high
        r_sat(1) = y_r(1)+param.chi_sat_lim.high;
    elseif chi_error < param.chi_sat_lim.low
        r_sat(1) = y_r(1)+param.chi_sat_lim.low;
    end
    

    % Lateral
    % Saturate
    [u(1,1),r_sat] = controllers(1).control(y_r,y_r_dot,r_sat,d_hat,t);
    r(2) = r_sat(2);
    [u(3,1),r] = controllers(2).control(y_r,y_r_dot,r,d_hat,t);
    
    % Longitudinal
%     if y_r(3) < param.take_off_alt
%         r(4) = param.take_off_pitch;
%         u(2,1) = controllers(3).cascade.control(y_r,y_r_dot,r,d_hat,t);
%         u(4,1) = 1;
%     else
        % Saturate
        r_sat = r;
        r_sat(3) = min(param.h_sat_lim.high+y_r(3),r_sat(3));
        r_sat(3) = max(param.h_sat_lim.low+y_r(3),r_sat(3));
        
        % Controller
        [u(2,1),r_sat] = controllers(3).control(y_r,y_r_dot,r_sat,d_hat,t);
        r(4) = r_sat(4);
        [u(4,1),r] = controllers(4).control(y_r,y_r_dot,r,d_hat,t);
%     end
    
%     u(1,1) = param.u_0(1);
%     u(2,1) = param.u_0(2);
%     u(3,1) = param.u_0(3);
%     u(4,1) = param.u_0(4);
end

% Dynamic Equilibrium Input
function [u,x,y_r] = get_equilibrium(~,param,functions)
    if isfield(param,'u_0')
        u = param.u_0;
        x = param.x_0;
        y_r = param.y_r_0;
    else
        % Unpack
        V_a = param.trim.V_a;
        R = param.trim.R;
        gamma = param.trim.gamma;
        desired = get_x_dot_star(V_a,R,gamma);
        
        % Initial conditions
        initial = [0;0;0];
        
        % Wind
        param.wind = wind(wind.steady,[0;0;0],V_a);
        
        
        % function to trim
        fun = @(input) norm(functions.eqs_motion(0,get_x_star(input,V_a,R,gamma),get_u_star(input,V_a,R,gamma,param),param)-desired)^2;
        
        switch param.optimizer.name
            case "fminsearch"
                output = fminsearch(fun,initial);
            case "gradiant"
                output = gd_optimizer(fun,initial);
            otherwise
                error("Unrecognized optimizer")
        end

        x = get_x_star(output,V_a,R,gamma);
        x(3) = -param.trim.h_0;

        u = get_u_star(output,V_a,R,gamma,param);
        
        y_r = [0;0;param.trim.h_0;x(8);0;V_a];
    end
end

% Equations of Motion
function [x_dot,d] = eqs_motion(t,x,input,param)
    
    % Unpack --------------------------------------------------------------
    
    % State
    p_d     = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    % Input
    delta_a = input(1);
    delta_e = input(2);
    delta_r = input(3);
    delta_t = input(4);
    % Aircraft parameters
    % my_unpack(param.aircraft)
    
    %physical parameters of airframe
    mass = param.mass;
    Jy   = param.Jy;
    M    = param.M;% Dimensions
    c    = param.c;
    b    = param.b;
    S_wing    = param.S_wing;
    AR   = b^2/S_wing;
    alpha_0     = param.alpha_0; % Aerodynamics
    C_L_0       = param.C_L_0;
    C_L_alpha   = param.C_L_alpha;
    C_L_q       = param.C_L_q;
    C_L_delta_e = param.C_L_delta_e;
    C_D_p       = param.C_D_p;
    C_D_q       = param.C_D_q;
    C_D_delta_e = param.C_D_delta_e;
    C_Y_0       = param.C_Y_0;
    C_Y_beta    = param.C_Y_beta;
    C_Y_p       = param.C_Y_p;
    C_Y_r       = param.C_Y_r;
    C_Y_delta_a = param.C_Y_delta_a;
    C_Y_delta_r = param.C_Y_delta_r;
    e           = param.e;
    C_l_0       = param.C_l_0; % Stability
    C_l_beta    = param.C_l_beta;
    C_l_p       = param.C_l_p;
    C_l_r       = param.C_l_r;
    C_l_delta_a = param.C_l_delta_a;
    C_l_delta_r = param.C_l_delta_r;
    C_m_0       = param.C_m_0;
    C_m_alpha   = param.C_m_alpha;
    C_m_q       = param.C_m_q;
    C_m_delta_e = param.C_m_delta_e;
    C_n_0       = param.C_n_0;
    C_n_beta    = param.C_n_beta;
    C_n_p       = param.C_n_p;
    C_n_r       = param.C_n_r;
    C_n_delta_a = param.C_n_delta_a;
    C_n_delta_r = param.C_n_delta_r;
    S_prop      = param.S_prop; % Motor
    C_prop      = param.C_prop;
    k_motor     = param.k_motor;
    k_T_P       = param.k_T_P;
    k_Omega     = param.k_Omega;     
    
    Gamma = param.Gamma;
    
    % Trig
    S_phi = sin(phi);
    C_phi = cos(phi);
    S_theta = sin(theta);
    C_theta = cos(theta);
    T_theta = tan(theta);
    S_psi = sin(psi);
    C_psi = cos(psi);
    % Rotations
    R_bv2 = [1,0,0;
             0,C_phi,-S_phi;
             0,S_phi,C_phi];
    R_v2v1 = [C_theta,0,S_theta;
              0,1,0;
              -S_theta,0,C_theta];
    R_v1v = [C_psi,-S_psi,0;
             S_psi,C_psi,0;
             0,0,1];
    R_bv = R_v1v*R_v2v1*R_bv2;
    R_vb = R_bv.';
    R_bv12 = [1,S_phi*T_theta,C_phi*T_theta;
              0,C_phi,-S_phi;
              0,S_phi/C_theta,C_phi/C_theta];
    
    % Disturbance ---------------------------------------------------------
    
    % Air
    [base,gust,gust_dot] = param.wind.get(t);
    V_w_b = gust + R_vb*base;
    d = [R_bv*V_w_b;R_bv*gust_dot];
    air  = atmosphere(-p_d);
    rho = air.rho;
    % Wind Triangle
    V_g_b = [u;v;w];
    V_a_b = V_g_b - V_w_b;
    u_a = V_a_b(1);
    v_a = V_a_b(2);
    w_a = V_a_b(3);
    alpha = atan2(w_a,u_a);
    V_a = sqrt(u_a^2+v_a^2+w_a^2);
    beta = atan2(v_a,sqrt(u_a^2+w_a^2));

    % Aerodynamics --------------------------------------------------------
    
    % Drag and Lift Coeficents
    sigma = (1+exp(-M*(alpha-alpha_0))+exp(M*(alpha+alpha_0)))/((1+exp(-M*(alpha-alpha_0)))*(1+exp(M*(alpha+alpha_0))));
    C_D = C_D_p+(C_L_0 + C_L_alpha*alpha)^2/(pi*e*AR);
    C_L = (1-sigma)*(C_L_0+C_L_alpha*alpha)+2*sigma*sign(alpha)*sin(alpha)^2*cos(alpha);
    
    % Force Coeficents
    C_X = -C_D*cos(alpha)+C_L*sin(alpha);
    C_X_q = -C_D_q*cos(alpha)+C_L_q*sin(alpha);
    C_X_delta_e = -C_D_delta_e*cos(alpha)+C_L_delta_e*sin(alpha);
    C_Z = -C_D*sin(alpha)-C_L*cos(alpha);
    C_Z_q = -C_D_q*sin(alpha)-C_L_q*cos(alpha);
    C_Z_delta_e = -C_D_delta_e*sin(alpha)-C_L_delta_e*cos(alpha);
    
    % Equation Matrices
    Gravity = [-mass*param.g*S_theta;
               mass*param.g*C_theta*S_phi;
               mass*param.g*C_theta*cos(phi)];
           
    Aerodynamics = 1/2*rho*V_a*S_wing*[C_X*V_a+C_X_q*c/2*q;
                                    C_Y_0*V_a+C_Y_beta*beta*V_a+C_Y_p*b/2*p+C_Y_r*b/2*r;
                                    C_Z*V_a+C_Z_q*c/2*q];
                                
    Control_Force = 1/2*rho*V_a^2*S_wing*[C_X_delta_e*delta_e;
                                     C_Y_delta_a*delta_a+C_Y_delta_r*delta_r;
                                     C_Z_delta_e*delta_e]; 
                                 
    Motor_Force = 1/2*rho*S_prop*C_prop*((k_motor*delta_t)^2 - V_a^2)*[1;0;0];
    
    
    Stability = 1/2*rho*V_a*S_wing*[b*(C_l_0*V_a+C_l_beta*beta*V_a+C_l_p*b/2*p+C_l_r*b/2*r);
                                    c*(C_m_0*V_a+C_m_alpha*alpha*V_a+C_m_q*c/2*q);
                                    b*(C_n_0*V_a+C_n_beta*beta*V_a+C_n_p*b/2*p+C_n_r*b/2*r)];
                                      
    Control_Torque = 1/2*rho*V_a^2*S_wing*[b*(C_l_delta_a*delta_a+C_l_delta_r*delta_r);
                                           c*(C_m_delta_e*delta_e);
                                           b*(C_n_delta_a*delta_a+C_n_delta_r*delta_r)];
                                           
    Motor_Torque = -k_T_P*(k_Omega*delta_t)^2*[1;0;0];
                                      
    % Rigid Body Motion
    % Forces
    F = Gravity + Aerodynamics + Control_Force + Motor_Force;
    % Moments
    Tou = Stability + Control_Torque + Motor_Torque;

    % Rigid Body Responce -------------------------------------------------
    
    % Unpack
    fx = F(1);
    fy = F(2);
    fz = F(3);
    l  = Tou(1);
    m  = Tou(2);
    n  = Tou(3);
    
    % Equation Matrices
    Acceleration            = [r*v-q*w;
                               p*w-r*u;
                               q*u-p*v];
    Angular_Acceleration    = [Gamma(1)*p*q-Gamma(2)*q*r,Gamma(3)*l+Gamma(4)*n;
                               Gamma(5)*p*r-Gamma(6)*(p^2-r^2),1/Jy*m;
                               Gamma(7)*p*q-Gamma(1)*q*r,Gamma(4)*l+Gamma(8)*n];
                               
    % Initialize
    x_dot = zeros(12,1);

    % Equations of Motion
    x_dot(1:3)   = R_bv*x(4:6);
    x_dot(4:6)   = Acceleration+1/mass*[fx;fy;fz];
    x_dot(7:9)   = R_bv12*x(10:12);
    x_dot(10:12) = sum(Angular_Acceleration,2);
end

% Anamation Information
function [points,colors,history] = get_drawing(x,~,param)
    persistent previous_positions
    
    if isempty('previous_positions')
        previous_positions = [];
    end
    
    previous_positions = [previous_positions,x(1:3)];
    
    % Wing
    wing_w = param.wing_w;
    wing_l = param.wing_l;
    
    % Fusalage
    fuse_w  = param.fuse_w;
    fuse_l1 = param.fuse_l1;
    fuse_l2 = param.fuse_l2;
    fuse_l3 = param.fuse_l3;
    fuse_h  = param.fuse_h;
    
    % Horizontal Stabalizer
    tailwing_w = param.tailwing_w;
    tailwing_l = param.tailwing_l;
    
    % Virtical Stabalizer
    tail_h = param.tail_h;
    

    % Aircraft Parts
    wing.right = [0,wing_w/2,0;
                 -wing_l,wing_w/2,0;
                 -wing_l,0,0;
                 0,0,0];
    wing.left = [0,-wing_w/2,0;
                  -wing_l,-wing_w/2,0;
                  -wing_l,0,0;
                  0,0,0];
    fusalage.nose.top = [fuse_l2,fuse_w/2,-fuse_h/2;
                         fuse_l2,-fuse_w/2,-fuse_h/2;
                         fuse_l1,0,0];
    fusalage.nose.left = [fuse_l2,-fuse_w/2,-fuse_h/2;
                          fuse_l2,-fuse_w/2,fuse_h/2;
                          fuse_l1,0,0];
    fusalage.nose.right = [fuse_l2,fuse_w/2,-fuse_h/2;
                           fuse_l2,fuse_w/2,fuse_h/2;
                           fuse_l1,0,0];
    fusalage.nose.bottom = [fuse_l2,fuse_w/2,fuse_h/2;
                            fuse_l2,-fuse_w/2,fuse_h/2;
                            fuse_l1,0,0];
    fusalage.empanage.top = [fuse_l2,fuse_w/2,-fuse_h/2;
                            fuse_l2,-fuse_w/2,-fuse_h/2;
                            -fuse_l3,0,0];
    fusalage.empanage.left = [fuse_l2,-fuse_w/2,-fuse_h/2;
                              fuse_l2,-fuse_w/2,fuse_h/2;
                              -fuse_l3,0,0];
    fusalage.empanage.right = [fuse_l2,fuse_w/2,-fuse_h/2;
                               fuse_l2,fuse_w/2,fuse_h/2;
                               -fuse_l3,0,0];
    fusalage.empanage.bottom = [fuse_l2,fuse_w/2,fuse_h/2;
                                fuse_l2,-fuse_w/2,fuse_h/2;
                                -fuse_l3,0,0];
    horz_stab = [-fuse_l3,tailwing_w/2,0;
                 -fuse_l3+tailwing_l,tailwing_w/2,0;
                 -fuse_l3+tailwing_l,-tailwing_w/2,0;
                 -fuse_l3,-tailwing_w/2,0];
    vir_stab = [-fuse_l3,0,0;
                -fuse_l3,0,-tail_h;
                -fuse_l3+tailwing_l,0,0;];

    points = {wing.right,wing.left,fusalage.nose.top,fusalage.nose.left,fusalage.nose.right,fusalage.nose.bottom,fusalage.empanage.top,fusalage.empanage.left,fusalage.empanage.right,fusalage.empanage.bottom,horz_stab,vir_stab};
    colors = {'g','r','y','k','k','k','k','k','k','k','k','k'};
    

    R_bv = get_rotation(x(7),x(8),x(9),'b->v');
    R_NED_ENU = get_rotation(x(7),x(8),x(9),'NED->ENU');
    
    for i = 1:length(points)
        points{i} = points{i}.';
        
        points{i} = R_bv*points{i};
        points{i} = x(1:3)+points{i};
        
        points{i} = R_NED_ENU*points{i};
    end
    
    history{1} = R_NED_ENU*previous_positions;
end

% y_r Conversion
function [y_r_out,y_r_dot_out] = get_y_r(z,x,d,param)

    p_d = x(3);
    u = x(4);
    v = x(5);
    w = x(6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    p = x(10);
    q = x(11);
    r = x(12);
    
    g = param.g;
    V_w_g = d(1:3);
    V_w_g_dot = d(4:6);
    V_g_b_dot = z(9:11)-[ g*sin(theta);
                         -g*cos(theta)*sin(phi);
                         -g*cos(theta)*cos(phi)];

    R_bv = get_rotation(phi,theta,psi,'b->v');
    R_vb = get_rotation(phi,theta,psi,'v->b');
    V_g_g = R_bv*[u;v;w];
    p_d_dot = V_g_g(3);
    V_w_b = R_vb*V_w_g;
    V_w_b_dot = R_vb*V_w_g_dot;
    
    V_g_b = [u;v;w];
    V_a_b = V_g_b - V_w_b;
    V_a_b_dot = V_g_b_dot-V_w_b_dot;
    
    u_a = V_a_b(1);
    v_a = V_a_b(2);
    w_a = V_a_b(3);
    u_a_dot = V_a_b_dot(1);
    v_a_dot = V_a_b_dot(2);
    w_a_dot = V_a_b_dot(3);
    
    V_a = sqrt(sum(V_a_b.^2));
    V_a_dot = sqrt(sum(V_a_b_dot.^2));
    beta = atan2(v_a,sqrt(u_a^2+w_a^2));
    chi = atan2(V_g_g(2),V_g_g(1));
    beta_dot = 1/(v_a^2/(u_a^2+w_a^2)+1)*(v_a_dot/sqrt(u_a^2+w_a^2)-v_a*(u_a*u_a_dot+w_a*w_a_dot)/(u_a^2+w_a^2)^(3/2));
    
    rotational_velocity = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
                           0,cos(phi),-sin(phi);
                           0,sin(phi)/cos(theta),cos(phi)/cos(theta)]*[p;q;r];
    phi_dot = rotational_velocity(1);
    theta_dot = rotational_velocity(2);
    chi_dot = rotational_velocity(3);
    
    y_r_out(1,1) = chi;
    y_r_out(2,1) = phi;
    y_r_out(3,1) = -p_d;
    y_r_out(4,1) = theta;
    y_r_out(5,1) = beta;
    y_r_out(6,1) = V_a;
    
    y_r_dot_out(1,1) = chi_dot;
    y_r_dot_out(2,1) = phi_dot;
    y_r_dot_out(3,1) = -p_d_dot;
    y_r_dot_out(4,1) = theta_dot;
    y_r_dot_out(5,1) = beta_dot;
    y_r_dot_out(6,1) = V_a_dot;
end

% y_m Conversion
function y_m = get_y_m(z,param)
    p_n=z(1);
    p_e=z(2);
    V_gh=z(4);
    chi=z(5);
    P_static=z(6);
    P_dynamic=z(7);
    psi=z(8);
    a_x=z(9);
    a_y=z(10);
    a_z=z(11);
    p=z(12);
    q=z(13);
    r=z(14);
    
    g = param.g;
    
    air = atmosphere(0);
    
    h_Bar = P_static/(g*air.rho);
    p_d = h_Bar;
    
    u_a = sqrt(2/air.rho*P_dynamic);
    
%     phi = atan2(-a_y,-a_z);
%     try
%         theta = atan2(a_x,sqrt(g^2-a_x^2));
%     catch
%         theta = asin(a_x/g);
%         warning("Acceleration in the x direction is %f",a_x)
%     end
    
    w_n = V_gh*cos(chi)-u_a*cos(psi);
    w_e = V_gh*sin(chi)-u_a*sin(psi);
    w_d = 0;

    y_m(1,1) = p_n;
    y_m(2,1) = p_e;
    y_m(3,1) = p_d;
    y_m(4,1) = u_a;
    y_m(5,1) = a_x;
    y_m(6,1) = a_y;
    y_m(7,1) = a_z;
    y_m(8,1) = psi;
    y_m(9,1) = p;
    y_m(10,1) = q;
    y_m(11,1) = r;
    y_m(12,1) = chi;
    y_m(13,1) = V_gh;
    y_m(14,1) = w_n; 
    y_m(15,1) = w_e;
    y_m(16,1) = w_d;
end

% Other Functions
function output = get_x_star(input,V_a,R,gamma)
    alpha = input(1);
    beta = input(2);
    phi = input(3);
    output = [0;
              0;
              0;
              V_a*cos(alpha)*cos(beta);
              V_a*sin(beta);
              V_a*sin(alpha)*cos(beta);
              phi;
              alpha + gamma;
              0;
              -V_a/R*sin(alpha + gamma);
              V_a/R*sin(phi)*cos(alpha + gamma);
              V_a/R*cos(phi)*cos(alpha + gamma)];
end

function output = get_x_dot_star(V_a,R,gamma)
    output = [V_a*cos(gamma);0;-V_a*sin(gamma);0;0;0;0;0;V_a/R;0;0;0];
end

function output = get_u_star(input,V_a,R,gamma,param)
    
    % Unpack
    x = get_x_star(input,V_a,R,gamma);
    v       = x(5);
    w       = x(6);
    theta   = x(8);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    air  = atmosphere(0);
    rho = air.rho;
    my_unpack(param.aircraft)
    alpha = input(1);
    beta = input(2);
    
    % Lift and Drag
    sigma = (1+exp(-M*(alpha-alpha_0))+exp(M*(alpha+alpha_0)))/((1+exp(-M*(alpha-alpha_0)))*(1+exp(M*(alpha+alpha_0))));
    C_D = C_D_p+(C_L_0 + C_L_alpha*alpha)^2/(pi*e*AR);
    C_L = (1-sigma)*(C_L_0+C_L_alpha*alpha)+2*sigma*sign(alpha)*sin(alpha)^2*cos(alpha);
    
    % Lift and Drag to Force
    C_X = -C_D*cos(alpha)+C_L*sin(alpha);
    C_X_q = -C_D_q*cos(alpha)+C_L_q*sin(alpha);
    C_X_delta_e = -C_D_delta_e*cos(alpha)+C_L_delta_e*sin(alpha);

    % Control Inputs nessisary to Cause these Forces
    delta_e = ((Jxz*(p^2-r^2)+(Jx-Jz)*p*r)/(1/2*rho*V_a^2*c*S_wing) - C_m_0 - C_m_alpha*alpha - C_m_q*c*q/(2*V_a))/C_m_delta_e;
    delta_t = sqrt((2*mass*(-r*v+q*w+param.g*sin(theta)) - rho*V_a^2*S_wing*(C_X+C_X_q*c*q/(2*V_a)+C_X_delta_e*delta_e))/(rho*S_prop*C_prop*k_motor^2)+(V_a/k_motor)^2);
    delta_long = [C_p_delta_a,C_p_delta_r;C_r_delta_a,C_r_delta_r]^-1*[(-Gamma(1)*p*q+Gamma(2)*q*r)/(1/2*rho*V_a^2*S_wing*b)-C_p_0-C_p_beta*beta-C_p_p*b*p/(2*V_a)-C_p_r*b*r/(2*V_a);
                                                                       (-Gamma(7)*p*q+Gamma(1)*q*r)/(1/2*rho*V_a^2*S_wing*b)-C_r_0-C_r_beta*beta-C_r_p*b*p/(2*V_a)-C_r_r*b*r/(2*V_a)];
    % Pack
    output = [delta_long(1);delta_e;delta_long(2);delta_t];
end

function [a_phi_1,a_phi_2,a_beta_1,a_beta_2,a_theta_1,a_theta_2,a_theta_3,a_V_1,a_V_2,a_V_3] = get_tf_coefficents(param)
    my_unpack(param.aircraft)
    delta_e = param.u_0(2);
    delta_t = param.u_0(4);
    alpha = atan2(param.x_0(6),param.x_0(4));
    V_a = param.trim.V_a;
    a_phi_1     = 1/4*rho*V_a*S_wing*b^2*C_p_p;
    a_phi_2     = 1/2*rho*V_a^2*S_wing*b*C_p_delta_a;
    a_beta_1    = -rho*V_a*S_wing/(2*mass)*C_Y_beta; 
    a_beta_2    = rho*V_a*S_wing/(2*mass)*C_Y_delta_r;
    a_theta_1   = -rho*V_a*c^2*S_wing/(4*Jy)*C_m_q;
    a_theta_2   = -rho*V_a^2*c*S_wing/(2*Jy)*C_m_alpha;
    a_theta_3   = rho*V_a^2*c*S_wing/(2*Jy)*C_m_delta_e;
    a_V_1       = rho*V_a*S_wing/mass*(C_D_0+C_D_alpha*alpha+C_D_delta_e*delta_e) + rho*S_prop/mass*C_prop*V_a;
    a_V_2       = rho*S_prop/mass*C_prop*k_motor^2*delta_t;
    a_V_3       = param.g;
end



