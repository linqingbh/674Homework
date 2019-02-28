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
param.chi_sat_lim.high = 45*pi/180;
param.chi_sat_lim.low = -45*pi/180;

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
param.x_names = ["p_{n}";"p_{e}";"p_{d}";"u";"v";"w";"\phi";"\theta";"\psi";"p";"q";"r"];
param.sensor_names = ["GPS_n";"GPS_e";"GPS_h";"GPS_Vg";"GPS_chi";"Bar";"Pito";"Comp";"Accel_x";"Accel_y";"Accel_z";"RateGyro_p";"RateGyro_q";"RateGyro_r"];
param.m_names = ["p_{n}";"p_{e}";"p_{d}";"u_a";"\phi";"\theta";"\psi";"p";"q";"r";"\chi";"V_gh";"u_w";"v_w";"w_w"];
param.u_names = ["delta_a";"delta_e";"delta_r";"delta_t"];
param.r_names = ["\chi","\phi","h","\theta","\beta","V_a"];

param.x_e = [0;0;0;0;0;0;0;0;0;0;0;0];

% State Space Equations

%% Uncertainty
% Here you are seting the standard deviation of the uncertianty
% Values next to each setting are useful for understanding the scale
% The values for random are one standard deviation of random error.
% The falues for bias are one standard deviations offset.
% k,m,b

param.uncertian_param = {'mass'};
param.D_in_param.random  = 0.2.*[param.mass];
param.D_in_param.bias    = 0.0.*[param.mass];
% F
param.uncertian_u = false;
param.D_in_u.random      = 0.0;
param.D_in_u.bias        = 0.5; 
% z,z_dot
param.uncertian_x = [false,false,false,false];
param.D_out.random       = [0,0,0,0];
param.D_out.bias         = [0,0,0,0];
% z,z_dot - measured
param.uncertain_N = [false,false,false,false];
param.N.random           = [0.001,0.001,0,0];
param.N.bias             = [0,0,0,0];

% Wind
param.W_s = [0;1;-1];
param.gust_condition = 'moderate';
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
settings.end         = 100;     % s
t = settings.start:settings.step:settings.end;

settings.publish     = 0.2;    % s
settings.window      = [-1000,1000,-1000,1000,0,200]; % m
settings.view        = [45,45];
settings.gph_per_img = 3;
settings.labels      = ["p_{e} - Latitude (m)","p_{n} - Longitude (m)","-p_{d} - Altitude (m)"];

%% Functions

functions.get_drawing = @get_drawing;
functions.u_e = @u_e;
functions.eqs_motion = @eqs_motion;
functions.controller_architecture = @controller_architecture; 
functions.y_r = @y_r;

% Controller
function u = controller_architecture(controllers,x,r,d_hat,param)
    % Lateral
    % Saturate
    current = y_r(x,param);
    chi = current(1);
%     r(1) = min(param.chi_sat_lim.high+chi,r(1));
%     r(1) = max(param.chi_sat_lim.low+chi,r(1));
    
    
    
    u(1,1) = controllers(1).control(x,r(1),d_hat);
    u(3,1) = controllers(2).control(x,r(5),d_hat);
    
    % Longitudinal
    if -x(3) < param.take_off_alt
        u(2,1) = controllers(3).cascade.control(x,param.take_off_pitch,d_hat);
        u(4,1) = 1;
    else
        % Saturate
        r(3) = min(param.h_sat_lim.high-x(3),r(3));
        r(3) = max(param.h_sat_lim.low-x(3),r(3));
        
        % Controller
        u(2,1) = controllers(3).control(x,r(3),d_hat);
        u(4,1) = controllers(4).control(x,r(6),d_hat);
    end
    
    %u(1,1) = param.u_0(1);
    %u(2,1) = param.u_0(2);
    %u(3,1) = param.u_0(3);
    %u(4,1) = param.u_0(4);
end

% Dynamic Equilibrium Input
function [u,x] = u_e(x,param)
    if isfield(param,'u_0')
        u = param.u_0;
        x = param.x_0;
    else
        % Unpack
        V_a = param.trim.V_a;
        R = param.trim.R;
        gamma = param.trim.gamma;
        
        % Initial conditions
        alpha = 0;
        beta = 0;
        phi = 0;
        
        switch param.optimizer.name
            case "fminsearch"
                output = fminsearch(@(variables) J(variables,V_a,R,gamma,param),[alpha,beta,phi]);
                alpha = output(1);
                beta = output(2);
                phi = output(3);
            case "gradiant"
                [alpha,beta,phi] = min_J(alpha,beta,phi,V_a,R,gamma,param); 
            otherwise
                error("Unrecognized optimizer")
        end

        x = x_star(alpha,beta,phi,V_a,R,gamma);

        u = u_star(alpha,beta,V_a,x,param);
    end
end

% Equations of Motion
function x_dot = eqs_motion(dt,x,input,core)
    
    param = core.param;
    
    persistent x_k_u x_k_v x_k_w
    if isempty(x_k_u)
        x_k_u = 0;
        x_k_v = [0;0];
        x_k_w = [0;0];
    end

    %------------------------------------------------
    % Aerodynamics
    
    % Unpack
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
    % Trig
    S_phi = sin(phi);
    C_phi = cos(phi);
    S_theta = sin(theta);
    C_theta = cos(theta);
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
    % Air
    % Static wind
    W_s = param.W_s;
    
    % Gusts wigd
    if -p_d <= 50
        switch param.gust_condition
            case "light"
                sigma_u = 1.06;
                sigma_v = sigma_u;
                sigma_w = 0.7;
                L_u = 200;
                L_v = L_u;
                L_w = 50;
                white_noise = randn;
            case "moderate"
                sigma_u = 2.12;
                sigma_v = sigma_u;
                sigma_w = 1.4;
                L_u = 200;
                L_v = L_u;
                L_w = 50;
                white_noise = randn;
            otherwise % Steady
                sigma_u = 0;
                sigma_v = sigma_u;
                sigma_w = 0;
                L_u = 1;
                L_v = L_u;
                L_w = 1;
                white_noise = 0;
        end
    else
        switch param.gust_condition
            case "light"
                sigma_u = 1.5;
                sigma_v = sigma_u;
                sigma_w = 1.5;
                L_u = 533;
                L_v = L_u;
                L_w = 533;
                white_noise = randn;
            case "moderate"
                sigma_u = 3.0;
                sigma_v = sigma_u;
                sigma_w = 3.0;
                L_u = 533;
                L_v = L_u;
                L_w = 533;
                white_noise = randn;
            otherwise % Steady
                sigma_u = 0;
                sigma_v = sigma_u;
                sigma_w = 0;
                L_u = 1;
                L_v = L_u;
                L_w = 1;
                white_noise = 0;
        end
    end
    
    function [y,x_k2] = get_next_wind(num,den,x_k,white_noise,dt)

       I = eye(length(x_k));

       [A,B,C,D] = tf2ss(num,den);
       
       x_k2 = (I+dt*A)*x_k + dt*B*white_noise;
       y = C*x_k + D*white_noise;
    end
    V_design    = param.V_design;
    [u_w_g,x_k_u] = get_next_wind(sigma_u*sqrt(2*V_design/L_u),[1,V_design/L_u],x_k_u,white_noise,dt);
    [v_w_g,x_k_v] = get_next_wind(sigma_v*sqrt(3*V_design/L_v)*[1,V_design/(sqrt(3)*L_v)],conv([1,V_design/L_v],[1,V_design/L_v]),x_k_v,white_noise,dt);
    [w_w_g,x_k_w] = get_next_wind(sigma_w*sqrt(3*V_design/L_w)*[1,V_design/(sqrt(3)*L_w)],conv([1,V_design/L_w],[1,V_design/L_w]),x_k_w,white_noise,dt);

    W_g = [u_w_g;v_w_g;w_w_g];
    
    V_w_b = R_vb*W_s+W_g; % Wind in the inertial NED frame 
    
    air  = atmosphere(-p_d);
    rho = air.rho;
    
    % Input
    delta_a = input(1);
    delta_e = input(2);
    delta_r = input(3);
    delta_t = input(4);
    % Aircraft parameters
    g    = param.g; % Rigid Body
    mass = param.mass;
    Jx   = param.Jx;
    Jy   = param.Jy;
    Jz   = param.Jz;
    Jxz  = param.Jxz;
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
    % Shorthand
    Gamma(1) = Jxz*(Jx-Jy+Jz)/(Jx*Jz-Jxz^2);
    Gamma(2) = (Jz*(Jz-Jy)+Jxz^2)/(Jx*Jz-Jxz^2);
    Gamma(3) = Jz/(Jx*Jz-Jxz^2);
    Gamma(4) = Jxz/(Jx*Jz-Jxz^2);
    Gamma(5) = (Jz-Jx)/Jy;
    Gamma(6) = Jxz/Jy;
    Gamma(7) = ((Jx-Jy)*Jx+Jxz^2)/(Jx*Jz-Jxz^2);
    Gamma(8) = Jx/(Jx*Jz-Jxz^2);
    
    % Wind Triangle
    V_g_b = [u;v;w];
    V_a_b = V_g_b - V_w_b;
    u_r = V_a_b(1);
    v_r = V_a_b(2);
    w_r = V_a_b(3);
    alpha = atan2(w_r,u_r);
    V_a = sqrt(u_r^2+v_r^2+w_r^2);
    beta = atan2(v_r,sqrt(u_r^2+w_r^2));
    if ismethod(core,"publish")
        core.publish("V_a",V_a_b)
    end

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
    Gravity = [-mass*g*sin(theta);
               mass*g*cos(theta)*sin(phi);
               mass*g*cos(theta)*cos(phi)];
           
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
                                      
    % Rigid Body
    % Forces
    F = Gravity + Aerodynamics + Control_Force + Motor_Force;
    % Moments
    Tou = Stability + Control_Torque + Motor_Torque;

    % -----------------------------------------
    % Rigid Body Responce

    % Unpack
    fx = F(1);
    fy = F(2);
    fz = F(3);
    l  = Tou(1);
    m  = Tou(2);
    n  = Tou(3);
    
    % State
    % Equation Matrices
    Velocity                = [C_theta*C_psi,S_phi*S_theta*C_psi-C_phi*S_psi,C_phi*S_theta*C_psi+S_phi*S_psi;
                               C_theta*S_psi,S_phi*S_theta*S_psi+C_phi*C_psi,C_phi*S_theta*S_psi-S_phi*C_psi;
                               -S_theta,S_phi*C_theta,C_phi*C_theta];
    Acceleration            = [r*v-q*w;
                               p*w-r*u;
                               q*u-p*v];
    Angular_Velocity        = [1,S_phi*tan(theta),C_phi*tan(theta);
                               0,C_phi,-S_phi;
                               0,S_phi/C_theta,C_phi/C_theta];
    Angular_Acceleration    = [Gamma(1)*p*q-Gamma(2)*q*r,Gamma(3)*l+Gamma(4)*n;
                               Gamma(5)*p*r-Gamma(6)*(p^2-r^2),1/Jy*m;
                               Gamma(7)*p*q-Gamma(1)*q*r,Gamma(4)*l+Gamma(8)*n];
                               

    % Initialize
    x_dot = zeros(length(x),1);

    % Equations of Motion
    x_dot(1:3)   = Velocity*x(4:6);
    x_dot(4:6)   = Acceleration+1/mass*[fx;fy;fz];
    x_dot(7:9)   = Angular_Velocity*x(10:12);
    x_dot(10:12) = sum(Angular_Acceleration,2);
    
%     if x_dot(10)<-2
%         throw = 1;
%     end

    
    if ismethod(core,"publish")
        core.publish("V_a_dot",x_dot(4:6))
    end

end

% Anamation Information
function [points,colors,history] = get_drawing(x,settings,param)
    persistent previous_positions
    
    if isempty('previous_positions')
        previous_positions = [];
    end
    
    previous_positions = [previous_positions,x(1:3)];
    
    c_phi = cos(x(7));
    s_phi = sin(x(7));
    c_theta = cos(x(8));
    s_theta = sin(x(8));
    c_psi = cos(x(9));
    s_psi = sin(x(9));

    settings.step;
    
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
    
    % Rotation matricies
    R_bv2 = [1,0,0;
             0,c_phi,-s_phi;
             0,s_phi,c_phi];
    R_v2v1 = [c_theta,0,s_theta;
              0,1,0;
              -s_theta,0,c_theta];
    R_v1v = [c_psi,-s_psi,0;
             s_psi,c_psi,0;
             0,0,1];
    R_bv = R_v1v*R_v2v1*R_bv2;
    R_NED_ENU = [0,1,0;
                 1,0,0;
                 0,0,-1];
    
    for i = 1:length(points)
        points{i} = points{i}.';
        
        points{i} = R_bv*points{i};
        points{i} = x(1:3)+points{i};
        
        points{i} = R_NED_ENU*points{i};
    end
    
    history{1} = R_NED_ENU*previous_positions;
end

% Get position
function [y_r_out,y_r_dot_out] = y_r(x,core)

    h = -x(3);

    phi = x(7);
    theta = x(8);
    psi = x(9);
    
    p = x(10);
    q = x(11);
    r = x(12);

    c_phi = cos(phi);
    s_phi = sin(phi);
    c_theta = cos(theta);
    s_theta = sin(theta);
    c_psi = cos(psi);
    s_psi = sin(psi);

    R_bv2 = [1,0,0;
             0,c_phi,-s_phi;
             0,s_phi,c_phi];
    R_v2v1 = [c_theta,0,s_theta;
              0,1,0;
              -s_theta,0,c_theta];
    R_v1v = [c_psi,-s_psi,0;
             s_psi,c_psi,0;
             0,0,1];
    R_bv = R_v1v*R_v2v1*R_bv2;
    
    V_g_b = x(4:6);
    V_g_g = R_bv*V_g_b;
    h_dot = -V_g_g(3);
    
    try
        V_a_b = core.subscribe("V_a");
        V_a_b_dot = core.subscribe("V_a_dot");
    catch
        V_a_b = x(4:6);
        V_a_b_dot = [0;0;0];
    end
    u_a = V_a_b(1);
    v_a = V_a_b(2);
    w_a = V_a_b(3);
    u_a_dot = V_a_b_dot(1);
    v_a_dot = V_a_b_dot(2);
    w_a_dot = V_a_b_dot(3);
    V_a = sqrt(sum(V_a_b.^2));
    V_a_dot = sqrt(sum(V_a_b_dot.^2));
    beta = atan2(v_a,sqrt(u_a^2+w_a^2));
    beta_dot = 1/(v_a^2/(u_a^2+w_a^2)+1)*(v_a_dot/sqrt(u_a^2+w_a^2)-v_a*(u_a*u_a_dot+w_a*w_a_dot)/(u_a^2+w_a^2)^(3/2));
    chi = atan2(V_g_g(2),V_g_g(1));
    
    rotational_velocity = [1,s_phi*tan(theta),c_phi*tan(theta);
                           0,c_phi,-s_phi;
                           0,s_phi/c_theta,c_phi/c_theta]*[p;q;r];
    phi_dot = rotational_velocity(1);
    theta_dot = rotational_velocity(2);
    chi_dot = rotational_velocity(3);
    
    y_r_out(1,1) = chi;
    y_r_out(2,1) = phi;
    y_r_out(3,1) = h;
    y_r_out(4,1) = theta;
    y_r_out(5,1) = beta;
    y_r_out(6,1) = V_a;
    
    y_r_dot_out(1,1) = chi_dot;
    y_r_dot_out(2,1) = phi_dot;
    y_r_dot_out(3,1) = h_dot;
    y_r_dot_out(4,1) = theta_dot;
    y_r_dot_out(5,1) = beta_dot;
    y_r_dot_out(6,1) = V_a_dot;
end

function output = x_star(alpha,beta,phi,V_a,R,gamma)
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

function output = x_dot_star(V_a,R,gamma)
    output = [V_a*cos(gamma);0;-V_a*sin(gamma);0;0;0;0;0;V_a/R;0;0;0];
end

function output = u_star(alpha,beta,V_a,x,param)
    air  = atmosphere(0);
    rho = air.rho;

    v       = x(5);
    w       = x(6);
    theta   = x(8);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    g    = param.g; % Rigid Body
    mass = param.mass;
    Jx   = param.Jx;
    Jy   = param.Jy;
    Jz   = param.Jz;
    Jxz  = param.Jxz;
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
    % Shorthand
    Gamma(1) = Jxz*(Jx-Jy+Jz)/(Jx*Jz-Jxz^2);
    Gamma(2) = (Jz*(Jz-Jy)+Jxz^2)/(Jx*Jz-Jxz^2);
    Gamma(3) = Jz/(Jx*Jz-Jxz^2);
    Gamma(4) = Jxz/(Jx*Jz-Jxz^2);
    Gamma(5) = (Jz-Jx)/Jy;
    Gamma(6) = Jxz/Jy;
    Gamma(7) = ((Jx-Jy)*Jx+Jxz^2)/(Jx*Jz-Jxz^2);
    Gamma(8) = Jx/(Jx*Jz-Jxz^2);
    
    C_p_0 = gamma(3)*C_l_0 + gamma(4)*C_n_0;
    C_p_beta = gamma(3)*C_l_beta + gamma(4)*C_n_beta;
    C_p_p = gamma(3)*C_l_p + gamma(4)*C_n_p;
    C_p_r = gamma(3)*C_l_r + gamma(4)*C_n_r;
    C_p_delta_a = gamma(3)*C_l_delta_a + gamma(4)*C_n_delta_a;
    C_p_delta_r = gamma(3)*C_l_delta_r + gamma(4)*C_n_delta_r;
    C_r_0 = gamma(4)*C_l_0 + gamma(8)*C_n_0;
    C_r_beta = gamma(4)*C_l_beta + gamma(8)*C_n_beta;
    C_r_p = gamma(4)*C_l_p + gamma(8)*C_n_p;
    C_r_r = gamma(4)*C_l_r + gamma(8)*C_n_r;
    C_r_delta_a = gamma(4)*C_l_delta_a + gamma(8)*C_n_delta_a;
    C_r_delta_r = gamma(4)*C_l_delta_r + gamma(8)*C_n_delta_r;
    
    
    sigma = (1+exp(-M*(alpha-alpha_0))+exp(M*(alpha+alpha_0)))/((1+exp(-M*(alpha-alpha_0)))*(1+exp(M*(alpha+alpha_0))));
    C_D = C_D_p+(C_L_0 + C_L_alpha*alpha)^2/(pi*e*AR);
    C_L = (1-sigma)*(C_L_0+C_L_alpha*alpha)+2*sigma*sign(alpha)*sin(alpha)^2*cos(alpha);
    
    C_X = -C_D*cos(alpha)+C_L*sin(alpha);
    C_X_q = -C_D_q*cos(alpha)+C_L_q*sin(alpha);
    C_X_delta_e = -C_D_delta_e*cos(alpha)+C_L_delta_e*sin(alpha);

    delta_e = ((Jxz*(p^2-r^2)+(Jx-Jz)*p*r)/(1/2*rho*V_a^2*c*S_wing) - C_m_0 - C_m_alpha*alpha - C_m_q*c*q/(2*V_a))/C_m_delta_e;
    delta_t = sqrt((2*mass*(-r*v+q*w+g*sin(theta)) - rho*V_a^2*S_wing*(C_X+C_X_q*c*q/(2*V_a)+C_X_delta_e*delta_e))/(rho*S_prop*C_prop*k_motor^2)+(V_a/k_motor)^2);
    delta_long = [C_p_delta_a,C_p_delta_r;C_r_delta_a,C_r_delta_r]^-1*[(-Gamma(1)*p*q+Gamma(2)*q*r)/(1/2*rho*V_a^2*S_wing*b)-C_p_0-C_p_beta*beta-C_p_p*b*p/(2*V_a)-C_p_r*b*r/(2*V_a);
                                                                       (-Gamma(7)*p*q+Gamma(1)*q*r)/(1/2*rho*V_a^2*S_wing*b)-C_r_0-C_r_beta*beta-C_r_p*b*p/(2*V_a)-C_r_r*b*r/(2*V_a)];
    
    output = [delta_long(1);delta_e;delta_long(2);delta_t];
end

function output = J(input,V_a,R,gamma,param)
    
    alpha = input(1);
    beta = input(2);
    phi = input(3);

    param.wind = [0;0;0];
    param.gust_condition = 'steady';
    
    x = x_star(alpha,beta,phi,V_a,R,gamma);
    u = u_star(alpha,beta,V_a,x,param);
    x_dot = x_dot_star(V_a,R,gamma);
    
    core.param = param;
    
    f = eqs_motion(1000,x,u,core);
    output = norm(x_dot(3:end)-f(3:end))^2;
end

function [alpha,beta,phi] = min_J(alpha,beta,phi,V_a,R,gamma,param)
    epsilon = param.optimizer.epsilon;
    N = param.optimizer.N;
    kappa = param.optimizer.kappa;
    for k = 1:N
        alpha_plus = alpha + epsilon;
        beta_plus = beta + epsilon;
        phi_plus = phi + epsilon;
        dJ_dalpha = (J([alpha_plus,beta,phi],V_a,R,gamma,param)-J([alpha,beta,phi],V_a,R,gamma,param))/epsilon;
        dJ_dbeta = (J([alpha,beta_plus,phi],V_a,R,gamma,param)-J([alpha,beta,phi],V_a,R,gamma,param))/epsilon;
        dJ_dphi = (J([alpha,beta,phi_plus],V_a,R,gamma,param)-J([alpha,beta,phi],V_a,R,gamma,param))/epsilon;
        alpha = alpha - kappa*dJ_dalpha;
        beta = beta - kappa*dJ_dbeta;
        phi = phi - kappa*dJ_dphi;
    end
end