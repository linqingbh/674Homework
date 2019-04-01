function [A,B,C,D] = get_linear_model(eqs_motion,get_y_r,sensors,param)
    f = @(x,u) eqs_motion([],x,u,param);
    h = @(x,u) get_y_r(sense(x,eqs_motion([],x,u,param),sensors,param),x,zeros(size(param.d_names)),param);
    
%     f = @(x,u) y_as_x(x,u,eqs_motion,get_y_r,sensors,param);
%     h = @(x,~) x;

    A = numerical_jacobian(@(x) f(x,param.u_0),param.x_0);
    B = numerical_jacobian(@(u) f(param.x_0,u),param.u_0);
    C = numerical_jacobian(@(x) h(x,param.u_0),param.x_0,[],[],[],param.r_is_angle);
    D = numerical_jacobian(@(u) h(param.x_0,u),param.u_0,[],[],[],param.r_is_angle);
    
    [A2,B2]= get_book_model(param);
%     A([4,6,11,8,3],[4,6,11,8,3]) = A2;
%     B([4,6,11,8,3],[2,4]) = B2;
end

function x = y2x(y_r,y_r_dot)
x(1,1) = 0;
x(2,1) = 0;
x(3,1) = -y_r(3);
x(4,1) = fminsearch(@(u) find_velocity(u,y_r(6),y_r(5),-y_r_dot(3),y_r(2),y_r(4),y_r(1)),y_r(6));
x(5,1) = sin(y_r(5))*y_r(6);
x(6,1) = sqrt(y_r(6)^2-x(4)^2-x(5)^2);
x(7,1) = y_r(2);
x(8,1) = y_r(4);
x(9,1) = y_r(1);
x(10:12,1) = [1,sin(y_r(2))*tan(y_r(4)),-cos(y_r(2))*tan(y_r(4));
            0,cos(y_r(2)),sin(y_r(2));
            0,-sin(y_r(2))/cos(y_r(4)),cos(y_r(2))/cos(y_r(4))]*y_r_dot([2,4,1]);
end


function z = sense(x,x_dot,sensors,param)
    z = zeros(size(param.z_names));
    d = zeros(size(param.d_names));
    for j = 1:length(sensors)
        last_update = sensors(j).last_update;
        last_reading = sensors(j).y_m;
         z(sensors(j).z_indexes) = sensors(j).sense(x,x_dot,d,[]);
        sensors(j).last_update = last_update;
        sensors(j).y_m = last_reading;
    end
end

function y_dot = y_as_x(y,u,eqs_motion,get_y_r,sensors,param)
    [~,y_r_dot,y_r_ddot] = get_y_r(sense(y2x(y(1:6),y(7:end)),eqs_motion([],y2x(y(1:6),y(7:end)),u,param),sensors,param),y2x(y(1:6),y(7:end)),zeros(size(param.d_names)),param,eqs_motion([],y2x(y(1:6),y(7:end)),u,param));
    y_dot = [y_r_dot;y_r_ddot];
end

function error = find_velocity(u,V_a,beta,p_d_dot_actual,phi,theta,psi)
    v = sin(beta)*V_a;
    w = sqrt(V_a^2-u^2-v^2);
    V_g = get_rotation(phi,theta,psi,'b->v',[u;v;w]);
    p_d_dot = V_g(3);
    error = abs(p_d_dot_actual-p_d_dot);
end

function [A,B] = get_book_model(param)
    my_unpack(param)
    delta_a = u_0(1);
    delta_e = u_0(2);
    delta_r = u_0(3);
    delta_t = u_0(4);
    u = x_0(4);
    v = x_0(5);
    w = x_0(6);
    phi = x_0(7);
    theta = x_0(8);
    psi = x_0(9);
    p = x_0(10);
    q = x_0(11);
    r = x_0(12);
    S = S_wing;
    alpha = theta;
    m = mass;
    V_a = sqrt(u^2+v^2+w^2);
    
    C_X_0 = -C_D_0*cos(alpha)+C_L_0*sin(alpha);
    C_X_q = -C_D_q*cos(alpha)+C_L_q*sin(alpha);
    C_X_alpha = -C_D_alpha*cos(alpha)+C_L_alpha*sin(alpha);
    C_X_delta_e = -C_D_delta_e*cos(alpha)+C_L_delta_e*sin(alpha);
    C_Z_0 = -C_D_0*sin(alpha)-C_L_0*sin(alpha);
    C_Z_q = -C_D_q*sin(alpha)-C_L_q*sin(alpha);
    C_Z_alpha = -C_D_alpha*sin(alpha)-C_L_alpha*sin(alpha);
    C_Z_delta_e = -C_D_delta_e*sin(alpha)-C_L_delta_e*sin(alpha);
    
    X_u =    u*rho*S/m*(C_X_0 + C_X_alpha*alpha+C_X_delta_e*delta_e)-rho*S*w*C_X_alpha/(2*m)+rho*S*c*C_X_q*u*q/(4*m*V_a)-rho*S_prop*C_prop*u/m;
    X_w = -q+w*rho*S/m*(C_X_0 + C_X_alpha*alpha+C_X_delta_e*delta_e)+rho*S*c*C_X_q*w*q/(4*m*V_a)+rho*S*C_X_alpha*u/(2*m)-rho*S_prop*C_prop*w/m;
    X_q = -w+rho*V_a*S*C_X_q*c/(4*m);
    X_delta_e = rho*V_a^2*S*C_X_delta_e/(2*m);
    X_delta_t = rho*S_prop*C_prop*k_motor^2*delta_t/m;
    Z_u = q+u*rho*S/m*(C_Z_0+C_Z_alpha*alpha+C_Z_delta_e*delta_e)-rho*S*C_Z_alpha*w/(2*m)+u*rho*S*C_Z_q*c*q/(4*m*V_a);
    Z_w =   w*rho*S/m*(C_Z_0+C_Z_alpha*alpha+C_Z_delta_e*delta_e)+rho*S*C_Z_alpha*u/(2*m)+rho*w*S*c*C_Z_q*q/(4*m*V_a);
    Z_q = u+rho*V_a*S*C_Z_q*c/(4*m);
    Z_delta_e = rho*V_a^2*S*C_Z_delta_e/(2*m);
    M_u = u*rho*S*c/Jy*(C_m_0+C_m_alpha*alpha+C_m_delta_e*delta_e)-rho*S*c*C_m_alpha*w/(2*Jy)+rho*S*c^2*C_m_q*q*u/(4*Jy*V_a);
    M_w = w*rho*S*c/Jy*(C_m_0+C_m_alpha*alpha+C_m_delta_e*delta_e)+rho*S*c*C_m_alpha*u/(2*Jy)+rho*S*c^2*C_m_q*q*w/(4*Jy*V_a);
    M_q = rho*V_a*S*c^2*C_m_q/(4*Jy);
    M_delta_e = rho*V_a^2*S*c*C_m_delta_e/(2*Jy);
    
    A = [X_u,X_w,X_q,-g*cos(theta),0;
         Z_u,Z_w,Z_q,-g*sin(theta),0;
         M_u,M_w,M_q,0,0;
         0,0,1,0,0;
         -sin(theta),cos(theta),0,-u*cos(theta)-w*sin(theta),0];
    B = [X_delta_e,X_delta_t;
         Z_delta_e,0;
         M_delta_e,0;
         0,0;
         0,0];
    
    
    L_p = rho*V_design*S_wing*b^2/4*C_p_p;
    L_r = rho*V_design*S_wing*b^2/4*C_p_r;
    L_delta_a = rho*V_design^2*S_wing*b/2*C_p_delta_a;
    L_delta_r = rho*V_design^2*S_wing*b/2*C_p_delta_r;
    N_p = rho*V_design*S_wing*b^2/4*C_r_p;
    N_r = rho*V_design*S_wing*b^2/4*C_r_r;
    N_delta_a = rho*V_design^2*S_wing*b/2*C_r_delta_a;
    N_delta_r = rho*V_design^2*S_wing*b/2*C_r_delta_r;

    
end
