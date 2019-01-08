%% Header
% Parameters

% Object for passing data
core = piping();

%% Pysical Parameters

% Aircraft Dimensions
% Wing
param.wing_w = 1.5;
param.wing_l = 0.2;
    
% Fusalage
param.fuse_w = 0.15;
param.fuse_l1 = 0.2;
param.fuse_l2 = 0.1;
param.fuse_l3 = 0.8;
param.fuse_h = 0.15;
    
% Horizontal Stabalizer
param.tailwing_w = 0.2;
param.tailwing_l = 0.12;
    
% Vertical Stabalizer
param.tail_h = 0.1;

% State Description
param.x_names = ["\theta";"\phi";"\psi"];
param.u_names = ["Thrust"];

%% Simulation 
% Dimensions
settings.side = 0.25;     % m - Dimension of the side of the box

% Simulation
settings.start       = 0;      % s
settings.step        = 0.01;   % s
settings.end         = 50;     % s
t = settings.start:settings.step:settings.end;

settings.publish     = 0.2;    % s
settings.window      = [-1,1,-1,1,-1,1]; % m

%% Functions

functions.get_drawing = @get_drawing;

% Anamation Information
function [points,colors] = get_drawing(x,settings,param)
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
    wing.left = [0,wing_w/2,0;
                 -wing_l,wing_w/2,0;
                 -wing_l,0,0;
                 0,0,0];
    wing.right = [0,-wing_w/2,0;
                  -wing_l,-wing_w/2,0;
                  -wing_l,0,0;
                  0,0,0];
    fusalage.nose.top = [fuse_l2,fuse_w/2,fuse_h/2;
                         fuse_l2,-fuse_w/2,fuse_h/2;
                         fuse_l1,0,0];
    fusalage.nose.left = [fuse_l2,-fuse_w/2,fuse_h/2;
                          fuse_l2,-fuse_w/2,-fuse_h/2;
                          fuse_l1,0,0];
    fusalage.nose.right = [fuse_l2,fuse_w/2,fuse_h/2;
                           fuse_l2,fuse_w/2,-fuse_h/2;
                           fuse_l1,0,0];
    fusalage.nose.bottom = [fuse_l2,fuse_w/2,-fuse_h/2;
                            fuse_l2,-fuse_w/2,-fuse_h/2;
                            fuse_l1,0,0];
    fusalage.empanage.top = [fuse_l2,fuse_w/2,fuse_h/2;
                            fuse_l2,-fuse_w/2,fuse_h/2;
                            -fuse_l3,0,0];
    fusalage.empanage.left = [fuse_l2,-fuse_w/2,fuse_h/2;
                              fuse_l2,-fuse_w/2,-fuse_h/2;
                              -fuse_l3,0,0];
    fusalage.empanage.right = [fuse_l2,fuse_w/2,fuse_h/2;
                               fuse_l2,fuse_w/2,-fuse_h/2;
                               -fuse_l3,0,0];
    fusalage.empanage.bottom = [fuse_l2,fuse_w/2,-fuse_h/2;
                                fuse_l2,-fuse_w/2,-fuse_h/2;
                                -fuse_l3,0,0];
    horz_stab = [-fuse_l3,tailwing_w/2,0;
                 -fuse_l3+tailwing_l,tailwing_w/2,0;
                 -fuse_l3+tailwing_l,-tailwing_w/2,0;
                 -fuse_l3,-tailwing_w/2,0];
    vir_stab = [-fuse_l3,0,0;
                -fuse_l3,0,tail_h;
                -fuse_l3+tailwing_l,0,0;];
    


    points = {wing.right,wing.left,fusalage.nose.top,fusalage.nose.left,fusalage.nose.right,fusalage.nose.bottom,fusalage.empanage.top,fusalage.empanage.left,fusalage.empanage.right,fusalage.empanage.bottom,horz_stab,vir_stab};
    colors = {'g','r','k','k','k','k','k','k','k','k','k','k'};
end