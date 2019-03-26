clear all
clc

plan.collision_step = 0.01;
plan.margin_of_saftey = 1;
test = path_planner(plan);
obsticals.corners = [1,1,-1,-1,1,1,-1,-1;
                     1,-1,1,-1,1,-1,1,-1;
                     1,1,1,1,-1,-1,-1,-1];
v1 = [0;5;0];
v2 = [0;-5;0];

test.check_for_collision(v1,v2,obsticals)