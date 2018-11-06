%world frame is robot base frame
clc;
clear;
close all;
dbstop if error;
set(0, 'DefaultFigureRenderer', 'opengl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
run global_CONFIG.m

perturb_val = 10;
perturb_val_t = -perturb_val + 2*perturb_val*rand(3,1)
input_w_t_p = True_w_T_p(1:3,4) + perturb_val_t;
perturb_val_r = [-0.8*perturb_val + 1.6*perturb_val*rand(1,3)]
perturb_val_r = perturb_val_r.*(pi/180);
% perturb_val_r = [perturb_val + 2*perturb_val*rand(1,3)].*(pi/180);
input_w_r_p = eul2rotm([rotm2eul(True_w_T_p(1:3,1:3)) + perturb_val_r]);
input_w_T_p = [input_w_r_p,input_w_t_p;0 0 0 1];

figure;
%plotting robot base
scatter3(0,0,0,100,'d','filled','r');
hold on;
quiver3(0,0,0,1,0,0,100,'r');hold on;quiver3(0,0,0,0,1,0,100,'g');hold on;quiver3(0,0,0,0,0,1,100,'b');
hold on;
%plotting scan traj
scatter3d(scan_traj,'.');

transformed_ptcloud = apply_transformation(part_ptcloud,input_w_T_p);
hold on;
scatter3d(transformed_ptcloud,'.');
KDtree = KDTreeSearcher(transformed_ptcloud);
num_of_neighbours = 1;
idx = knnsearch(KDtree,scan_traj,'K',num_of_neighbours);
corresponding_val_from_part_ptcloud = transformed_ptcloud(idx,:);
Error_max_d = max(dist(corresponding_val_from_part_ptcloud,scan_traj));
Error_mean_d = sum(dist(corresponding_val_from_part_ptcloud,scan_traj))/size(scan_traj,1);
fprintf('input_w_T_p:\n\n');
disp(input_w_T_p);
fprintf('max_d :%f, mean_d :%f\n',Error_max_d,Error_mean_d);