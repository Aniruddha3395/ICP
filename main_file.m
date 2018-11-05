%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%DELETE THIS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try convex hull method if you can
% finish kuka code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%world frame is robot base frame
clc;
clear;
close all;
dbstop if error;
set(0, 'DefaultFigureRenderer', 'opengl');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global tool_F_T_tcp x0;
%%%%%%%%%%%%%%%%%
% Tool frame transformation of TCP wrt Flange (T_tcp_wrt_F) tool
% transformation is zero when pts wrt flange
tool_t = [-0.2;-0.01;48.6];                    % in mm
tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
tool_F_T_tcp = [tool_r,tool_t;0 0 0 1];
%%%%%%%%%%%%%%%%%
generate_model_data = false;
generate_scan_traj_data = false;
show_true_T = false;
perturb_true_T = false;
%%%%%%%%%%%%%%%%%
if generate_model_data
    [part_ptcloud,part_ptcloud_normals] = generate_part_ptcloud_from_part_STL('car_bonnet_1031_new.stl');
else
    part_ptcloud = dlmread('part_ptcloud.csv');
    part_ptcloud_normals = dlmread('part_ptcloud_normals.csv');
end
if generate_scan_traj_data
    scan_traj = get_traj_wrt_tcp('data_for_ICP.csv');
else
    scan_traj = dlmread('scanned_traj.csv');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

True_w_T_p = [
    1.0000   -0.0001    0.0002  477.4196
    0.0001    1.0000    0.0014 -143.6004
    -0.0003   -0.0014    1.0000  311.3996
    0         0         0    1.0000
    ];
if show_true_T
    figure;
    %plotting robot base
    scatter3(0,0,0,100,'d','filled','r');
    hold on;
    quiver3(0,0,0,1,0,0,100,'r');hold on;quiver3(0,0,0,0,1,0,100,'g');hold on;quiver3(0,0,0,0,0,1,100,'b');
    hold on;
    %plotting part ptcloud
    scatter3d(part_ptcloud,'.');
    hold on;
    %plotting scan traj
    scatter3d(scan_traj,'*');
    
    transformed_ptcloud = apply_transformation(part_ptcloud,True_w_T_p);
    hold on;
    scatter3d(transformed_ptcloud,'.');
    KDtree = KDTreeSearcher(transformed_ptcloud);
    num_of_neighbours = 1;
    idx = knnsearch(KDtree,scan_traj,'K',num_of_neighbours);
    corresponding_val_from_part_ptcloud = transformed_ptcloud(idx,:);
    Error_max_d = max(dist(corresponding_val_from_part_ptcloud,scan_traj));
    Error_mean_d = sum(dist(corresponding_val_from_part_ptcloud,scan_traj))/size(scan_traj,1);
    fprintf('max_d :%f, mean_d :%f\n',Error_max_d,Error_mean_d);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if perturb_true_T
    perturb_val = 100;
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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%perform icp here
%start T: given by some scanner i.e. input w_T_p
%start seed for icp: eye(4);
% type = 'weighted_max_mean_d';
% type = 'weighted_max_mean_plane_d';
type = 'mean_plane_d';
tol = 0.1;
perturb_val = 0.6;
input_w_T_p  = all_input_w_T_p(12);
input_part_ptcloud_icp = apply_transformation(part_ptcloud,input_w_T_p);
fval_curr = Inf;
input_part_ptcloud_icp_true = input_part_ptcloud_icp;
icp_T_final = eye(4);
x0 = [0 0 0 1 0 0 0];
timer_start = tic;
while fval_curr>tol
% for i = 1:3
    syms tx ty tz q0 q1 q2 q3;
    [icp_T,x,fval] = perform_ICP(input_part_ptcloud_icp,scan_traj,tx,ty,tz,q0,q1,q2,q3,type);
    if fval<fval_curr
        fval_curr=fval;
        x0 = x;
        icp_T_final = icp_T*icp_T_final;
        input_part_ptcloud_icp = apply_transformation(input_part_ptcloud_icp,icp_T);
        icp_T_final_save = icp_T_final;
        input_part_ptcloud_icp_save = input_part_ptcloud_icp;
    else
        x0 = [0 0 0 1 0 0 0];
        perturb_val_t = -perturb_val + 2*perturb_val*rand(3,1);
        icp_t = icp_T(1:3,4) + perturb_val_t;
        perturb_val_r = [-0.5*perturb_val + 1*perturb_val*rand(1,3)].*(pi/180);
        icp_r = eul2rotm([rotm2eul(icp_T(1:3,1:3)) + perturb_val_r]);
        icp_T = [icp_r,icp_t;0 0 0 1];
        icp_T_final = icp_T*icp_T_final;
        input_part_ptcloud_icp = apply_transformation(input_part_ptcloud_icp,icp_T);
        %         perturb_seed_val = 10;
        %         perturb_seed_val_t = -perturb_seed_val + 2*perturb_seed_val*rand(1,3);
        %         x0(1,1:3) = x(1,1:3) + perturb_seed_val_t;
        %         perturb_seed_val_r = [-0.5*perturb_seed_val + 1*perturb_seed_val*rand(1,3)].*(pi/180);
        %         x0(1,4:7) = eul2quat(quat2eul(x(1,4:7))+perturb_seed_val_r);   
    end
    disp([fval_curr,fval]);
    if toc(timer_start)>120;
        break;
    end
end

%icp_T is transformation of part wrt scan 
Final_w_T_p = icp_T_final_save*input_w_T_p;
hold on;
%plotting robot base
scatter3(0,0,0,100,'d','filled','r');
hold on;
quiver3(0,0,0,1,0,0,100,'r');hold on;quiver3(0,0,0,0,1,0,100,'g');hold on;quiver3(0,0,0,0,0,1,100,'b');
hold on;
%plotting scan traj
scatter3d(scan_traj,'filled');

% transformed_ptcloud = apply_transformation(input_part_ptcloud_icp_true,eye(4));
% hold on;
% scatter3d(transformed_ptcloud,'r.');
% transformed_ptcloud = apply_transformation(input_part_ptcloud_icp,icp_T);
% hold on;
% scatter3d(transformed_ptcloud,'b.');

transformed_ptcloud = apply_transformation(part_ptcloud,Final_w_T_p);
hold on;
scatter3d(transformed_ptcloud,'g.');

KDtree = KDTreeSearcher(transformed_ptcloud);
num_of_neighbours = 1;
idx = knnsearch(KDtree,scan_traj,'K',num_of_neighbours);
corresponding_val_from_part_ptcloud = transformed_ptcloud(idx,:);
Error_max_d = max(dist(corresponding_val_from_part_ptcloud,scan_traj));
Error_mean_d = sum(dist(corresponding_val_from_part_ptcloud,scan_traj))/size(scan_traj,1);
Error_weighted_max_mean_d = (Error_max_d+Error_mean_d)/2;

idx = knnsearch(KDtree,scan_traj,'K',5);
d = zeros(size(scan_traj,1),1);
for i = 1:size(scan_traj,1)
corresponding_val_from_part_ptcloud = transformed_ptcloud(idx(i,:),:);
d(i) = get_pt_to_lsf_plane_dist(scan_traj(i,:),corresponding_val_from_part_ptcloud);
end
Error_mean_plane_d = sum(d)/size(d,1);
Error_max_plane_d = max(d);

fprintf('max_d :%f,\n max_plane_d :%f,\n mean_d :%f,\n mean_plane_d :%f,\n weighted_max_mean_d :%f\n',Error_max_d,Error_max_plane_d,Error_mean_d,Error_mean_plane_d,Error_weighted_max_mean_d);

disp(Final_w_T_p);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %perform icp here
% %start T: given by some scanner i.e. input w_T_p
% %start seed for icp: eye(4);
% input_w_T_p  = all_input_w_T_p(1);
% input_part_ptcloud_icp = apply_transformation(part_ptcloud,input_w_T_p);
% fval_curr = Inf;
% input_part_ptcloud_icp_true = input_part_ptcloud_icp;
% icp_T_final = eye(4);
% 
% while fval_curr>0.1
% % for i = 1:3
%     x0 = [0 0 0 1 0 0 0];
%     syms tx ty tz q0 q1 q2 q3;
%     [icp_T,x,fval_curr] = perform_icp(input_part_ptcloud_icp,scan_traj,tx,ty,tz,q0,q1,q2,q3);
%     disp(fval_curr);
%     x0 = x;
%     icp_T_final = icp_T*icp_T_final;
%     input_part_ptcloud_icp = apply_transformation(input_part_ptcloud_icp,icp_T);
% end
% 
% %icp_T is transformation of part wrt scan 
% Final_w_T_p = icp_T_final*input_w_T_p;
% hold on;
% %plotting robot base
% scatter3(0,0,0,100,'d','filled','r');
% hold on;
% quiver3(0,0,0,1,0,0,100,'r');hold on;quiver3(0,0,0,0,1,0,100,'g');hold on;quiver3(0,0,0,0,0,1,100,'b');
% hold on;
% %plotting scan traj
% scatter3d(scan_traj,'filled');
% 
% % transformed_ptcloud = apply_transformation(input_part_ptcloud_icp_true,eye(4));
% % hold on;
% % scatter3d(transformed_ptcloud,'r.');
% % transformed_ptcloud = apply_transformation(input_part_ptcloud_icp,icp_T);
% % hold on;
% % scatter3d(transformed_ptcloud,'b.');
% 
% transformed_ptcloud = apply_transformation(part_ptcloud,Final_w_T_p);
% hold on;
% scatter3d(transformed_ptcloud,'g.');


