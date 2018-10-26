clc;
clear;
close all;
dbstop if error;
set(0, 'DefaultFigureRenderer', 'opengl');

% profile on;

global main_dir;
global Seed_T tool_T Complete_T KDtree;
global raw_scanned_pts_path_dir raw_scanned_pts_file_name raw_scanned_pts_wrt_tcp_file_name;
global path_for_storing_sample_data;
global input_file input_file_pts_from_kuka_scan;
global output_file_path output_file;
global stl_name;
global model_ptcloud_normals;
global tcp_publisher_pts_are_flange_pts;
global scan_ptcloud_file model_ptcloud_file model_ptcloud_normals_file;


True_t = [478.0508;-141.7436;312.4528];                   % in mm
True_r = eul2rotm([0 0 0]);
True_T = [True_r,True_t;0 0 0 1];

%% Definitions and Inputs
%%%%%%%%%%%%  input directries  %%%%%%%%%%%%%
main_dir = '/home/rflin/Desktop/';
%
stl_file_path = 'ICP/CADandSTL/car_bonnet/';
stl_file ='sample_file_8_car_bonnet.STL';
%
raw_scanned_pts_path_dir = 'ICP/test/test_for_car_bonnet/';
raw_scanned_pts_file_name = 'data_for_ICP.csv';
raw_scanned_pts_wrt_tcp_file_name = 'data_for_icp_wrt_tcp.csv';
%
path_for_storing_sample_data = 'ICP/sample_data/car_bonnet_samples/';
model_ptcloud_file ='sample_34_model_ptcloud.csv';
model_ptcloud_normals_file = 'sample_34_model_ptcloud_normals.csv';
scan_ptcloud_file = 'sample_34_scan_ptcloud.csv';
%
%pts file on which direct transformation is applied...either scnned pts
%from kuka or pts on parts
input_file = raw_scanned_pts_file_name;
input_file = '/home/rflin/Desktop/ICP/test/test_for_car_bonnet/bdry_pts_inf_loop.csv';
%
output_file_path = 'ICP/';
output_file = 'test_data.csv';

% Input transformations for ICP
% Seed/Initial guess transformation of part wrt robot base
% Seed_t = [476.91;-141.46;310.52];                   % in mm
% Seed_r = eul2rotm([0 0 0]);                         % in rad (alpha beta gamma rot)
Seed_t = [471.91;-146.46;303.52];                   % in mm
Seed_r = eul2rotm([5*pi/180 0 0]);                  % in rad (alpha beta gamma rot)

%%%%%%%%%%
% %generating new seed for initial guess....distorting x y z and eul angles
% perturb_val_t = -10 + 20*rand(3,1);
% perturb_val_r = [-5 + 10*rand(1,3)].*(pi/180);
% Seed_t = True_t + perturb_val_t;
% Seed_r = eul2rotm([rotm2eul(True_r) + perturb_val_r]);
%%%%%%%%%%


Seed_T = [Seed_r,Seed_t;0 0 0 1];

% Tool frame transformation of TCP wrt Flange (T_tcp_wrt_F)
tool_t = [-0.2;-0.01;48.6];                    % in mm
tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
tool_T = [tool_r,tool_t;0 0 0 1];

%%%%%%%%
generate_data = true;
tcp_publisher_pts_are_flange_pts = true;
T_part_wrt_base_for_seed = true;
input_file_pts_from_kuka_scan = false;   %false if input file is from part frame
% options: use_fmincon, use_fminunc
optm_method = 'use_fmincon';
%options: sum_d, max_d, mean_d, rms_d
error_fun = 'mean_d';
%%%%%%%%

stl_name = strcat(main_dir,stl_file_path,stl_file);
if generate_data
    [model_ptcloud,model_ptcloud_normals] = generate_model_ptcloud_from_STL(stl_name);
    dlmwrite(strcat(main_dir,path_for_storing_sample_data,model_ptcloud_file),model_ptcloud);
    dlmwrite(strcat(main_dir,path_for_storing_sample_data,model_ptcloud_normals_file),model_ptcloud_normals);
    scan_ptcloud = get_seed_for_icp(tcp_publisher_pts_are_flange_pts);
    
else
    model_ptcloud = dlmread(strcat(main_dir,path_for_storing_sample_data,model_ptcloud_file));
    model_ptcloud_normals = dlmread(strcat(main_dir,path_for_storing_sample_data,model_ptcloud_normals_file));
    scan_ptcloud = dlmread(strcat(main_dir,path_for_storing_sample_data,scan_ptcloud_file));
end


KDtree = KDTreeSearcher(model_ptcloud);


%% decision of selcting the sedd after pertubation....error metric is fval jump
small_val = 0.5;
big_val = 5;
fval_curr = Inf;
while fval_curr>1.6
% for i = 1:1
cla;

scan_ptcloud = get_seed_for_icp(tcp_publisher_pts_are_flange_pts);
[ICP_transformation_matrix,fval_new] = perform_ICP(model_ptcloud,scan_ptcloud,optm_method,error_fun);
if T_part_wrt_base_for_seed
    Complete_T = ICP_transformation_matrix*inv(Seed_T);
else
    Complete_T = ICP_transformation_matrix*Seed_T;
end
Final_T = inv(Complete_T);

if fval_new<fval_curr
    fval_curr = fval_new;
    perturb_val_t = small_val + 2*small_val*rand(3,1);
    Seed_t = Final_T(1:3,4) + perturb_val_t;
    perturb_val_r = [-0.5*small_val + small_val*rand(1,3)].*(pi/180);
    Seed_r = eul2rotm([rotm2eul(Final_T(1:3,1:3)) + perturb_val_r]);
    Seed_T = [Seed_r,Seed_t;0 0 0 1];
else
    perturb_val_t = -big_val + 2*big_val*rand(3,1);
    Seed_t_new = Seed_t + perturb_val_t;
    perturb_val_r = [-0.5*big_val + big_val*rand(1,3)].*(pi/180);
    Seed_r_new = eul2rotm([rotm2eul(Seed_r) + perturb_val_r]);
    Seed_T = [Seed_r_new,Seed_t_new;0 0 0 1];
end


[fval_curr,  fval_new]
end

Final_T
%%% after this if i apply local optimizer again will it work??
%%%%try that
%%%%also make another entry of sa like fmincon ....make other entries if
%%%%required as well








%% decision of selcting the sedd after pertubation....error metric is sumt^2 and sumr^2
% 
% E_t_curr = Inf;
% E_r_curr = Inf;
% 
% for i = 1:5
% cla;
% 
% perturb_val_t = -10 + 20*rand(3,1);
% Seed_t = Seed_t + perturb_val_t;
% perturb_val_r = [-5 + 10*rand(1,3)].*(pi/180);
% Seed_r = eul2rotm([rotm2eul(Seed_r) + perturb_val_r]);
% Seed_T = [Seed_r,Seed_t;0 0 0 1];
% 
% scan_ptcloud = get_seed_for_icp(tcp_publisher_pts_are_flange_pts);
% [ICP_transformation_matrix] = perform_ICP(model_ptcloud,scan_ptcloud,optm_method,error_fun);
% if T_part_wrt_base_for_seed
%     Complete_T = ICP_transformation_matrix*inv(Seed_T);
% else
%     Complete_T = ICP_transformation_matrix*Seed_T;
% end
% Final_T = inv(Complete_T);
% 
% E_t_new = sum( [True_T(1:3,4) - Final_T(1:3,4)].^2 );
% E_r_new = sum( sum( [True_T(1:3,1:3) - Final_T(1:3,1:3)].^2) );
% 
% if E_t_new<E_t_curr
%     E_t_curr = E_t_new;
%     Seed_t = Final_T(1:3,4);
% end
% 
% 
% if E_r_new<E_r_curr
%     E_r_curr = E_r_new;
%     Seed_r = Final_T(1:3,1:3);
% end
% 
% end

%    [ 1.0000   -0.0036   -0.0060  475.9318
%     0.0035    1.0000   -0.0055 -145.9004
%     0.0060    0.0055    1.0000  310.7967
%          0         0         0    1.0000]


%% only checking if after error after perturbation is less than tol or not
% not decision of when to stop or what to select othewise

% t_safe = false;
% r_safe = false;
% while t_safe~=true && r_safe~=true
% %     for i = 1:1
%     cla;
%     scan_ptcloud = get_seed_for_icp(tcp_publisher_pts_are_flange_pts);
%     [ICP_transformation_matrix] = perform_ICP(model_ptcloud,scan_ptcloud,optm_method,error_fun);
% 
%     if T_part_wrt_base_for_seed
%         Complete_T = ICP_transformation_matrix*inv(Seed_T);
%     else
%         Complete_T = ICP_transformation_matrix*Seed_T;
%     end
%     Final_T = inv(Complete_T);
%     
%     E = (True_T - Final_T)
%     if sum(E(1:3,4).^2)>0.5
%         t_safe=false;
%         perturb_val_t = -10 + 20*rand(3,1);
%         Seed_t = True_t + perturb_val_t;
%     else
%         t_safe=true;
%     end
%     
%     if sum(sum(E(1:3,1:3).^2))>1e-6
%         r_safe=false;
%         perturb_val_r = [-5 + 10*rand(1,3)].*(pi/180);
%         Seed_r = eul2rotm([rotm2eul(True_r) + perturb_val_r]);
%     else
%         r_safe=true;
%     end
%     Seed_T = [Seed_r,Seed_t;0 0 0 1];
%     
% end
% 




% %% ICP
%
% figure;
% KDtree = KDTreeSearcher(model_ptcloud);
% [ICP_transformation_matrix] = perform_ICP(model_ptcloud,scan_ptcloud,optm_method,error_fun);
%
% %% final transformation matrix from part frame to robot base frame
%
% if T_part_wrt_base_for_seed
%     Complete_T = ICP_transformation_matrix*inv(Seed_T);
% else
%     Complete_T = ICP_transformation_matrix*Seed_T;
% end
% Final_T = inv(Complete_T);















%% tests and geting complete transformation

% input_file_pts_from_kuka_scan = true;
% input_file = raw_scanned_pts_file_name;

% run evaluate_vals.m

% profile viewer;