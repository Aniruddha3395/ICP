clc;
clear;
close all;
dbstop if error;

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
Seed_t = [476.91;-141.46;310.52];                   % in mm
Seed_t = [471.91;-146.46;303.52];                   % in mm
Seed_r = eul2rotm([5*pi/180 0 0]);                         % in rad (alpha beta gamma rot)
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
error_fun = 'max_d';
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

%% ICP
figure;
KDtree = KDTreeSearcher(model_ptcloud);
[ICP_transformation_matrix] = perform_ICP(model_ptcloud,scan_ptcloud,optm_method,error_fun);

%% final transformation matrix from part frame to robot base frame
if T_part_wrt_base_for_seed
    Complete_T = ICP_transformation_matrix*inv(Seed_T);
else
    Complete_T = ICP_transformation_matrix*Seed_T;
end
Final_T = inv(Complete_T)

%% tests and geting complete transformation

% input_file_pts_from_kuka_scan = true;
% input_file = raw_scanned_pts_file_name;

% run evaluate_vals.m

% profile viewer;