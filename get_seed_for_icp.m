function [pts_for_icp] = get_seed_for_icp(tcp_publisher_pts_are_flange_pts)

global main_dir;
global Seed_T;
global tool_T;
global raw_scanned_pts_path_dir;
global raw_scanned_pts_file_name;
global raw_scanned_pts_wrt_tcp_file_name;
global pts_from_tcp_publisher_wrt_tcp;
global scan_ptcloud_file;
global path_for_storing_sample_data;
global model_ptcloud_file;

%get appropriate scan data for ICP
pts_from_kuka_scanning = dlmread(strcat(main_dir,raw_scanned_pts_path_dir,raw_scanned_pts_file_name));

%get points from robot publisher
pts_from_tcp_publisher_wrt_tcp =  get_pts_from_scan(pts_from_kuka_scanning,tool_T,tcp_publisher_pts_are_flange_pts);

dlmwrite(strcat(main_dir,raw_scanned_pts_path_dir,raw_scanned_pts_wrt_tcp_file_name),pts_from_tcp_publisher_wrt_tcp);

%making pts from robot wrt part applying inverse seed transformation
pts_for_icp = apply_transformation(pts_from_tcp_publisher_wrt_tcp,inv(Seed_T));
dlmwrite(strcat(main_dir,path_for_storing_sample_data,scan_ptcloud_file),pts_for_icp);

%% visualization of seed

model_ptcloud = dlmread(strcat(main_dir,path_for_storing_sample_data,model_ptcloud_file));

scatter3d(model_ptcloud,'.');
hold on;
scatter3d(pts_from_kuka_scanning,'.');
hold on;
scatter3d(pts_from_tcp_publisher_wrt_tcp,'.');
hold on;
scatter3d(pts_for_icp,'.');

end
