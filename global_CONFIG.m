global tool_F_T_tcp data_file_dir icp_dir;
%%%%%%%%%%%%%%%%%
% Tool frame transformation of TCP wrt Flange (T_tcp_wrt_F) tool
% transformation is zero when pts wrt flange
%%%%%%%%%%%%%%%%%
tool_t = [1.7;0.7;45.6];                         % in mm
tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
tool_F_T_tcp = [tool_r,tool_t;0 0 0 1];
icp_dir = '/home/rflin/Desktop/';
data_file_dir = 'ICP_new/data_files/data13/';
stl_file = 'ICP_new/STL_CAD/fender_1106.stl';
True_w_T_p = [
    0.9999    0.0126         0    518.4;
   -0.0126    0.9999         0   -160.8;
         0         0    1.0000    314.3;
         0         0         0    1.0000
    ];
%%%%%%%%%%%%%%%%%
generate_model_data = true;
generate_scan_traj_data = true;
visualize_data = true;
if generate_model_data
    [part_ptcloud,part_ptcloud_normals] = generate_part_ptcloud_from_part_STL(...
        strcat(icp_dir,stl_file));
else
    part_ptcloud = dlmread(strcat(icp_dir,data_file_dir,'part_ptcloud.csv'));
    part_ptcloud_normals = dlmread(strcat(icp_dir,data_file_dir,'part_ptcloud_normals.csv'));
end
if generate_scan_traj_data
    scan_traj = get_traj_wrt_tcp(strcat(icp_dir,data_file_dir,'data_for_ICP.csv'));
else
    scan_traj = dlmread(strcat(icp_dir,data_file_dir,'scanned_traj.csv'));
end
if visualize_data
    figure;
    scatter3d(part_ptcloud,'.b');
    hold on;
    scatter3d(scan_traj,'.g');
end


%for ge blade
% Seed_t = [691.93;-171.70;319.70];                      % in mm
% Seed_r = eul2rotm([pi/2 0 0]);   

% for composite_mold
% Seed_t = [315.13;-307.71;116.72];                      % in mm
% Seed_r = eul2rotm([0 0 0]);     

% for fender
% Seed_t = [583.88;346.25;2.74];                      % in mm
% Seed_r = eul2rotm([pi/2 0 0]); 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        DATA         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% data1 %%%%%
% tool_t = [-0.2;-0.01;48.6];                    % in mm
% tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
% tool_F_T_tcp = [tool_r,tool_t;0 0 0 1];
% icp_dir = 'C:/Users/ABB IRB120/Desktop/';
% data_file_dir = 'data_files/data1/';
% stl_file = 'ICP_new/STL_CAD/car_bonnet_1031_new.stl';
% True_w_T_p = [
%     1.0000   -0.0001    0.0002  477.4196;
%     0.0001    1.0000    0.0014 -143.6004;
%     -0.0003   -0.0014    1.0000  311.3996;
%           0         0         0    1.0000
%     ];


%%%%%% data2 %%%%%
% tool_t = [1.7;0.7;45.6];                       % in mm
% tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
% tool_F_T_tcp = [tool_r,tool_t;0 0 0 1];
% icp_dir = 'C:/Users/ABB IRB120/Desktop/';
% data_file_dir = 'data_files/data2/';
% stl_file = 'ICP_new/STL_CAD/car_bonnet_1031_new.stl';
% True_w_T_p = [
%     0.9999    0.0126         0    518.4;
%    -0.0126    0.9999         0   -160.8;
%          0         0    1.0000    314.3;
%          0         0         0    1.0000
%     ];

%%%%%% data3 %%%%%
% tool_t = [1.7;0.7;45.6];                         % in mm
% tool_r = eul2rotm([0 0 0]);                    % in rad (alpha beta gamma rot)
% tool_F_T_tcp = [tool_r,tool_t;0 0 0 1];
% icp_dir = 'C:/Users/ABB IRB120/Desktop/';
% data_file_dir = 'ICP_new/data_files/data3/';
% stl_file = 'ICP_new/STL_CAD/car_bonnet_1031_new.stl';
% True_w_T_p = [
%     0.9999    0.0126         0    518.4;
%    -0.0126    0.9999         0   -160.8;
%          0         0    1.0000    314.3;
%          0         0         0    1.0000
%     ];