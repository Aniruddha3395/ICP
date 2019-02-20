pathdir_plane = '/home/rflin/usc_sanding_system/MATLAB_CODES/ICP_SVD_GUI_ROS/DATA/';
pathdir_plydata = '/home/rflin/usc_sanding_system/src/robotic_sanding/planning_vision_system/data/';
generate_plane_pts_csv = true;

%% ABB_T_Kinect transformation
Flange_T_Kinect = [
    0.9792    0.0828    0.0629    8.3023;
    -0.0269   -0.0323    0.9811   25.9767;
    0.0814   -0.9626   -0.0313   48.0426;
    0         0         0    1.0000
    ];

Flange_T_Kinect = [ % 2019 jan 10
    0.9986   -0.0286    0.0435   45.4106
   -0.0425    0.0294    0.9987   19.4460
   -0.0297   -0.9991    0.0282  133.2360
         0         0         0    1.0000];
         
         
% NOTE : ABB_T_Flange will change ...make sure to add appropriate
% transformations form files or modify this
ABB_T_Flange = [
    -0.0061   -0.0612    0.9981    1143.75;
    -0.9981    0.0615   -0.0023    248.89;
    -0.0612   -0.9962   -0.0615    1513.48;
    0         0         0    1.0000
    ];
ABB_T_Kinect = ABB_T_Flange*Flange_T_Kinect;

%% PLY to csv conversion for PLANE
if generate_plane_pts_csv
    pcdata = pcread(strcat(pathdir_plane,'CapturedPLANE.ply'));
    pts = zeros(pcdata.Count,3);
    for i=1:pcdata.Count
        pts(i,:) = pcdata.Location(i,1:3);
    end
    pts = pts.*1000;
    pts_T = apply_transformation(pts,ABB_T_Kinect);
    pts_T = pts_T(1:3:end,:);
    pts_for_plane = [];
    for i=1:size(pts_T,1)
        if pts_T(i,3) > 0
            pts_for_plane = [pts_for_plane;pts_T(i,:)];
        end
    end
    dlmwrite(strcat(pathdir_plane,'table_plane.csv'),pts_for_plane);
else
    pts_for_plane = dlmread(strcat(pathdir_plane,'table_plane.csv'));
end

%% PLY to csv conversion for data from kinect
pcdata = pcread(strcat(pathdir_plydata,'CapturedPLY.ply'));
pts = zeros(pcdata.Count,3);
for i=1:pcdata.Count
    pts(i,:) = pcdata.Location(i,1:3);
end
pts = pts.*1000;
scan_pts = apply_transformation(pts,ABB_T_Kinect);

%% code for filtering pts
% filterring the table plane
z_filter_idx = find(pts_for_plane(:,3)>0);
plane_pts_z_filtered = pts_for_plane(z_filter_idx,:);
x_filter_idx_ll = find(plane_pts_z_filtered(:,1)>600);
plane_pts_xll_filtered = plane_pts_z_filtered(x_filter_idx_ll,:);
x_filter_idx = find(plane_pts_xll_filtered(:,1)<1400);
plane_pts_xz_filtered = plane_pts_xll_filtered(x_filter_idx,:);
y_filter_idx_ll = find(plane_pts_xz_filtered(:,2)>-200);
plane_pts_yll_filtered = plane_pts_xz_filtered(y_filter_idx_ll,:);
y_filter_idx = find(plane_pts_yll_filtered(:,2)<1000);
plane_pts_xyz_filtered = plane_pts_yll_filtered(y_filter_idx,:);

% fit the plane with linear square fitting
x_avg = sum(plane_pts_xyz_filtered(:,1))/size(plane_pts_xyz_filtered,1);
y_avg = sum(plane_pts_xyz_filtered(:,2))/size(plane_pts_xyz_filtered,1);
z_avg = sum(plane_pts_xyz_filtered(:,3))/size(plane_pts_xyz_filtered,1);
L00 = sum((plane_pts_xyz_filtered(:,1)-x_avg).^2);
L01 = sum((plane_pts_xyz_filtered(:,1)-x_avg).*(plane_pts_xyz_filtered(:,2)-y_avg));
L11 = sum((plane_pts_xyz_filtered(:,2)-y_avg).^2);
R0 = sum((plane_pts_xyz_filtered(:,3)-z_avg).*(plane_pts_xyz_filtered(:,1)-x_avg));
R1 = sum((plane_pts_xyz_filtered(:,3)-z_avg).*(plane_pts_xyz_filtered(:,2)-y_avg));
A = -((L11*R0-L01*R1)/(L00*L11-L01^2));
B = -((L00*R1-L01*R0)/(L00*L11-L01^2));
C = 1;
D = -(z_avg+A*x_avg+B*y_avg);

% filtering the component ptcloud
% primary filter with axes bounds
z_filter_idx_for_part = find(scan_pts(:,3)>0);
scan_pts_z_filtered = scan_pts(z_filter_idx_for_part,:);
y_filter_idx_for_part = find(scan_pts_z_filtered(:,2)<1200);
scan_pts_yz_filtered = scan_pts_z_filtered(y_filter_idx_for_part,:);
% y_filter_idx_for_part2 = find(scan_pts_yz_filtered(:,2)>0);
% scan_pts_yz_filtered = scan_pts_yz_filtered(y_filter_idx_for_part2,:);
x_filter_idx_for_part = find(scan_pts_yz_filtered(:,1)<1700);
scan_pts_yz_filtered = scan_pts_yz_filtered(x_filter_idx_for_part,:);
% y_filter_idx_for_part2 = find(scan_pts_yz_filtered(:,2)>100);
% scan_pts_yz_filtered = scan_pts_yz_filtered(y_filter_idx_for_part2,:);

% seconadry filtering with table plane
filtered_data =[];
for i=1:size(scan_pts_yz_filtered,1)
        dist_p_plane = abs(A*scan_pts_yz_filtered(i,1)+B*scan_pts_yz_filtered(i,2)...
        +C*scan_pts_yz_filtered(i,3)+D)/(sqrt(A^2+B^2+C^2));
    if dist_p_plane >35
        filtered_data = [filtered_data;scan_pts_yz_filtered(i,:)];
    end
end

% transforming ptcloud from ABB base to kuka base
ABB_T_KUKA = [
        -0.0357,    0.9993,    0.0065,    1.2064*1000.0;
        -0.9993,   -0.0357,   -0.0070,    1.1619*1000.0;
        -0.0068,   -0.0068,    1.0000,    0.1248*1000.0;
        0,         0,         0,    1.0000];

filtered_data = apply_transformation(filtered_data, inv(ABB_T_KUKA));    

% figure;
% scatter3d(filtered_data,'.');
dlmwrite(strcat(pathdir_plane,'scan_data.csv'),filtered_data);
