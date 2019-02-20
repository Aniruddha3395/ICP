clc
clear;
close all;
set(0, 'DefaultFigureRenderer', 'opengl');

%part ptcloud - blue
%start ptcloud - red
%final ptcloud - green

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
view_data = false;
%%%%%%%%

global part_pts scan_pts num_iter;
part_pts = [];
scan_pts = [];
num_iter = 300;

model_ptcloud_data = 'Blade_ptcloud.csv';
scan_ptcloud_data = 'BladeScan1.csv';

if view_data
    model_ptcloud = dlmread(model_ptcloud_data);
    scan_ptcloud = dlmread(scan_ptcloud_data);
    m0 = [0,0,0];
    s0 = [0,0,0];
    nx = [1,0,0];ny = [0,1,0];nz = [0,0,1];
    scatter3d(model_ptcloud,'.b');
    hold on;
    scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
    hold on;
    quiver3(m0(1),m0(2),m0(3),1,0,0,200,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,200,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,200,'b');
    hold on;
    scatter3d(scan_ptcloud,'.r');
    hold on;
    scatter3(s0(1),s0(2),s0(3),100,'d','filled','r');
    hold on;
    quiver3(s0(1),s0(2),s0(3),nx(1),nx(2),nx(3),200,'r');hold on;quiver3(s0(1),s0(2),s0(3),ny(1),ny(2),ny(3),200,'g');hold on;quiver3(s0(1),s0(2),s0(3),nz(1),nz(2),nz(3),200,'b');
end

Transform_mat_new = ICP_SVD_with_AnchorPoints(model_ptcloud_data,scan_ptcloud_data)

