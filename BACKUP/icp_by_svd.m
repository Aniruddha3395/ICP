clc;
clear;
close all;
set(0, 'DefaultFigureRenderer', 'opengl');
run CONFIG.m

%%%%%%%%%%%%%%%%%%% icp with kdtree-knnsearch and svd %%%%%%%%%%%%%%%%%%%

Error_val = [Inf, Inf, Inf];
Error_val_prev = [Inf, Inf, Inf];
threshold = 1;
start_scan_ptcloud = scan_ptcloud;

%make KDTree
KDtree = KDTreeSearcher(model_ptcloud);
Transform_mat_new = eye(4);
Transform_mat_n = eye(4);
% while Error_val>threshold
for i = 1:100
%     cla;
%     scatter3d(model_ptcloud,'.b');
%     hold on;
%     scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
%     hold on;
%     quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
%     hold on;
%     view(-67,49);
%     scatter3(start_scan_ptcloud(:,1),start_scan_ptcloud(:,2),start_scan_ptcloud(:,3),'.r')
    
    idx = knnsearch(KDtree,scan_ptcloud,'K',1);
    corresponding_val_from_model_ptcloud = model_ptcloud(idx,:);
    dis = dist(corresponding_val_from_model_ptcloud,scan_ptcloud);
    
    %get transformation matrix
    Transform_mat = SVD_scan_T_model(corresponding_val_from_model_ptcloud,scan_ptcloud);
    Transform_mat_new = Transform_mat*Transform_mat_new;
    
    scan_ptcloud_transformed = apply_transformation(scan_ptcloud,inv(Transform_mat));
    s0_transformed = apply_transformation(s0,inv(Transform_mat));
    s0 = s0_transformed;
    Transform_mat_n(1:3,1:3) = Transform_mat(1:3,1:3);
    nx_transformed = apply_transformation(nx,inv(Transform_mat_n));
    nx = nx_transformed;
    ny_transformed = apply_transformation(ny,inv(Transform_mat_n));
    ny = ny_transformed;
    nz_transformed = apply_transformation(nz,inv(Transform_mat_n));
    nz = nz_transformed;
    
%     hold on;
%     scatter3d(scan_ptcloud_transformed,'.g');
%     hold on;
%     scatter3(s0_transformed(1),s0_transformed(2),s0_transformed(3),100,'d','filled','r');
%     hold on;
%     quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nx_transformed(1),nx_transformed(2),nx_transformed(3),100,'r');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),ny_transformed(1),ny_transformed(2),ny_transformed(3),100,'g');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nz_transformed(1),nz_transformed(2),nz_transformed(3),100,'b');
    scan_ptcloud = scan_ptcloud_transformed;
%     pause(0.001);
    disp(i);
end

aa = figure;
cla;
scatter3d(model_ptcloud,'.b')
hold on;
scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
hold on;
quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
hold on;
scatter3d(scan_ptcloud_transformed,'.g');
hold on;
scatter3(s0_transformed(1),s0_transformed(2),s0_transformed(3),100,'d','filled','r');
hold on;
quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nx_transformed(1),nx_transformed(2),nx_transformed(3),100,'r');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),ny_transformed(1),ny_transformed(2),ny_transformed(3),100,'g');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nz_transformed(1),nz_transformed(2),nz_transformed(3),100,'b');
hold on;
scatter3d(start_scan_ptcloud,'.r');

% Transform_mat_new


