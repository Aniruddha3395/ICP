%%%%%%%%%% icp with anchor points correspondance matching %%%%%%%%%%%%%
%%%%%%%%%%%%%% followed by kdtree-knnsearch and svd %%%%%%%%%%%%%%%%%%%
% INPUT : model pointcloud file, scan pointcloud file
% OUTPUT : trnasformation matrix

%part ptcloud - blue
%start ptcloud - red
%final ptcloud - green

function Transform_mat_new = ICP_SVD_with_AnchorPoints(model_ptcloud_file,scan_ptcloud_file)

disp('...Anchorpoints Selection (Start with Part Pointcloud)...');
fprintf('\n\n');
global part_pts scan_pts;

part_ptcloud = dlmread(model_ptcloud_file);
scan_ptcloud = dlmread(scan_ptcloud_file);
m0 = [0,0,0];
s0 = [0,0,0];
nx = [1,0,0];ny = [0,1,0];nz = [0,0,1];

fun_flag = false;
while ~fun_flag
fig1 = figure();
set(fig1,'units','normalized','outerpos',[0 0 1 1.2]);
axis equal;
addToolbarExplorationButtons(fig1);
pause(0.1);
scatter3d(part_ptcloud,'.b');
hold on;
scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
hold on;
quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
hold on;
scatter3d(scan_ptcloud,'.r');
hold on;

fun_flag = generate_AnchorPoints(fig1);
end

Transform_mat_new = ICP_SVD_with_AnchorPoints_mex(part_pts,scan_pts,part_ptcloud,scan_ptcloud);
scan_ptcloud_transformed = apply_transformation(scan_ptcloud,inv(Transform_mat_new));
s0_transformed = apply_transformation(s0,inv(Transform_mat_new));
Transform_mat_n = eye(4);
Transform_mat_n(1:3,1:3) = Transform_mat_new(1:3,1:3);
nx_transformed = apply_transformation(nx,inv(Transform_mat_n));
nx = nx_transformed;
ny_transformed = apply_transformation(ny,inv(Transform_mat_n));
ny = ny_transformed;
nz_transformed = apply_transformation(nz,inv(Transform_mat_n));
nz = nz_transformed;

% visualize data
figure;
visualize_ICPSVD(part_ptcloud,m0,scan_ptcloud_transformed,s0_transformed,...
    scan_ptcloud,nx_transformed,ny_transformed,nz_transformed);
end

