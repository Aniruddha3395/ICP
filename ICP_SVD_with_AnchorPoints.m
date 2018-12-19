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

set(0, 'DefaultFigureRenderer', 'opengl');
part_ptcloud = dlmread(model_ptcloud_file);
scan_ptcloud = dlmread(scan_ptcloud_file);
start_scan_ptcloud = scan_ptcloud;
m0 = [0,0,0];
s0 = [0,0,0];
nx = [1,0,0];ny = [0,1,0];nz = [0,0,1];

fun_flag = false;
while ~fun_flag
fig1 = figure;
set(fig1,'units','normalized','outerpos',[0 0 1 1.2]);
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

%%%%%%%%%%%%%% initial transformation %%%%%%%%%%%%%%%%%%%%
T_init = eye(4);
T_init_n = eye(4);
if ~isempty(part_pts) && ~isempty(scan_pts)
    T_init = SVD_scan_to_model(part_pts,scan_pts);
end
scan_ptcloud_transformed = apply_transformation(scan_ptcloud,inv(T_init));
s0_transformed = apply_transformation(s0,inv(T_init));
s0 = s0_transformed;
T_init_n(1:3,1:3) = T_init(1:3,1:3);
nx_transformed = apply_transformation(nx,inv(T_init_n));
nx = nx_transformed;
ny_transformed = apply_transformation(ny,inv(T_init_n));
ny = ny_transformed;
nz_transformed = apply_transformation(nz,inv(T_init_n));
nz = nz_transformed;

%%%%%%%% visualise data %%%%%%%%%%
% figure;
% visualize_ICPSVD(part_ptcloud,m0,scan_ptcloud_transformed,s0_transformed,...
%     start_scan_ptcloud,nx_transformed,ny_transformed,nz_transformed);

%%%%%%%%%%%%%%%%%%%%%%%% icp_svd starts here %%%%%%%%%%%%%%%%%%%%%%%%%

scan_ptcloud = scan_ptcloud_transformed;
model_ptcloud = part_ptcloud;

%make KDTree
KDtree = KDTreeSearcher(model_ptcloud);
Transform_mat_new = eye(4);
Transform_mat_n = eye(4);

for i = 1:100
    idx = knnsearch(KDtree,scan_ptcloud,'K',1);
    corresponding_val_from_model_ptcloud = model_ptcloud(idx,:);
    dis = dist(corresponding_val_from_model_ptcloud,scan_ptcloud);
    
    %get transformation matrix
    Transform_mat = SVD_scan_to_model(corresponding_val_from_model_ptcloud,scan_ptcloud);
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
    
    %%%%%% visualize data in loop %%%%%%
    %     cla;
    %     visualize_ICPSVD(model_ptcloud,m0,scan_ptcloud_transformed,s0_transformed,...
    %     start_scan_ptcloud,nx_transformed,ny_transformed,nz_transformed);
    %     pause(0.001);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    scan_ptcloud = scan_ptcloud_transformed;
end

% visualize data
figure;
visualize_ICPSVD(model_ptcloud,m0,scan_ptcloud_transformed,s0_transformed,...
    start_scan_ptcloud,nx_transformed,ny_transformed,nz_transformed);
end

