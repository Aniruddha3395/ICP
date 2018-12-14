%%%%%%%%%%%%%%%%%%% icp with kdtree-knnsearch and svd %%%%%%%%%%%%%%%%%%%
% INPUT : model pointcloud file, scan pointcloud file
% OUTPUT : trnasformation matrix

%part ptcloud - blue
%start ptcloud - red
%final ptcloud - green

function Transform_mat_new = ICP_SVD(model_ptcloud_file,scan_ptcloud_file)

set(0, 'DefaultFigureRenderer', 'opengl');
model_ptcloud = dlmread(model_ptcloud_file);
scan_ptcloud = dlmread(scan_ptcloud_file);
start_scan_ptcloud = scan_ptcloud;
m0 = [0,0,0];
s0 = [0,0,0];
nx = [1,0,0];ny = [0,1,0];nz = [0,0,1];

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
