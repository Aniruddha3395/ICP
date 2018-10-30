clc;
clear;
close all;
dbstop if error;
set(0, 'DefaultFigureRenderer', 'opengl');

global Seed_T Complete_T KDtree;
global tcp_publisher_pts_are_flange_pts;
global fval_chk;

run declarations.m

KDtree = KDTreeSearcher(model_ptcloud);


%% decision of selcting the sedd after pertubation....error metric is fval jump
small_val = 0.1;
big_val = 5;
fval_curr = Inf;
fval_chk = 1000;
while fval_curr>1
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

% if fval_new<fval_curr
    fval_curr = fval_new;
    perturb_val_t = -small_val + 2*small_val*rand(3,1);
    Seed_t_new = Seed_t + perturb_val_t;
    perturb_val_r = [-0.5*small_val + small_val*rand(1,3)].*(pi/180);
    Seed_r_new = eul2rotm([rotm2eul(Seed_r) + perturb_val_r]);
    Seed_T = [Seed_r_new,Seed_t_new;0 0 0 1];
    
%     fval_curr = fval_new;
%     perturb_val_t = small_val + 2*small_val*rand(3,1);
%     Seed_t = Final_T(1:3,4) + perturb_val_t;
%     perturb_val_r = [-0.5*small_val + small_val*rand(1,3)].*(pi/180);
%     Seed_r = eul2rotm([rotm2eul(Final_T(1:3,1:3)) + perturb_val_r]);
%     Seed_T = [Seed_r,Seed_t;0 0 0 1];

%     else
%     perturb_val_t = -big_val + 2*big_val*rand(3,1);
%     Seed_t_new = Seed_t + perturb_val_t;
%     perturb_val_r = [-0.5*big_val + big_val*rand(1,3)].*(pi/180);
%     Seed_r_new = eul2rotm([rotm2eul(Seed_r) + perturb_val_r]);
%     Seed_T = [Seed_r_new,Seed_t_new;0 0 0 1];
% end


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