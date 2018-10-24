%% optimizer function: 
% Optimize with Fmincon

function [transformation_matrix] = optimize_with_fmincon(KDtree,model_ptcloud,scan_ptcloud,tx,ty,tz,q0,q1,q2,q3,optm_method,error_fun)

x = [tx,ty,tz,q0,q1,q2,q3];
x0 = [0 0 0 1 0 0 0];

fun = @(x)error_function(x,KDtree,model_ptcloud,scan_ptcloud,optm_method,error_fun);
nonlcon = @inq_constaints;

options = optimoptions(@fmincon,'Display','iter');%,'Algorithm','interior-point','DiffMaxChange',1e-8,...
%     'FunctionTolerance',1e-8,'StepTolerance',1e-10);
[x,fval] = fmincon(fun,x0,[],[],[],[],[],[],nonlcon,options)

rotation_mat = quat2rotm([x(4) x(5) x(6) x(7)]); 
translation_mat = [x(1);x(2);x(3)];

transformation_matrix = [rotation_mat,translation_mat;0 0 0 1];
transformed_data = apply_transformation(scan_ptcloud,transformation_matrix);

hold on;
scatter3d(transformed_data,'*');

function [c,ceq] = inq_constaints(x)
    c = 0;
    ceq = (x(4)^2) + (x(5)^2) + (x(6)^2) + (x(7)^2) - 1;
end

end



% [scan_ptcloud,corresponding_val_from_model_ptcloud] = correspondance_with_near_neighbour(KDtree,model_ptcloud,scan_ptcloud);
% [Error_prev,scan_ptcloud_transformed] = error_function(scan_ptcloud,corresponding_val_from_model_ptcloud,tx,ty,tz,rx,ry,rz);

% d_tx = -1;
% d_ty = -1;
% d_tz = -1;
% d_rx = -1;
% d_ry = -1;
% d_rz = -1;
% tx = tx+d_tx;
% ty = ty+d_ty;
% tz = tz+d_tz;
% rx = rx+d_rx;
% ry = ry+d_ry;
% rz = rz+d_rz;


% for i = 1:100
%     cla;
%     scatter3d(model_ptcloud,'.');
%     hold on;
%     
%     
%     [scan_ptcloud,corresponding_val_from_model_ptcloud] = correspondance_with_near_neighbour(KDtree,model_ptcloud,scan_ptcloud);
%     [Error_new,scan_ptcloud_transformed] = error_function(scan_ptcloud,corresponding_val_from_model_ptcloud,tx,ty,tz,rx,ry,rz);
%     Error_new
%     if Error_new<Error_prev
%         scan_ptcloud = scan_ptcloud_transformed;
%         tx = tx+d_ty;
%     end
%     
%     hold on;
%     scatter3d(scan_ptcloud_transformed,'.');
%     pause(0.001);
% end

