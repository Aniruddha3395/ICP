clc;
clear;
close all;
set(0, 'DefaultFigureRenderer', 'opengl');

% part_ptcloud = dlmread('BikeFender_ptcloud.csv');
% scan_ptcloud = dlmread('BikeFenderSegmented0.csv');

part_ptcloud = dlmread('HeliBlade_ptcloud.csv');
scan_ptcloud = dlmread('HeliBladeSegmented3.csv');
m0 = [0,0,0];
s0 = [0,0,0];
nx = [1,0,0];ny = [0,1,0];nz = [0,0,1];

fig1 = figure;
scatter3d(part_ptcloud,'.b');
hold on;
scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
hold on;
quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
hold on;
scatter3d(scan_ptcloud,'.r');
hold on;

%% User-Interface for Selection of Points

% Define object for Data Cursor
dcm_obj = datacursormode(fig1)
set(dcm_obj,'SnapToDataVertex','off')
part_pts = [];
scan_pts = [];
key=0;
flag = 0;
idx = 1;

% Keep Selecting Start and End points alternatively.
while 1
    key=0;
    fprintf('Select Point on Part Pointcloud')
    while key==0
        try 
            key = waitforbuttonpress; 
        catch
            flag=1; 
            break; 
        end
    end
    if flag==1
        fprintf('\nSelection Complete\n');
        break;
    end
    c_info = getCursorInfo(dcm_obj);
    part_pt = c_info.Position;
    text(part_pt(1),part_pt(2),part_pt(3),num2str(idx),'FontSize',14);
    part_pts = [part_pts;part_pt];
    
    key=0;
    fprintf('Select Corresponding Point on Scanned Pointcloud')
    while key==0
        try 
            key = waitforbuttonpress; 
        catch
            flag=1;
            break; 
        end
    end
    if flag==1
        fprintf('\nSelection Complete\n');
        break;
    end
    c_info = getCursorInfo(dcm_obj);
    scan_pt = c_info.Position;
    text(scan_pt(1),scan_pt(2),scan_pt(3),num2str(idx),'FontSize',14);
    scan_pts = [scan_pts;scan_pt];
    
    %Plot the Points
    scatter3d(part_pts,'*g'); %Plot the Points
    scatter3d(scan_pts,'*k'); %Plot the Points
    idx = idx+1;
end

if size(part_pts,1)~=size(scan_pts,1)
    disp('one to one unique correspondance not possible...check number of points in both matrices');
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_init = eye(4);
T_init_n = eye(4);
T_init = SVD_scan_T_model(part_pts,scan_pts);
scan_ptcloud_transformed = [inv(T_init)*[scan_ptcloud';ones(1,size(scan_ptcloud,1))]]';
scan_ptcloud_transformed = scan_ptcloud_transformed(:,1:3);
s0_transformed = [inv(T_init)*[s0';ones(1,size(s0,1))]]';
s0_transformed = s0_transformed(:,1:3);
T_init_n(1:3,1:3) = T_init(1:3,1:3);
nx_transformed = [inv(T_init_n)*[nx';ones(1,size(nx,1))]]';
nx_transformed = nx_transformed(:,1:3);
nx = nx_transformed;
ny_transformed = [inv(T_init_n)*[ny';ones(1,size(ny,1))]]';
ny_transformed = ny_transformed(:,1:3);
ny = ny_transformed;
nz_transformed = [inv(T_init_n)*[nz';ones(1,size(nz,1))]]';
nz_transformed = nz_transformed(:,1:3);
nz = nz_transformed;

figure;
scatter3d(part_ptcloud,'.b');
hold on;
scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
hold on;
quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
hold on;
scatter3d(scan_ptcloud,'.r');
hold on;
scatter3d(scan_ptcloud_transformed,'.g');
hold on;
scatter3(s0_transformed(1),s0_transformed(2),s0_transformed(3),100,'d','filled','r');
hold on;
quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nx_transformed(1),nx_transformed(2),nx_transformed(3),100,'r');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),ny_transformed(1),ny_transformed(2),ny_transformed(3),100,'g');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nz_transformed(1),nz_transformed(2),nz_transformed(3),100,'b');
s0 = s0_transformed;


scan_ptcloud = scan_ptcloud_transformed;
model_ptcloud = part_ptcloud;
run icp_by_svd.m


