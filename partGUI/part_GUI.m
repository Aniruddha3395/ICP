clear all; close all;
% transformation of part wrt robot

global w_T_part;
global part_position;
global part_orientation;
global original_v;
global original_f;
global original_n;
global original_name;
global raw_scan_pts_wrt_tcp;

w_T_part = eye(4);

% part_position = [0,0,0];
% part_orientation = [0 0 0]; % euler angles. matlab's 'ZYX', real life 'XYZ'



part_position = [0.4775,-0.1436,0.3114];
part_orientation = [0 0 -0.0013]; % euler angles. matlab's 'ZYX', real life 'XYZ'

w_T_part(1:3,1:3) = eul2rotm(part_orientation);
w_T_part(1:3,4) = part_position';

[v, f, n, name] = stlRead('car_bonnet_1031_new.stl');
original_v = v./1000;     %values in mm
original_f = f;
original_n = n;
original_name = name;
[v,n] = stlTransform(original_v,original_n,w_T_part);
stlPlot(v, original_f, original_name, [0.5 0.5 0.5]);
% axis([-1 1 -1 1 -1 1])
daspect([1 1 1])
xlabel('x');ylabel('y');zlabel('z');

hold on;
raw_scan_pts_wrt_tcp = dlmread('scanned_traj.csv');
raw_scan_pts_wrt_tcp = raw_scan_pts_wrt_tcp./1000;
scatter3d(raw_scan_pts_wrt_tcp,'.');


% Create slider
% text(1500 850,'Translate x');

sld1 = uicontrol('Style', 'slider',...
    'String', 'Translate x: ',...
    'Min',-1,'Max',1,'Value',part_position(1),...
    'Position', [1500 800 200 20],...
    'Callback', @part_plot_x,'Sliderstep',[1/50000 1/50000]);

sld2 = uicontrol('Style', 'slider',...
    'String', 'Translate y',...
    'Min',-1,'Max',1,'Value',part_position(2),...
    'Position', [1500 750 200 20],...
    'Callback', @part_plot_y,'Sliderstep',[1/50000 1/50000]);

sld3 = uicontrol('Style', 'slider',...
    'String', 'Translate z',...
    'Min',-1,'Max',1,'Value',part_position(3),...
    'Position', [1500 700 200 20],...
    'Callback', @part_plot_z,'Sliderstep',[1/50000 1/50000]);

sld4 = uicontrol('Style', 'slider',...
    'String', 'Rotate z',...
    'Min',-pi,'Max',pi,'Value',part_orientation(1),...
    'Position', [1500 650 200 20],...
    'Callback', @part_plot_Alpha,'Sliderstep',[1/50000 1/50000]);

sld5 = uicontrol('Style', 'slider',...
    'String', 'Rotate y',...
    'Min',-pi,'Max',pi,'Value',part_orientation(2),...
    'Position', [1500 600 200 20],...
    'Callback', @part_plot_Beta,'Sliderstep',[1/50000 1/50000]);

sld6 = uicontrol('Style', 'slider',...
    'String', 'Rotate x',...
    'Min',-pi,'Max',pi,'Value',part_orientation(3),...
    'Position', [1500 550 200 20],...
    'Callback', @part_plot_Gamma,'Sliderstep',[1/50000 1/50000]);


