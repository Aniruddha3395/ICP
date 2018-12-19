%%%%%%%%%%%%%% GUI for Selection of Points %%%%%%%%%%%%%%%%%%%

function fun_flag = generate_AnchorPoints(fig)

global part_pts;
global scan_pts;
global pt_idx_p;
global pt_idx_s;
global flag;
global text_arr_p;
global text_arr_s;
global scatter_arr_p;
global scatter_arr_s;
global ptr;
global disp_p_pts;
global disp_s_pts;
global select_data;
global helpbx;

% Define object for Data Cursor
dcm_obj = datacursormode(fig);
set(dcm_obj,'SnapToDataVertex','off');
set(gcf,'color','w');
part_pts = [];
scan_pts = [];
text_arr_p = [];
text_arr_s = [];
scatter_arr_p = [];
scatter_arr_s = [];
flag = 0;
pt_idx_p = 1;
pt_idx_s = 1;
ptr = 0;

%%%% command messages %%%%
helpbx = uicontrol(fig,'Style','text');
helpbx.Position = [600 980 500 20];
helpbx.FontWeight = 'bold';
helpbx.FontSize = 12;
helpbx.String = '...Anchorpoints Selection (Start with Part Pointcloud)...';
start_pt_reached = 0;

%%%%% clear all button %%%%
cldata = uicontrol(fig,'Style','pushbutton');
cldata.Position = [1500 100 150 50];
cldata.String = 'Clear All Data';
cldata.FontWeight = 'bold';
cldata.BackgroundColor = [0.8,1,0.7];
cldata.ForegroundColor = [0.1,0,0.3];
cldata.FontSize = 12;
cldata.Callback = @ClearCompleteData;

%%%%% undo button %%%%
clldata = uicontrol(fig,'Style','pushbutton');
clldata.Position = [1500 160 150 50];
clldata.String = 'Undo Selection';
clldata.FontWeight = 'bold';
clldata.BackgroundColor = [1,0.8,0.8];
clldata.ForegroundColor = [0.1,0,0.3];
clldata.FontSize = 12;
clldata.Callback = @ClearLastAnchorPoint;

%%%%% done button %%%%
close_win = uicontrol(fig,'Style','pushbutton');
close_win.Position = [1500 40 150 50];
close_win.String = 'DONE';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;

%%%% displaying points on the screen %%%%
disp_p_pts_h = uicontrol(fig,'Style','text');
disp_p_pts_h.Position = [20 975 250 20];
disp_p_pts_h.FontWeight = 'bold';
disp_p_pts_h.ForegroundColor = [0.6,0,0];
disp_p_pts_h.String = 'Anchor Points on Part Pointcloud';

disp_p_ax = uicontrol(fig,'Style','text');
disp_p_ax.Position = [20 955 250 20];
disp_p_ax.FontWeight = 'bold';
disp_p_ax.String = 'X               Y                Z';

disp_p_pts = uicontrol(fig,'Style','text');
disp_p_pts.Position = [20 755 250 200];
disp_p_pts.FontWeight = 'bold';
disp_p_pts.String = num2str(part_pts);

disp_s_pts_h = uicontrol(fig,'Style','text');
disp_s_pts_h.Position = [20 260 250 20];
disp_s_pts_h.FontWeight = 'bold';
disp_s_pts_h.ForegroundColor = [0.6,0,0];
disp_s_pts_h.String = 'Anchor Points on Scan Pointcloud';

disp_s_ax = uicontrol(fig,'Style','text');
disp_s_ax.Position = [20 240 250 20];
disp_s_ax.FontWeight = 'bold';
disp_s_ax.String = 'X               Y                Z';

disp_s_pts = uicontrol(fig,'Style','text');
disp_s_pts.Position = [20 40 250 200];
disp_s_pts.FontWeight = 'bold';
disp_s_pts.String = num2str(scan_pts);

select_data=0;
% Keep model and scan points alternatively, starting with model points.
while 1
    key=0;
    fprintf('Select Anchor Point');
    if start_pt_reached==1
    helpbx.String = 'Select Anchor Point';
    end
    start_pt_reached = 1;
    
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
    if select_data/2==round(select_data/2)
        part_pt = c_info.Position;
        text_p = text(part_pt(1),part_pt(2),part_pt(3),num2str(pt_idx_p),'FontSize',14);
        text_arr_p = [text_arr_p;text_p];
        part_pts = [part_pts;part_pt];
        ptr = 1;
        pt_idx_p = pt_idx_p+1;
        disp_p_pts.String = num2str(part_pts);
        %Plot the Points
        if ~isempty(part_pts)
            scatter_p = scatter3(part_pts(:,1),part_pts(:,2),part_pts(:,3),'filled','k'); %Plot the Points
            scatter_arr_p = [scatter_arr_p;scatter_p];
        end
        select_data=select_data+1;
    else
        scan_pt = c_info.Position;
        text_s = text(scan_pt(1),scan_pt(2),scan_pt(3),num2str(pt_idx_s),'FontSize',14);
        text_arr_s = [text_arr_s;text_s];
        scan_pts = [scan_pts;scan_pt];
        ptr = 2;
        %Plot the Points
        if ~isempty(scan_pts)
            scatter_s = scatter3(scan_pts(:,1),scan_pts(:,2),scan_pts(:,3),'filled','k'); %Plot the Points
            scatter_arr_s = [scatter_arr_s;scatter_s];
        end
        pt_idx_s = pt_idx_s+1;
        disp_s_pts.String = num2str(scan_pts);
        select_data=select_data+1;
    end
end

if size(part_pts,1)~=size(scan_pts,1)
    disp('One to one unique correspondance not possible...check number of points in both matrices');
    fun_flag = false;
else
    fun_flag=true;
end

end


