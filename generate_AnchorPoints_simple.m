%%%%%%%%%%%%%% GUI for Selection of Points %%%%%%%%%%%%%%%%%%%

function generate_AnchorPoints_simple(fig)

global part_pts scan_pts pt_idx flag text_arr_p text_arr_s scatter_arr_p scatter_arr_s;

% Define object for Data Cursor
dcm_obj = datacursormode(fig);
set(dcm_obj,'SnapToDataVertex','off')
part_pts = [];
scan_pts = [];
text_arr_p = [];
text_arr_s = [];
scatter_arr_p = [];
scatter_arr_s = [];
flag = 0;
pt_idx = 1;

c = uicontrol(fig,'Style','pushbutton');
c.Position = [20 75 80 20];
c.String = 'Clear All Data';
c.Callback = @ClearCompleteData;

% Keep model and scan points alternatively, starting with model points.
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
    text_p = text(part_pt(1),part_pt(2),part_pt(3),num2str(pt_idx),'FontSize',14);
    text_arr_p = [text_arr_p;text_p];
    part_pts = [part_pts;part_pt];
    
    %Plot the Points
    if ~isempty(part_pts)
        scatter_p = scatter3(part_pts(:,1),part_pts(:,2),part_pts(:,3),'filled','k'); %Plot the Points
        scatter_arr_p = [scatter_arr_p;scatter_p];
    end
    
    
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
    text_s = text(scan_pt(1),scan_pt(2),scan_pt(3),num2str(pt_idx),'FontSize',14);
    text_arr_s = [text_arr_s;text_s];
    scan_pts = [scan_pts;scan_pt];
    
    %Plot the Points
    if ~isempty(scan_pts)
        scatter_s = scatter3(scan_pts(:,1),scan_pts(:,2),scan_pts(:,3),'filled','k'); %Plot the Points
        scatter_arr_s = [scatter_arr_s;scatter_s];
    end
    pt_idx = pt_idx+1;
end

if size(part_pts,1)~=size(scan_pts,1)
    disp('one to one unique correspondance not possible...check number of points in both matrices');
    return;
end

end


function ClearCompleteData(source,event)
global part_pts scan_pts pt_idx flag text_arr_p text_arr_s scatter_arr_p scatter_arr_s;
part_pts = [];
scan_pts = [];
flag = 0;
pt_idx = 1;
delete(text_arr_p);
delete(text_arr_s);
delete(scatter_arr_p);
delete(scatter_arr_s);

end
