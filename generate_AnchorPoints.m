%%%%%%%%%%%%%% GUI for Selection of Points %%%%%%%%%%%%%%%%%%%

function [part_pts,scan_pts] = generate_AnchorPoints(fig)

% Define object for Data Cursor
dcm_obj = datacursormode(fig)
set(dcm_obj,'SnapToDataVertex','off')
part_pts = [];
scan_pts = [];
key=0;
flag = 0;
idx = 1;

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

end