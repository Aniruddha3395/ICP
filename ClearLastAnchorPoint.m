% GUI Function
function ClearLastAnchorPoint(~,~)

global ptr; 
global part_pts; 
global scan_pts;
global text_arr_p; 
global text_arr_s; 
global scatter_arr_p;
global scatter_arr_s;
global pt_idx_p;
global pt_idx_s;
global disp_p_pts; 
global disp_s_pts 
global select_data;
global helpbx;

if ptr==0
    fprintf('\n There are no selected points\n');
elseif ptr==1
    if isempty(part_pts)
        fprintf('\n There are no points selected on the part ptcloud\n');
        helpbx.String = 'There are no points selected on the part ptcloud';
    else
        part_pts(end,:) = [];
        delete(scatter_arr_p(end));
        scatter_arr_p(end) = [];
        delete(text_arr_p(end));
        text_arr_p(end) = [];
        fprintf('\n Last point of part ptcloud deleted\n');
        helpbx.String = 'Last point of part ptcloud deleted';
        ptr=2;
        pt_idx_p = pt_idx_p-1;
        select_data = select_data-1;
    end
elseif ptr==2
    if isempty(scan_pts)
        fprintf('\n There are no points selected on the scan ptcloud\n');
        helpbx.String = 'There are no points selected on the scan ptcloud';
    else
        scan_pts(end,:) = [];
        delete(text_arr_s(end));
        text_arr_s(end) = [];
        delete(scatter_arr_s(end));
        scatter_arr_s(end) = [];
        fprintf('\n Last point of scan ptcloud deleted\n');
        helpbx.String = 'Last point of scan ptcloud deleted';
        ptr=1;
        pt_idx_s = pt_idx_s -1;
        select_data = select_data-1;
    end
end
disp_p_pts.String = num2str(part_pts);
disp_s_pts.String = num2str(scan_pts);

end