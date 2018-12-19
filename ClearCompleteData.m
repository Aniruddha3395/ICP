% GUI Function
function ClearCompleteData(~,~)
global part_pts; 
global scan_pts; 
global pt_idx_p; 
global pt_idx_s; 
global flag; 
global text_arr_p; 
global text_arr_s; 
global scatter_arr_p; 
global scatter_arr_s;
global disp_p_pts; 
global disp_s_pts; 
global select_data;
global helpbx;
part_pts = [];
scan_pts = [];
flag = 0;
pt_idx_p = 1;
pt_idx_s = 1;
delete(text_arr_p);
delete(text_arr_s);
delete(scatter_arr_p);
delete(scatter_arr_s);
disp_p_pts.String = num2str(part_pts);
disp_s_pts.String = num2str(scan_pts);
select_data=0;
clc;
disp('...Anchorpoints Selection...');
fprintf('\n\n');
fprintf('Select Anchor Point');
helpbx.String = 'All data cleared...Select Anchor Point';
end