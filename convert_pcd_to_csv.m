clc;
clear;
close all;

pathdir = '/home/aniruddha/Desktop/ICP_MATLAB/expt/data_files/Partial Point Clouds/';
filename= 'DomeSegmented';

for num = 0:10
    pcdata = pcread(strcat(pathdir,filename,num2str(num),'.pcd'));
    pts = zeros(pcdata.Count,3);
    
    for i=1:pcdata.Count
        pts(i,:) = pcdata.Location(i,:);
    end
    
    % scatter3d(pts,'.');
    pts = pts.*1000;
    dlmwrite(strcat(pathdir,filename,num2str(num),'.csv'),pts);
end