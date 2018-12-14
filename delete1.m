clc;
clear;
close all;

for ffnum=1:3
    switch ffnum
        case 1
            partname = 'BikeFender';
            scanr = 6;
        case 2
            partname = 'Dome';
            scanr = 10;
        case 3
            partname = 'HeliBlade';
            scanr = 17;
    end
    
    model_ptcloud_data = strcat(partname,'_ptcloud.csv');
    for scanr_counter = 0:scanr
        scan_ptcloud_data = strcat(partname,'Segmented',num2str(scanr_counter),'.csv');
        run CONFIG.m
        run icp_by_svd.m
%         savefig(aa,strcat('fig',num2str(ffnum),'_',num2str(scanr_counter),'.fig'));
        fprintf('fig %d - %d generated\n\n',ffnum,scanr_counter);
    end
    
end