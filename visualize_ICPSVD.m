function visualize_ICPSVD(model_ptcloud,m0,scan_ptcloud_transformed,s0_transformed,start_scan_ptcloud,nx_transformed,ny_transformed,nz_transformed)

scatter3d(model_ptcloud,'.b');
hold on;
scatter3(m0(1),m0(2),m0(3),100,'d','filled','b');
hold on;
quiver3(m0(1),m0(2),m0(3),1,0,0,100,'r');hold on;quiver3(m0(1),m0(2),m0(3),0,1,0,100,'g');hold on;quiver3(m0(1),m0(2),m0(3),0,0,1,100,'b');
hold on;
scatter3d(scan_ptcloud_transformed,'.g');
hold on;
scatter3(s0_transformed(1),s0_transformed(2),s0_transformed(3),100,'d','filled','r');
hold on;
quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nx_transformed(1),nx_transformed(2),nx_transformed(3),100,'r');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),ny_transformed(1),ny_transformed(2),ny_transformed(3),100,'g');hold on;quiver3(s0_transformed(1),s0_transformed(2),s0_transformed(3),nz_transformed(1),nz_transformed(2),nz_transformed(3),100,'b');
hold on;
scatter3d(start_scan_ptcloud,'.r');
end