clc;
clear;
addpath("..");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools");
addpath("../tools/board_extraction");
warning("off");
undistortFisheyeImage()
%% parameters
disp("******load data and parameters!*******");
borW=0.77;
borH=0.63;
pcd_path1 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-lidar/pcd1";
pcd_path2 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-lidar/pcd2";

pcd_list1 = dir(pcd_path1);
pcd_list1 = pcd_list1(3:end);
pcd_list2 = dir(pcd_path2);
pcd_list2 = pcd_list2(3:end);
if size(pcd_list1,1)*size(pcd_list2,1)==0
    disp("Error! Data folder is empty");
    return;
end
pcd_corner3D1 = {};
pc_bors_coeff1 ={};
pcd_corner3D2 = {};
pc_bors_ceoff2 ={};
num=1;
disp("******* Extract corner features*******");
disp("*** the corner extraction in point could is time-consuming, please wait!");
for idx = 1:size(pcd_list1,1)
    disp("Frame: "+num2str(idx)+"st");
    % 输入数据
    pcd_file1 = strcat(pcd_list1(idx).folder,'/',pcd_list1(idx).name);
    pcd_file2 = strcat(pcd_list2(idx).folder,'/',pcd_list2(idx).name);
    pc_raw1 = pcread(pcd_file1);
    pc_raw2 = pcread(pcd_file2);
    pc_array1 = pc_raw1.Location()';
    pc_array2 = pc_raw2.Location()';
    [pts_bor1,bor_coeff1,err1] = boardpts_ext(pc_array1,borW,borH);
    if size(pts_bor1,2)<10
        disp("*** frame: "+num2str(idx)+ " is dropped");
        continue;
    end
    [pts_bor2,bor_coeff2,err2] = boardpts_ext(pc_array2,borW,borH);
    if size(pts_bor2,2)<10
        disp("*** frame: "+num2str(idx)+ " is dropped");
        continue;
    end
    pc_corners1 = borcorner_ext(pts_bor1,borW,borH);
    pc_corners2 = borcorner_ext(pts_bor2,borW,borH);
    
    % save data
    pcd_corner3D1{num} = pc_corners1;
    pc_bors_coeff1{num}=bor_coeff1;
    pcd_corner3D2{num} = pc_corners2;
    pc_bors_ceoff2{num}=bor_coeff2;
    num = num +1;
end
disp("****** starting extrinsic calibration*******");
TInit = plane_init(pc_bors_ceoff2,pc_bors_coeff1,pcd_corner3D2,pcd_corner3D1);
TOptm = corner_optm(pcd_corner3D1,pcd_corner3D2,TInit);
disp("*********Final Extrinsics: LiDAR2 to LiDAR1*********");
disp(TOptm);

figure;
axis equal;
plot3(pc_array1(1,:),pc_array1(2,:),pc_array1(3,:),'.r');
pc_array2_aft = TOptm(1:3,1:3)*pc_array2+ TOptm(1:3,4);
hold on;
plot3(pc_array2_aft(1,:),pc_array2_aft(2,:),pc_array2_aft(3,:),'.b');

pc_all = [pc_array1,pc_array2_aft];
pt = pointCloud(pc_all');
pt.Intensity = [ones(1,size(pc_array1,2)),255*ones(1,size(pc_array2_aft,2))]';
pcwrite(pt,'result.pcd');

