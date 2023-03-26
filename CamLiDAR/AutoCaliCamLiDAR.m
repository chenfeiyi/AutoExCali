clc;
clear;
addpath("..");
addpath("../tools");
addpath("../solver");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");
warning('off');
%% parameters
disp("*********load parameters and data*********");
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
borW=0.767; % the geometric size of the checkerboard, unit meter
borH=0.626; % the geometric size of the checkerboard, unit meter
pcd_path = "/home/cfy/Documents/unifiedCali/data/real-world/cam-lidar/pcd2";
img_path = "/home/cfy/Documents/unifiedCali/data/real-world/cam-lidar/img2";
%% load data
img_list = dir(img_path);
pcd_list = dir(pcd_path);
pcd_list = pcd_list(3:end);
img_list = img_list(3:end);

img_corner3D={};
pcd_corner3D={};
pc_bors_coeff={};
cam_bors_coeff={};
%% extract corner features
disp("*********extract corner features*********");
disp("*** the corner extraction in point could is time-consuming, please wait!");
num=1;
for idx = 1:5
    % 输入数据
    disp("*** frame: "+num2str(idx)+"st");
    img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
    img_raw = imread(img_file);
    img_un = myundistortImage(img_raw,K,D);
    pc_raw = pcread(pcd_file);
    pc_array = pc_raw.Location()';

    [pts_bor,bor_coeff,err] = boardpts_ext(pc_array,borW,borH);
    if size(pts_bor,2)<10
        disp("*** frame: "+num2str(idx)+ " is dropped");
        continue;
    end
    pc_corners = borcorner_ext(pts_bor,borW,borH); % this step is time-consuming,please wait
    [im_corners,camPts,cam_plane_coeff,pro_err] = imgbor_ext(img_raw,K,D,0.07,borW,borH);
    if size(im_corners,2)<4
        disp("*** frame: "+num2str(idx)+ " is dropped");
        continue;
    end
    % save data
    pcd_corner3D{num} = pc_corners;
    pc_bors_coeff{num}=bor_coeff;
    img_corner3D{num} = im_corners;
    cam_bors_coeff{num} = cam_plane_coeff;
    num = num + 1;
end
%% extrinsic calibration
disp("*********starting extrinsic calibration*********");
TInit = plane_init(pc_bors_coeff,cam_bors_coeff,pcd_corner3D,img_corner3D);
disp("*********Initial Extrinsics*********");
disp(TInit);
TOptm = corner_optm(img_corner3D,pcd_corner3D,TInit);
disp("*********Final Extrinsics: LiDAR to camera*********");
disp(TOptm);

%% visualize
img_un = myundistortImage(img_raw,K,D);
figure;
img_pro = pt_project_depth2image(TOptm,K,pc_array,img_un);
imshow(img_pro);



