%% implementation of the paper <Unified Calibration for Multi-camera Multi-LiDAR Systems
%  using a Single Checkerboard>
clc;
clear;
addpath("..");
addpath("../tools");
addpath("../solver/");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");
%% load parameters
disp("******load data and parameters!*******");
K1= [ 825.6334,0,639.9610;0,824.9260,384.6734;0,0,1.0000];
D1=[-0.3371,0.1315,-6.3185e-06,-3.6323e-04,0];

K2 = [897.4566,0,635.4040;0,896.7992,375.3149;0,0,1];
D2 = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
borW=0.77;
borH=0.63;
img_path1 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-camera/img1";
img_path2 = "/home/cfy/Documents/unifiedCali/data/real-world/dual-camera/img2";
img_list1 = dir(img_path1);
img_list2 = dir(img_path2);
img_list1 = img_list1(3:end);
img_list2 = img_list2(3:end);
if size(img_list1,1)*size(img_list2,1)==0
    disp("Error! Data folder is empty");
    return;
end
%% extract features
disp("******* Extract corner features*******");
img_pxs1 = {};
img_pts1={};
img_pose1={};
img_pxs2 = {};
img_pts2={};
img_pose2={};
num=1;
for idx = 1:5
    disp("Frame: "+num2str(idx)+"st");
    img_file1 = strcat(img_list1(idx).folder,'/',img_list1(idx).name);
    img_raw1 = imread(img_file1);
    img_file2 = strcat(img_list2(idx).folder,'/',img_list2(idx).name);
    img_raw2 = imread(img_file2);
    [pxs1,pts1,T1,status1] = detectImage(img_raw1,0.07,K1,D1);
    [pxs2,pts2,T2,status2] = detectImage(img_raw2,0.07,K2,D2);
    if status2~=0 || status1~=0
        disp("Frame: "+num2str(idx)+"st, dropped");
        continue;
    end
    img_pts1{num}=pts1;
    img_pxs1{num}=pxs1;
    img_pose1{num} = T1;
    img_pts2{num}=pts2;
    img_pxs2{num}=pxs2;
    img_pose2{num} = T2;
    num = num + 1;
end

%% calculate extinsic parameters
disp("****** starting extrinsic calibration*******");
TInits = {};
for idx=1:size(img_pose1,2)
    TInit = img_pose1{idx}*inv(img_pose2{idx});
    TInits{idx} = TInit;
end
TInit = averageT(TInits);
TOptm = unitray_optm(img_pxs1,img_pts2,TInit,K1);
disp("********The final extrinsics********")
disp(TOptm);


function [imagePoints,worldPoints,T,status] = detectImage(img,pattern_size,K,D)
IntrinsicMatrix = K';
radialDistortion = [D(1),D(2),D(5)];
tangentialDist = [D(3),D(4)];
imageSize = [size(img,1),size(img,2)];
principalPoint =[K(1,3),K(2,3)];
cam_param = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
    radialDistortion,'TangentialDistortion',tangentialDist);
[imagePoints,boardSize] = detectCheckerboardPoints(img);

worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
worldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
[worldOrientation,worldLocation,inliers,status] = estimateWorldCameraPose(imagePoints,worldPoints,cam_param);
T = eye(4);
T(1:3,1:3) = worldOrientation';
T(1:3,4) = worldLocation';
T = inv(T);
worldPoints=T(1:3,1:3)*worldPoints'+T(1:3,4);

end

function Taver = averageT(Ts)
t=[0,0,0]';
eul=[0,0,0]';
for idx=1:size(Ts,2)
    Tcur = Ts{idx};
    t = t+Tcur(1:3,4);
    eul = eul + rotm2eul(Tcur(1:3,1:3))';
end
t=t./size(Ts,2);
eul =eul./size(Ts,2);
Taver = eye(4);
Taver(1:3,1:3) = eul2rotm(eul');
Taver(1:3,4) = t;
end

