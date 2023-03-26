img_path = "/home/cfy/Desktop/tmp/2/1660790036.988286.jpg";
pcd_path = "/home/cfy/Desktop/tmp/1/1660790036.891512.pcd";

K = [1.3349e+03                     0      970.9237
     0    1.3333e+03      503.9811
     0                     0                   1];
D = [-0.4469 0.2503 2.9988e-05 -3.9176e-04  -0.0900];
T = [-0.1665    0.0835    0.9825    0.1274
   -0.9859   -0.0339   -0.1641   -0.0829
    0.0196   -0.9959    0.0880   -0.3542
         0         0         0    1.0000];
T = inv(T);
pcd_raw = pcread(pcd_path);
img_raw = imread(img_path);
img_un = myundistortImage(img_raw,K,D);
pts = pcd_raw.Location()';
img_pro = pt_project_depth2image(T,K,pts,img_un);
imshow(img_pro);