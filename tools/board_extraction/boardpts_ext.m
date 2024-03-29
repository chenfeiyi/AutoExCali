function [pts_out,plane_model,error_out] = boardpts_ext(pts_in,borW,borH)
pts_in_raw = pts_in;
ns_raw = KDTreeSearcher(pts_in_raw(1:3,:)');
%% params
search_radius = 5; %\mu_1
discontinuity_thres = 0.1; % default 0.05
eigen_thres = 0.90; % default 0.95
line_maxlen= 1;
min_err_thres = 0.06; %default 0.045 每个点到平面的平均距离，距离小于这个表示有效平面
template_rate = 10;
dbscan_elpson = 0.2;
sampleSize = 5;
maxdistance = 0.04;
%% only use points whose distance less than search_radius
pts_in_norm  = vecnorm(pts_in);
idx = find(pts_in_norm<search_radius);
pts_in = pts_in(:,idx);

% pts_cl = pointCloud(pts_in_raw');
% pts_cl.Intensity = ones(1,size(pts_in_raw,2))';
% [idx, dist] = knnsearch(ns_raw,pts_in','k',1);
% pts_cl.Intensity(idx) = 255;
% pcwrite(pts_cl,'radius.pcd');

%% extract ring points
thetaAlpha = [[],[]];
for index = 1: size(pts_in,2)
    R = norm(pts_in(:,index));
    z = pts_in(3,index);
    theta = round(asind(z/R));
    alpha = atan2d(pts_in(2,index),pts_in(1,index))+180;
    
    thetaAlpha = [thetaAlpha,[theta;alpha]];
end
pts_in = [pts_in;thetaAlpha];
%% find line segment from each line and filer impossible line segment.
pts_potional = [];
for angle = -15:2:15
    thetaidx = find(pts_in(4,:)==angle);
    same_ring_pts = pts_in(:,thetaidx);
    [B,sorted_idx] = sort(same_ring_pts(5,:));
    sorted_ring_pts =  same_ring_pts(1:3,sorted_idx);
    clusters_pts=[];
    offset=1;
    for index = 1:size(sorted_ring_pts,2)-1
        d_pp = norm(sorted_ring_pts(:,index+1)-sorted_ring_pts(:,index));
        if d_pp<discontinuity_thres && index~=(size(sorted_ring_pts,2)-1)
            continue;
        else
            offset = index+1;
            break;
        end
    end
    clusters_pts=[];
    size_pts = size(sorted_ring_pts,2);
    for index = offset:size_pts+offset-1
        if index > size(sorted_ring_pts,2)
            valid_idx = mod(index,size_pts);
        else
            valid_idx = index;
        end
        if valid_idx==0
            valid_idx=size_pts;
        end
        valid_idx = int32(valid_idx);
        valid_idx_plus = index+1;
        if valid_idx_plus > size_pts
            valid_idx_plus = mod(valid_idx_plus,size_pts);
        end
        if valid_idx_plus==0
           valid_idx_plus = size_pts; 
        end
        valid_idx_plus = int32(valid_idx_plus);
        d_pp = norm(sorted_ring_pts(:,valid_idx_plus)-sorted_ring_pts(:,valid_idx));
        
        if d_pp<discontinuity_thres && index~=(size_pts+offset-1)
            clusters_pts = [clusters_pts,sorted_ring_pts(:,valid_idx)];
        else
            clusters_pts =[clusters_pts,sorted_ring_pts(:,valid_idx)];
            dis = norm(clusters_pts(:,1) - clusters_pts(:,end));
            
            if size(clusters_pts,2)>15 && dis<line_maxlen
                [coeff,score,latent]=pca(clusters_pts');
                alpha1d = (latent(1)-latent(2))/latent(1);
                if alpha1d>eigen_thres
                    pts_potional = [pts_potional,clusters_pts];
                end
            end
            clusters_pts = [];
        end
    end
end

% pts_cl = pointCloud(pts_in_raw');
% pts_cl.Intensity = ones(1,size(pts_in_raw,2))';
% [idx, dist] = knnsearch(ns_raw,pts_potional','k',1);
% pts_cl.Intensity(idx) = 255;
% pcwrite(pts_cl,'preprocessed.pcd');
%% generate template board board points in board coordinate system
bor_corners = [[0;0;0],[0;borH;0],[borW;borH;0],[borW;0;0]];

x = linspace(bor_corners(1,1),bor_corners(1,2),template_rate);
y = linspace(bor_corners(2,1),bor_corners(2,2),template_rate);
z = linspace(bor_corners(3,1),bor_corners(3,2),template_rate);
template_pts = [x;y;z];
sample_vec = (bor_corners(:,3) - bor_corners(:,2))./(template_rate-1);
for sample_step = 1:template_rate-1
    new_sample_pts = [x;y;z] + sample_vec*sample_step;
    template_pts = [template_pts,new_sample_pts];
end


%% dbscan to cluster
idx = dbscan(pts_potional',dbscan_elpson,10);
%% find board from each cluster using icp
min_err = 100;
min_cluster=[];
min_transform=[];

% pts_cl = pointCloud(pts_in_raw');
% pts_cl.Intensity = ones(1,size(pts_in_raw,2))';
for idx_ele = 1: max(idx)
    idx_part = find(idx==idx_ele);
    pts_cluster = pts_potional(:,idx_part);
    
%     [idx_cl, dist] = knnsearch(ns_raw,pts_cluster','k',1);
%     pts_cl.Intensity(idx_cl) = idx_ele*255/max(idx);

    T = bor_icp(pts_cluster,template_pts);
    ptCloudIn = pointCloud(pts_cluster');
    ptCloudOut = pcdownsample(ptCloudIn, 'gridAverage', 0.05);
    err = match_error(template_pts,ptCloudOut.Location()',T);
    
    R = vecnorm(pts_cluster);
    z = pts_cluster(3,:);
    thetas = round(asind(z./R));
    thetas = unique(thetas);
    if err<min_err && size(thetas,2)>3
        min_err = err;
        min_cluster = pts_cluster;
        min_transform= T;
    end
end
% pcwrite(pts_cl,'clusters.pcd');
%% return check
error_out = 0;
if size(min_cluster,2)==0
    disp("no tag");
    pts_out = [];
    plane_model = [];
    return;
end

%% in previous preprocess, we may remove useful points. 
%  So we use kdtree to refind useful points based on previous cluster results
[model,inlinerIdx] = plane_ransac(min_cluster,0.03);
ns = KDTreeSearcher(pts_in(1:3,:)');
idx = rangesearch(ns,min_cluster',borH/2);

for index = 1:size(idx,1)
    candi_pts = pts_in(1:3,idx{index});
    diss = Plane3dVal(model,candi_pts');
    idxx = find(diss<maxdistance);
    candi_pts = candi_pts(:,idxx);
    idx_mem = ismember(candi_pts',min_cluster','row');
    candi_pts = candi_pts(:,~idx_mem);
    min_cluster = [min_cluster,candi_pts];    
end
% use dbscan to remove outliers
idx = dbscan(min_cluster',0.18,10);

idx_max = 0 ;
max_size = 0;
for idx_num = 1: max(idx)
    idx_ele = find(idx==idx_num);
    
    if size(idx_ele,1)>max_size
        max_size = size(idx_ele,1);
        idx_max = idx_ele;
    end
end
min_cluster = min_cluster(:,idx_max);
[model,inlieridx] = plane_ransac(min_cluster,0.03);
min_cluster =min_cluster(:,inlieridx);

ptCloudIn = pointCloud(min_cluster');
ptCloudOut = pcdownsample(ptCloudIn, 'gridAverage', 0.05);
min_err = match_error(template_pts,ptCloudOut.Location()',min_transform);
    
error_out = min_err;
if min_err>min_err_thres
    disp("no tag");
    pts_out = [];
    plane_model = [];
else
    pts_out = min_cluster;
    pts_in_ele_normal = -pinv(pts_out')*ones(size(pts_out,2),1);
    plane_model = [pts_in_ele_normal;1];
    plane_model = plane_model ./norm(pts_in_ele_normal );
    
%     pts_cl = pointCloud(pts_in_raw');
%     pts_cl.Intensity = ones(1,size(pts_in_raw,2))';
%     [idx_cl, dist] = knnsearch(ns_raw,pts_out','k',1);
%     pts_cl.Intensity(idx_cl) = 255;
%     pcwrite(pts_cl,"output.pcd");
end

end

