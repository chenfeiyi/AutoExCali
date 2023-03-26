function [corners] = borcorner_ext(bor_pts,borW,borH)
is_display=0;
corners=[];

%% find candidate solution using icp
sample_rate=200;
x = 0:borW/sample_rate:borW;
y = 0:borH/sample_rate:borH;

sample_pts=[];
sample_pts =[sample_pts,[x;zeros(2,size(x,2))]];
sample_pts =[sample_pts,[x;borH*ones(1,size(x,2));zeros(1,size(x,2))]];
sample_pts =[sample_pts,[zeros(1,size(y,2)-2);y(2:end-1);zeros(1,size(y,2)-2)]];
sample_pts =[sample_pts,[borW*ones(1,size(y,2)-2);y(2:end-1);zeros(1,size(y,2)-2)]];

sample_pts = sample_pts - [borW/2,borH/2,0]';


Tf0 = eye(4);
[plane_n,inlierIdx] = plane_ransac(bor_pts,0.02);
plane_n = plane_n(1:3);
ax = -cross(plane_n,[0,0,1]');
angle = [0,0,1]*plane_n;
angle = acosd(angle);
Tf0(1:3,1:3) = axang2rotm([ax',angle*pi/180]);
Tf0(1:3,4) = mean(bor_pts,2);
edge_pts_idx = find_pts_ring_edges(bor_pts);
edge_pts = bor_pts(:,edge_pts_idx);

%% enumerate extrinics around candidate solution and take the extrinsic assoicated with minimum loss
theta = [0:5:90];
q = [];
source_normal =[0,0,1]';
for idx = 1:size(theta,2)
    q = [q;[cosd(theta(idx)),sind(theta(idx))*source_normal']];
end
min_error=10;
Ttmp= Tf0;
TInit = Tf0;
for idx=1:size(theta,2)
    deltaT = [[quat2rotm(q(idx,:)),[0,0,0]'];[0,0,0,1]];
    Ttmp = inv(deltaT*inv(Tf0));

    sample_pts_tmp = Ttmp(1:3,1:3)*sample_pts+Ttmp(1:3,4);
    error = suqare_dis(edge_pts,sample_pts_tmp);
    if error<min_error
        min_error = error;
        TInit=Ttmp;
    end
end
plane_coeff = plane_ransac(bor_pts,0.03);
qn = plane_coeff(1:3);
pts_onboard=[];
for idx=1:size(bor_pts,2)
    pt = bor_pts(:,idx);
    ptonboard =[0,0,-plane_coeff(4)/plane_coeff(3)]';
    vec = pt - ptonboard;
    a = qn'*vec*qn;
    pts_onboard = [pts_onboard,vec - a+ptonboard];
end
edge_pts_idx = find_pts_ring_edges(pts_onboard);
edge_pts = pts_onboard(:,edge_pts_idx);

%% optimize extrinsics
corner3D = [[0;0;0],[borW;0;0],[borW;borH;0],[0;borH;0]];
corner3D = corner3D - [borW/2;borH/2;0];

TInit = inv(TInit);
TOptm = GlobalSearchOptm(edge_pts,corner3D,TInit);
TOptm = inv(TOptm);
corners = TOptm(1:3,1:3)*corner3D+TOptm(1:3,4);

if is_display
    %     corners3D = TInit(1:3,1:3)*corner3D+TInit(1:3,4);
    %     lines_cor = findCorres(corners3D,edge_pts);
    %     pts1idx = find(lines_cor==1);
    %     pts2idx = find(lines_cor==2);
    %     pts3idx = find(lines_cor==3);
    %     pts4idx = find(lines_cor==4);
    %     pts1 = edge_pts(:,pts1idx);
    %     pts2 = edge_pts(:,pts2idx);
    %     pts3 = edge_pts(:,pts3idx);
    %     pts4 = edge_pts(:,pts4idx);
    %
    %     figure;
    %     axis equal;
    %     hold on;
    %     plot3(bor_pts(1,:),bor_pts(2,:),bor_pts(3,:),'.b')
    %     plot3(pts1(1,:),pts1(2,:),pts1(3,:),'.r','MarkerSize',30);
    %     plot3(pts2(1,:),pts2(2,:),pts2(3,:),'.g','MarkerSize',30);
    %     plot3(pts3(1,:),pts3(2,:),pts3(3,:),'.b','MarkerSize',30);
    %     plot3(pts4(1,:),pts4(2,:),pts4(3,:),'.k','MarkerSize',30);
    %     plot3(corners3D(1,1:2),corners3D(2,1:2),corners3D(3,1:2),'o-r','LineWidth',2);
    %     plot3(corners3D(1,2:3),corners3D(2,2:3),corners3D(3,2:3),'o-g','LineWidth',2);
    %     plot3(corners3D(1,3:4),corners3D(2,3:4),corners3D(3,3:4),'o-b','LineWidth',2);
    %     plot3(corners3D(1,[4,1]),corners3D(2,[4,1]),corners3D(3,[4,1]),'o-k','LineWidth',2);
    %
    figure
    axis equal
    plot3(edge_pts(1,:),edge_pts(2,:),edge_pts(3,:),'.b')
    hold on
    sample_pts2 = TInit(1:3,1:3)*sample_pts+TInit(1:3,4);
    plot3([corners(1,:),corners(1,1)],[corners(2,:),corners(2,1)],[corners(3,:),corners(3,1)],'o-r')
end

end

function [dis]=suqare_dis(pts_target,pts_source)

ns = createns(pts_source','nsmethod','kdtree');
[idx,dist]= knnsearch(ns,pts_target','k',1);
dis = sum(abs(dist));
end


function line_cor = findCorres(corners,pts_in)
dir = zeros(3,4);
dir(:,1) = corners(:,2) - corners(:,1);
dir(:,2)= corners(:,3) - corners(:,2);
dir(:,3) = corners(:,4) - corners(:,3);
dir(:,4) = corners(:,1) - corners(:,4);

dir(:,1)= dir(:,1)./norm(dir(:,1));
dir(:,2)= dir(:,2)./norm(dir(:,2));
dir(:,3)= dir(:,3)./norm(dir(:,3));
dir(:,4)= dir(:,4)./norm(dir(:,4));
line_cor=[];
for idx=1:size(pts_in,2)
    min_dis=10;
    min_idxidx=0;
    for idxidx=1:4
        dis = (eye(3) - dir(:,idxidx)*dir(:,idxidx)')*(pts_in(:,idx)-corners(:,idxidx));
        dis = norm(dis);
        if dis<min_dis
            min_dis = dis;
            min_idxidx = idxidx;
        end
    end
    line_cor = [line_cor,min_idxidx];
end
end

