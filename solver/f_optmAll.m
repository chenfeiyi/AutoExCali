function TOut = f_optmAll(TInit,corners_s,corners_t,b_pts_s,b_coeff_t)
%
% optimize the extrinsics using geometric corners and board points at the same time. 
% Minimize the point-to-point and point-to-plane distance simutaneously. Mainly used for lidar-camera calibration 
% TInit: initial extrinsics, [[R,t];[0,0,0,1]]
% corners_s: geometric corners from source sensor (lidar) [{[x1;y1;z1],[x2;y2;z2]...},{...},...]
% corners_t: geometric corners from target sensor (camera) [{[x1;y1;z1],[x2;y2;z2]...},{...},...]
% b_pts_s: board points from source sensor (lidar) [{[x1;y1;z1],[x2;y2;z2]...},{...},...]
% b_coeff_t: board coefficients from taeget sensor (camera) [{[a1;b1;c1;d1]},{[a2;b2;c2;d2]},...]
%
for idx = 1:size(corners_t,2)
    cur_corner_t = corners_t{idx};
    cur_corner_s = corners_s{idx};
    cur_corner_s_aft = TInit(1:3,1:3)*cur_corner_s+TInit(1:3,4);
    pts_t=[];
    pts_s =[];
    for idxidx=1:size(cur_corner_s_aft,2)
        min_dis = 10;
        min_idx= 0;
       for cam_idx = 1:size(cur_corner_t,2)
          dis = cur_corner_s_aft(:,idxidx)- cur_corner_t(:,cam_idx);
          dis = norm(dis);
          if min_dis>dis
            min_dis = dis;
            min_idx = cam_idx;
          end
       end
       if min_dis<10
          pts_t = [pts_t,cur_corner_t(:,min_idx)];
          pts_s = [pts_s,cur_corner_s(:,idxidx)];
       end
    end
    corners_t{idx} = pts_t;
    corners_s{idx} = pts_s;
end

x0 = [f_rotm2lee(TInit(1:3,1:3)),TInit(1:3,4)']; % 李代数　pha1,pha2,pha3,tx,ty,tz
F = @(x) objFunc(x(1:3),x(4:6),corners_s,corners_t,b_pts_s,b_coeff_t);
opts = optimoptions('fminunc','Display','off','Algorithm','trust-region','SpecifyObjectiveGradient',true,'MaxFunctionEvaluations', 10000,'MaxIterations',1500);
[x_optm,fval]=fminunc(F,double(x0),opts);

TOut = eye(4);
RR =  SO3.exp(x_optm(1:3));
TOut(1:3,1:3) = RR.R;
TOut(1:3,4) = x_optm(4:6)';

end


function [f,g] = objFunc(lee_vec,t_vec,corners_s,corners_t,b_pts_s,b_coeff_t)
RR = SO3.exp(lee_vec);
R = RR.R;
t_vec = t_vec';

f = 0;
lambda = 10;
for idx=1:size(corners_s,2)
    cur_c_s = corners_s{idx};
    cur_c_t = corners_t{idx};
    cur_bor_coeff_t = b_coeff_t{idx};
    cur_bor_s = b_pts_s{idx};

    for idx2 = 1:size(cur_c_s,2)
        f1 = R*cur_c_s(:,idx2) + t_vec - cur_c_t(:,idx2);
        f = f+f1'*f1;
    end

    f2 = cur_bor_coeff_t(1:3,:)'* (R*cur_bor_s+t_vec)+cur_bor_coeff_t(4,:);
    f = f + lambda*f2*f2'/size(f2,2);

end

g = zeros(6,1);

if nargout>1
    for idx=1:size(corners_s,2)
        cur_c_s = corners_s{idx};
        cur_c_t = corners_t{idx};
        cur_bor_coeff_t = b_coeff_t{idx};
        cur_bor_s = b_pts_s{idx};
        J1_R = [0,0,0]';
        J1_t = [0,0,0]';
        for idx2 = 1:size(cur_c_s,2)
            f1 = R*cur_c_s(:,idx2) + t_vec - cur_c_t(:,idx2);
            sub_J1_R = skew(R*cur_c_s(:,idx2));
            sub_J1_t = eye(3);
            J1_R = J1_R+2*sub_J1_R*f1;
            J1_t = J1_t+2*sub_J1_t*f1;
        end

        f2 = cur_bor_coeff_t(1:3,:)'* (R*cur_bor_s+t_vec)+cur_bor_coeff_t(4,:);
        J2_t =[0,0,0]';
        J2_R =[0,0,0]';
        for idx2=1:size(cur_bor_s,2)
            sub_J2_R = skew(R*cur_bor_s(:,idx2))*cur_bor_coeff_t(1:3,:);
            sub_J2_t = cur_bor_coeff_t(1:3,:);

            J2_t = J2_t+ 2*sub_J2_t*f2(:,idx2);
            J2_R = J2_R+ 2*sub_J2_R*f2(:,idx2);
        end
        J2_R = lambda*J2_R/size(cur_bor_s,2);
        J2_t = lambda*J2_t/size(cur_bor_s,2);

        g = g+ [J2_R+J1_R;J1_t+J2_t];
%     g = g+ [J1_R;J1_t];
    end
end
f = double(f);
g = double(g);
end

function lee_vec_o = f_rotm2lee(R)
quat = rotm2quat(R);
theta = 2*acos(quat(1));
n_vec = quat(2:4)./sin(theta/2);
lee_vec_o = theta*n_vec;
if R == eye(3)
    lee_vec_o = [0,0,0];
end

end