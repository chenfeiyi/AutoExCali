function TOptm = unitray_optm(img_pxs1,img_pts2,tf0,K1)
%  used for Lee's approach

all_pxs1 = [];
all_pts2 = [];

for idx=1:size(img_pxs1,2)
    all_pxs1 = [all_pxs1,img_pxs1{idx}'];
    all_pts2 = [all_pts2,img_pts2{idx}];
end


pts1 = [all_pxs1;ones(1,size(all_pxs1,2))];
pts1 = inv(K1)*pts1;
pts1 = pts1./vecnorm(pts1);

x0 = [double(rotm2eul(tf0(1:3,1:3))),double(tf0(1:3,4)')]; 
F = @(x) obj_func(x(1:3),x(4:6),pts1,all_pts2);
opts = optimoptions('fminunc','Algorithm','quasi-newton','Display','off','MaxFunctionEvaluations', 1000);
[x_optm,fval] = fminunc(F,x0,opts);

R = eul2rotm(x_optm(1:3));
TOptm = [[R,x_optm(4:6)'];[0,0,0,1]];
end


function error = obj_func(rvec,tvec,pts1,pts2)
R = eul2rotm(rvec);
error = 0;
pts2_aft = R*pts2+tvec';
pts2_aft = pts2_aft./vecnorm(pts2_aft);
diff12 = pts1 - pts2_aft;
error = sum(vecnorm(diff12).^2);
error = double(error);
end