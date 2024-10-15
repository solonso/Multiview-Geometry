rng(4); %fix seed

%% Step 1: Getting the Intrinsic and Extrinsic parameters:
% Intrinsic camera params
au = 557.0943; 
av = 712.9824;
u0 = 326.3819; 
v0 = 298.6679;

% Location of the world reference frame in camera coordinates in mm
Tx = 100; 
Ty = 0; 
Tz = 1500;
% World rotation w.r.t. camera coordinates
% Euler XYX1 angles
a = 0.8*pi/2;
b =   -1.8*pi/2;
c =  pi/5;

%% Step 2:The projection matrix P of the Camera
%The intrinsic parameters
K = [au 0 u0;
    0 av v0;
    0 0 1];

%The world frame representation of the camera frame
ctw = [Tx; Ty; Tz];

%The rotation of the camera frame w.r.t the world frame
cRw = [1 0 0; 0 cos(a) -sin(a);0 sin(a) cos(a)]*[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)]*[1 0 0; 0 cos(c) -sin(c);0 sin(c) cos(c)];

%The homogenous transformation of the camera frame refernced in the world
%frame
cTw = [cRw ctw; 0 0 0 1];

%The  projection matrix P 
P = K*[1 0 0 0;0 1 0 0;0 0 1 0]*cTw;

%Division of each element of the P witht the 3,4 element of the matrix, P
projection_params = P/P(3,4)

%% Step 3: 3D random generation function
%Randomly generated points
function p4d = points_3d_gen(n_points)
    %x=xmin+rand(4,6)*(xmax-xmin)
    p3d = -480+rand([3,n_points])*(480-(-480));
    p4d = [p3d; ones(1, n_points)]; %4xn randomly generated
end

%Projection of 3D points in 2D (u_v/w)pixels
function norm_projection_2d = points_2d_gen(p4d, projection_params)
    projection_2d =  projection_params*p4d;
    w_pixel = projection_2d(3, :);
    u_v_pixels = projection_2d(1:2, :);
    norm_projection_2d = u_v_pixels./w_pixel;
end

%% Step 4: The projection of the 3D 6 points on the image plane by using P
%The 2d projection of the 6 points
p4d_6 = points_3d_gen(6);
norm_projection_2d_6 = points_2d_gen(p4d_6, projection_params); %2d points

%% Step 5:Open a window in Matlab which will be used to show the image plane. Plot the 2D points.
figure;
scatter(norm_projection_2d_6(2, :), norm_projection_2d_6(1, :), 50, 'filled', 'MarkerFaceColor', [0.2, 0.6, 0.8]);

xlabel('u pixel along x-axis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('v pixel along y-axis', 'FontSize', 12, 'FontWeight', 'bold');
title('2D Image Points', 'FontSize', 16, 'FontWeight', 'bold');

set(gca, 'FontSize', 12, 'FontWeight', 'bold');
grid on;
box on; 

%
xlim([min(norm_projection_2d_6(2, :)) - 10, max(norm_projection_2d_6(2, :)) + 10]);
ylim([min(norm_projection_2d_6(1, :)) - 10, max(norm_projection_2d_6(1, :)) + 10]);

%% Step 6: Estimation of the 3x4 projection parameter matrix using Hall method
function intrinsic_params = estimate_params(pts_3d, norm_projection_2d_Q)
%Creating the rows of Qx and Qy
[rows,npoints]= size(pts_3d);
%The hall_Qx list of ones
hall_Qx = ones(npoints, 11);
%Hall_Qy list of ones
hall_Qy = ones(npoints, 11);

%Making the size(Q)
iXi = norm_projection_2d_Q(1,:);
iYi = norm_projection_2d_Q(2,:);
wXw = pts_3d(1,:);
wYw = pts_3d(2,:);
wZw = pts_3d(3,:);

%Computation of hall_Q for Qi number of points
for i = 1:npoints
Q_1 = [wXw(1,i) wYw(1,i) wZw(1,i) 1 0 0 0 0 -iXi(1,i)*wXw(1,i) -iXi(1,i)*wYw(1,i) -iXi(1,i)*wZw(1,i)];
Q_2 = [0 0 0 0 wXw(1,i) wYw(1,i) wZw(1,i) 1 -iYi(1,i)*wXw(1,i) -iYi(1,i)*wYw(1,i) -iYi(1,i)*wZw(1,i)];

hall_Qx(i, : ) = Q_1;
hall_Qy(i, : ) = Q_2;

end
hall_Q = [hall_Qx;hall_Qy];
B = [norm_projection_2d_Q(1,:),norm_projection_2d_Q(2,:)]';
A11 = [inv((hall_Q'*hall_Q))*hall_Q'*B ;1]';
 
intrinsic_params = [[A11(1:4)];[A11(5:8)];[A11(9:12)]];
end

%The hall method for 6 points
projection_2d_Q6=estimate_params(p4d_6, norm_projection_2d_6);

%% Step 7: P from Intrinsic Params
%Compare the matrix obtained in Step 6 to the one defined in step 2
proj_params_err_1 = mean(sqrt(sum(projection_params - projection_2d_Q6).^2))

%Extract the intrinsic parameter matrix K, and the camera rotation matrix ⬚𝑅𝑤𝑐 from the camera projection matrix P (see appendix)
function [K_new,cRw_new] = get_intrinsics_from_proj_matrix(P)
% Get left-side 3x3 block of P
M = P(:,1:3);
% This implements the RQ decomposition from QR in matlab
[Q,R] = qr(rot90(M,3));
R = rot90(R,2)';
Q = rot90(Q);
% Check the determinant of Q to make cRw a proper rotation
if det(Q) < 0
 cRw_new = -Q;
else
 cRw_new = Q;
end
% Get the normalized intrinsics
K_new = R/R(3,3);
end

[K_Q6, cRw_Q6] = get_intrinsics_from_proj_matrix(projection_2d_Q6);

%% Step 8: Comparison of a noisy projection matrix you obtain with the one you got in step 6
%Adding gaussian noise
function projection_param_noise = gaussian_noise(projection_param)
std_dev = 1 / 2;  % This ensures ±1 range for 95% of the points
gaussian = normrnd(0, std_dev, size(projection_param));

projection_param_noise = projection_param + gaussian;
end

%% Again, repeat step 6 with the noisy 2D points and the 3D points defined in step 3.
projection_noise_param_6 = gaussian_noise(norm_projection_2d_6);
est_projection_noise_Q6 = estimate_params(p4d_6,projection_noise_param_6);

%Compare the projection matrix you obtain with the one you got in step 6, with the noise-free points.
proj_err_6 = mean(sqrt(sum(projection_2d_Q6-est_projection_noise_Q6).^2))

%Extract the intrinsic parameter matrix K, and the camera rotation matrix %⬚𝑅𝑤%𝑐%from the camera projection you just computed.
[K_Q6_noise, cRw_Q6_noise] = get_intrinsics_from_proj_matrix(est_projection_noise_Q6);

%% Step 9: Now compute the 2D points with the projection matrix and compare them to those obtained in step 4. 
function [mean_err_K, matix_magnitude,phi_Q] = error(projection_params, est_projection_params)
%intrinsic parameter matrix K, and the camera rotation matrix
[K_Q, cRw_Q] = get_intrinsics_from_proj_matrix(projection_params)
%Estimated noisy intrinsic parameter matrix K, and the camera rotation matrix
[K_Q_noise, cRw_Q_noise] = get_intrinsics_from_proj_matrix(est_projection_params)

%mean squared error for Intrinsic parameter matrix K
mean_err_K = mean(sqrt(sum((K_Q-K_Q_noise).^2)));

%mean squared error for the rotation matrix
matrix_mult = cRw_Q*cRw_Q_noise';

d_1Q = matrix_mult(2,3) - matrix_mult(3,2);
d_2Q = matrix_mult(3,1) - matrix_mult(1,3);
d_3Q = matrix_mult(1,2) - matrix_mult(2,1);

matix_magnitude = sqrt(d_1Q*d_1Q + d_2Q*d_2Q + d_3Q*d_3Q);

phi_Q = asin(matix_magnitude*0.5);
end
%[mean_err_K6, dmag_6, phi_6] = error(projection_params, projection_2d_Q6)
[mean_err_K6, dmag_6, phi_6] = error(projection_params, est_projection_noise_Q6)

proj_2d_6Q_noise = points_2d_gen(p4d_6, est_projection_noise_Q6);
%For this comparison compute the average projection error.
proj_err_p6 = mean(sqrt(sum(norm_projection_2d_6-proj_2d_6Q_noise).^2))

%% Step 10: Increase the number of 3D points up to 10 points and then up to 50 points and repeat step 8 and 9.
mean_err = [];
mag_err = [];
phi_err = [];
proj_err = []
points = [6,10,50];
for i =points
     p4d = points_3d_gen(i);
     projections = points_2d_gen(p4d, projection_params);
     est_proj_param_org = estimate_params(p4d,projections)
     projection_noise_param = gaussian_noise(projections);
     est_proj_params_noise = estimate_params(p4d,projection_noise_param)
     [mean_err_Ki, dmag_i, phi_i] = error(est_proj_param_org, est_proj_params_noise);
     proj_err_i = mean(sqrt(sum(est_proj_param_org-est_proj_params_noise).^2))
     mean_err(end+1) = mean_err_Ki;
     mag_err(end+1) = dmag_i;
     phi_err(end+1) = phi_i;
     proj_err(end+1) = proj_err_i;
end
mean_err
mag_err
phi_err
proj_err

%% Plot a graph of the average projection error as a function
figure;
scatter(points, proj_err, 50, 'filled', 'MarkerFaceColor', [0.2, 0.6, 0.8]);
hold on;  
plot(points, proj_err, '-r', 'LineWidth', 1.5);  


xlabel('number of points', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('projection error', 'FontSize', 12, 'FontWeight', 'bold');
title('The average projection error', 'FontSize', 16, 'FontWeight', 'bold');

set(gca, 'FontSize', 12, 'FontWeight', 'bold');
grid on; 
box on;

