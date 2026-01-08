% This a empty script to help you move faster on you lab work.
clear all;
close all;
clc;
rng(45)
%% ####################################################################################################################################################################
%% Step 1
% Camera 1
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
imageSize = [256 256];
%% ####################################################################################################################################################################
%% Step 2
% Camera 2
au2 = 90; av2 = 110; uo2 = 128; vo2 = 128; 
ax = 0.1; by = pi/4; cz = 0.2; % XYZ EULER 
tx = -1000; ty = 190; tz = 230; 
%% ####################################################################################################################################################################
%% STEP 3
% Compute intrinsic matrices and projection matrices

K1 = [au1 0 uo1; 0 av1 vo1; 0 0 1]; % Intrisics matrix for camera 1
wR1c = eye(3);   % rotation of camera 1, from the camera to the world coordinate frame
wt1c = [0 0 0]'; % translation of camera 1, from the camera to the world coordinate frame

% Note: ************** You have to add your own code from here onward ************
%The intrinsic matrix for camera 2
K2 = [au2 0 uo2; 0 av2 vo2; 0 0 1];% Intrinsics matrix for camera 2
eul = [ax by cz]; %euler angles for X, Y and Z coordinates
wR2c = eul2rotm(eul, 'XYZ' ); %matlab function for computing the euler angles
wt2c = [tx ty tz]'; %translatiion of camera 2, from the camera to the world coordinate frame

% The projection matrix for camera 1
P1 = K1 * [wR1c' -wR1c' * wt1c];

%The projection matrix for camera 2 
P2 = K2 * [wR2c' -wR2c' * wt2c]; % You have to replace this. This is just to serve as input for the drawing functions

%% ####################################################################################################################################################################
%% STEP 4 
% Attention: This is an invented matrix just to have some input for the drawing
% functions. You have to compute it properly 
%Buildinf the skew symmetric matrix
skew_tx =[0 ,-tz,ty;
    tz,0,-tx;
    -ty,tx,0];

%Computing the fundamental matrix
F = inv(K2)' * wR2c' * skew_tx * inv(K1);

%Normalization of the fundamental matrix
F = F./F(3,3);

%testing that 

fprintf('Step 4:\n\tAnalytically obtained F:\n');
disp(F);
 %% ####################################################################################################################################################################
%% STEP 5
V(:,1) = [100;-400;2000];
V(:,2) = [300;-400;3000];
V(:,3) = [500;-400;4000];
V(:,4) = [700;-400;2000];
V(:,5) = [900;-400;3000];
V(:,6) = [100;-40;4000];
V(:,7) = [300;-40;2000];
V(:,8) = [500;-40;3000];
V(:,9) = [700;-40;4000];
V(:,10) = [900;-40;2000];
V(:,11) = [100;40;3000];
V(:,12) = [300;40;4000];
V(:,13) = [500;40;2000];
V(:,14) = [700;40;3000];
V(:,15) = [900;40;4000];
V(:,16) = [100;400;2000];
V(:,17) = [300;400;3000];
V(:,18) = [500;400;4000];
V(:,19) = [700;400;2000];
V(:,20) = [900;400;3000];
 %% ####################################################################################################################################################################
%% STEP 6
% Projection on image planes
cam1_p2d = mvg_projectPointToImagePlane(V,P1); %u1,v1
cam2_p2d = mvg_projectPointToImagePlane(V,P2);%u2,v2

%testing that
%multi = [cam1_p2d;ones(1,20)]'*F*[cam2_p2d;ones(1,20)]
 %% ####################################################################################################################################################################
%% STEP 7
% example of the plotting functions 
% Draw 2D projections on image planes
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Projected points on image plane 1');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Projected points on image plane 2');
 
% % Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
[U, S, V] = svd(F);
ep_1 = V(:,end) % These are invented values just for illustrating. You have to compute them
ep_2 = U(:,end) % These are invented values just for illustrating. You have to compute them
ep_1 = ep_1/ep_1(end,1) % Normalizing
ep_2 = ep_2/ep_2(end,1) % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig,ep_1',ep_2');
%% ####################################################################################################################################################################
%% Step 8
%% Making function to calculate the 8-point method
function u_n = u_matrix(x1, x2)
num_1 = length(x1);
u_n = [x2(1,:)'.*x1(1,:)' x2(1,:)'.*x1(2,:)' x2(1,:)' x2(2,:)'.*x1(1,:)' x2(2,:)'.*x1(2,:)' x2(2,:)' x1(1,:)' x1(2,:)' ones(num_1,1) ];
end

%8-Point method
U_n = u_matrix(cam1_p2d, cam2_p2d);%8-point computation for the two cameras
[U, S, V] = svd(U_n);

F_8u_n = reshape(V(:,9),3,3)'; %the last column of the V matrix which forms the 8-point fundamental matrix

F_8 = F_8u_n; %Making a copy of F_8u_n to apply the rank constraint

% Enforce rank2 constraint
[U,D,V] = svd(F_8);
F_8 = U*diag([D(1,1) D(2,2) 0])*V'; %Enforcing the rank 2 constraint by setting D(3,3) = 0
F_8 = F_8./F_8(3,3); %Normalizing the F_8 matrix

fprintf('Step 8:\n\tAnalytically obtained F-8 point with rank 2 enforcement:\n');
disp(F_8);
%% ####################################################################################################################################################################
%% step 9 sum of abs diff between step 4 and 8
difference1 = sum(abs(F_8-F),"all"); %calculating the sum of absoute difference between F-8 and F
fprintf('Step 9:\n\tComparing F-8 point and F:\n');
disp(difference1); 
%% ####################################################################################################################################################################
%% step 10 
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Using F_8: Projected points on image plane 1 with rank enforcement');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Using F_8: Projected points on image plane 2 with rank enforcement');

% Draw epipolar lines with rank 2 enforcement
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F_8);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
[U, S, V] = svd(F_8);
ep_1_8 = V(:,end); % These are invented values just for illustrating. You have to compute them
ep_2_8 = U(:,end); % These are invented values just for illustrating. You have to compute them
ep_1_8 = ep_1_8/ep_1_8(end,1); % Normalizing
ep_2_8 = ep_2_8/ep_2_8(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig,ep_1_8',ep_2_8');

%% Without Enforcing rank2 constraint
F_8n = F_8u_n;

cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'Using F_8: Projected points on image plane 1 without rank enforcement');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'Using F_8: Projected points on image plane 2 without rank enforcement');

% Draw epipolar lines with rank 2 enforcement
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F_8n);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'r');

% Draw epipoles
[U, S, V] = svd(F_8);
ep_1_8n = V(:,end); % These are invented values just for illustrating. You have to compute them
ep_2_8n = U(:,end); % These are invented values just for illustrating. You have to compute them
ep_1_8n = ep_1_8n/ep_1_8n(end,1); % Normalizing
ep_2_8n = ep_2_8n/ep_2_8n(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig,ep_1_8n',ep_2_8n');


% Compute epipolar lines in image 2
l_prime = F_8 * [cam1_p2d; ones(1, size(cam1_p2d, 2))]; 

% Normalize the line coefficients for better numerical stability
l_prime = l_prime ./ sqrt(l_prime(1,:).^2 + l_prime(2,:).^2);
%% ####################################################################################################################################################################
%% step 11
function noisy_cam_p2d = add_gaussian_noise(projection_param, range)
std_dev = range/2;  % This ensures ±1 range for 95% of the points
gaussian = std_dev*randn(size(projection_param));

noisy_cam_p2d = projection_param + gaussian;
end

%% ####################################################################################################################################################################
%% step 12

noisy_cam1_p2d= add_gaussian_noise(cam1_p2d, 1);
noisy_cam2_p2d= add_gaussian_noise(cam2_p2d, 1);

U_noise = u_matrix(noisy_cam1_p2d, noisy_cam2_p2d);
[U, S, noisy_V] = svd(U_noise);
F_8_noise = reshape(noisy_V(:,9),3,3)';

% Without rank 2 constriant enforcement
F_8_noise1 = F_8_noise;
[U,D,V] = svd(F_8_noise1);
F_8_noise1 = U*D*V';
F_8_noise1 = F_8_noise1./F_8_noise1(3,3);

fprintf('Step 4:\n\tAnalytically obtained F_8_noise 1 without rank 2 enforcement:\n');
disp(F_8_noise1);

difference_noise1 = sum(abs(F_8_noise1-F_8),"all");
disp(difference_noise1);


cam1_fig_noise = mvg_show_projected_points(noisy_cam1_p2d(1:2,:),imageSize,'Step 12: Noise-1 Projected points on image plane 1 without rank 2 enforcement');
cam2_fig_noise = mvg_show_projected_points(noisy_cam2_p2d(1:2,:),imageSize,'Step 12: Noise-1 Projected points on image plane 2 without rank 2 enforcement');

[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy_cam1_p2d,noisy_cam2_p2d,F_8_noise1);
[cam1_fig_noise,cam2_fig_noise] = mvg_show_epipolar_lines(cam1_fig_noise, cam2_fig_noise, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'r');

[U, S, V] = svd(F_8_noise1);
ep_1noise1 = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
ep_2noise1 = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
ep_1noise1 = ep_1noise1/ep_1noise1(end,1); % Normalizing
ep_2noise1 = ep_2noise1/ep_2noise1(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig_noise, cam2_fig_noise,ep_1noise1',ep_2noise1');


%% Without rank 2 constraint enforcement
% Enforce 
[U,D,V] = svd(F_8_noise);
F_8_noise = U*diag([D(1,1) D(2,2) 0])*V';
F_8_noise = F_8_noise./F_8_noise(3,3);



fprintf('Step 4:\n\tAnalytically obtained F_8_noise 1 with rank 2 enforcement:\n');
disp(F_8_noise);

difference_noise1n = sum(abs(F_8_noise-F_8),"all");
disp(difference_noise1n);


cam1_fig_noise = mvg_show_projected_points(noisy_cam1_p2d(1:2,:),imageSize,'Step 12: Noise-1 Projected points on image plane 1 with rank 2 enforcement');
cam2_fig_noise = mvg_show_projected_points(noisy_cam2_p2d(1:2,:),imageSize,'Step 12: Noise-1 Projected points on image plane 2 with rank 2 enforcement');

[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy_cam1_p2d,noisy_cam2_p2d,F_8_noise);
[cam1_fig_noise,cam2_fig_noise] = mvg_show_epipolar_lines(cam1_fig_noise, cam2_fig_noise, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

[U, S, V] = svd(F_8_noise);
ep_1noise1n = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
ep_2noise1n = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
ep_1noise1n = ep_1noise1n/ep_1noise1n(end,1); % Normalizing
ep_2noise1n = ep_2noise1n/ep_2noise1n(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig_noise, cam2_fig_noise,ep_1noise1n',ep_2noise1n');
%% ####################################################################################################################################################################
%% step 13
%Using the created function in step 11 which is flexible to add 95% noise
%[-2, 2]
noisy1_cam1_p2d= add_gaussian_noise(cam1_p2d, 2);
noisy2_cam2_p2d= add_gaussian_noise(cam2_p2d, 2);

U_noise1 = u_matrix(noisy1_cam1_p2d, noisy2_cam2_p2d);
[U, S, noisy1_V] = svd(U_noise1);
F_8_noise2_u = reshape(noisy1_V(:,9),3,3)';

%% Without Rank 2 Constraint Enforcement
% Enforce rank2 constraint
F_8_noise2 = F_8_noise2_u;
[U,D,V] = svd(F_8_noise2);
F_8_noise2 = U*D*V';
F_8_noise2 = F_8_noise2./F_8_noise2(3,3);

%test = 
fprintf('Step 13:\n\tAnalytically obtained F_8_noise 2 without rank 2 enforcement:\n');
disp(F_8_noise2);

difference_noise2 = sum(abs(F_8_noise2-F_8),"all")
disp(difference_noise2);


cam1_fig_noise2 = mvg_show_projected_points(noisy1_cam1_p2d(1:2,:),imageSize,'Step 13: Noise-2 Projected points on image plane 1 without rank 2 enforcement');
cam2_fig_noise2 = mvg_show_projected_points(noisy2_cam2_p2d(1:2,:),imageSize,'Step 13: Noise-2 Projected points on image plane 2 without rank 2 enforcement');

[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy1_cam1_p2d,noisy2_cam2_p2d,F_8_noise2);
[cam1_fig_noise2,cam2_fig_noise2] = mvg_show_epipolar_lines(cam1_fig_noise2, cam2_fig_noise2, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'r');

[U, S, V] = svd(F_8_noise2);
ep_1noise2 = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
ep_2noise2 = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
ep_1noise2 = ep_1noise2/ep_1noise2(end,1); % Normalizing
ep_2noise2 = ep_2noise2/ep_2noise2(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig_noise2, cam2_fig_noise2,ep_1noise2',ep_2noise2');


%% Enforce rank2 constraint
F_8_noise2n = F_8_noise2_u;
[U,D,V] = svd(F_8_noise2n);
F_8_noise2n = U*diag([D(1,1) D(2,2) 0])*V';
F_8_noise2n = F_8_noise2n./F_8_noise2n(3,3);

fprintf('Step 13:\n\tAnalytically obtained F_8_noise 2 with rank 2 enforcement:\n');
disp(F_8_noise2n);

difference_noise2n = sum(abs(F_8_noise2n-F_8),"all");
disp(difference_noise2n);

cam1_fig_noise2 = mvg_show_projected_points(noisy1_cam1_p2d(1:2,:),imageSize,'Step 13: Noise-2 Projected points on image plane 1 with rank 2 enforcement');
cam2_fig_noise2 = mvg_show_projected_points(noisy2_cam2_p2d(1:2,:),imageSize,'Step 13: Noise-2 Projected points on image plane 1 with rank 2 enforcement');

[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy1_cam1_p2d,noisy2_cam2_p2d,F_8_noise2n);
[cam1_fig_noise2,cam2_fig_noise2] = mvg_show_epipolar_lines(cam1_fig_noise2, cam2_fig_noise2, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

[U, S, V] = svd(F_8_noise2n);
ep_1noise2n = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
ep_2noise2n = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
ep_1noise2n = ep_1noise2n/ep_1noise2n(end,1); % Normalizing
ep_2noise2n = ep_2noise2n/ep_2noise2n(end,1); % Normalizing

[~,~] = mvg_show_epipoles(cam1_fig_noise2, cam2_fig_noise2,ep_1noise2n',ep_2noise2n');

%% ####################################################################################################################################################################
%% Step 14 (repetition of step 8 and 9)
    
    % Apply normalization to noisy points in step 13
    function [normalized_points, norm_matrix] = normalize(camera_2dp, camera_2d)
    %center of points
    center_point = sum(camera_2dp, 2)/size(camera_2dp, 2);
    center_points = camera_2dp - (center_point*ones(1, size(camera_2dp, 2)));
    
    %scale factor
    scale_factor = sqrt(2)/ (sum(sqrt(sum(center_points.^2)))/size(camera_2dp, 2));
    
    %norm matrix
    norm_matrix = [scale_factor*eye(2), -scale_factor*center_point; 0 0 1];
    
    normalized_points = norm_matrix*[camera_2d; ones(1, size(camera_2d, 2))];
    end
    
    [norm_cam1_p2d, norm_matrix1] = normalize(noisy1_cam1_p2d(1:2, :), noisy1_cam1_p2d);
    [norm_cam2_p2d, norm_matrix2] = normalize(noisy2_cam2_p2d(1:2, :), noisy2_cam2_p2d);
    
    %% ####################################################################################################################################################################
    %step 8
    u_norm = u_matrix(norm_cam1_p2d, norm_cam2_p2d);
    [U, S, norm_V] = svd(u_norm);
    F_8_nor = reshape(norm_V(:,9),3,3)';
    F_8_nor = F_8_nor ./F_8_nor(3,3);
    
    %% Without rank 2 enforcement
    F_8_norm1 = F_8_nor;
    %F_8_norm1 = F_8_norm1./F_8_norm1(3,3);
    [U, D, V] = svd(F_8_norm1);
    %F_8_norm = U*diag([D(1,1) D(2,2) 0])*V';
    % Without enforcing the rank 2 (using the D)
    F_8_norm1 = U * D * V';
    
    
    % undo scaling
    F_8_norm1 = norm_matrix1' * F_8_norm1 * norm_matrix2;
    
    
    fprintf('Step 14:\n\tF_8_norm denormalized without rank enforcement:\n');
    disp(F_8_norm1);
    %step 9:
    difference_norm1 = sum(abs(F_8_norm1-F_8),"all");
    disp(difference_norm1);
    
    
    cam1_fig_noise2 = mvg_show_projected_points(noisy1_cam1_p2d(1:2,:),imageSize,'Step 14: Noise-2 Projected points on image plane 1 without rank 2 enforcement');
    cam2_fig_noise2 = mvg_show_projected_points(noisy2_cam2_p2d(1:2,:),imageSize,'Step 14: Noise-2 Projected points on image plane 2 without rank 2 enforcement');
    
    [~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy1_cam1_p2d,noisy2_cam2_p2d,F_8_norm1);
    [cam1_fig_noise2,cam2_fig_noise2] = mvg_show_epipolar_lines(cam1_fig_noise2, cam2_fig_noise2, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'r');
    
    [U, S, V] = svd(F_8_norm1);
    ep_1norm1 = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
    ep_2norm1 = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
    ep_1norm1 = ep_1norm1/ep_1norm1(end,1); % Normalizing
    ep_2norm1 = ep_2norm1/ep_2norm1(end,1); % Normalizing
    
    %Draw the epipoles
    [~,~] = mvg_show_epipoles(cam1_fig_noise2, cam2_fig_noise2,ep_1norm1',ep_2norm1');
    
    %% With rank2 enforcement
    F_8_norm = F_8_nor;
    %F_8_norm = F_8_norm./F_8_norm(3,3);
    [U, D, V] = svd(F_8_norm);
    % Enforce rank-2 (D(3, 3) = 0)
    F_8_norm = U*diag([D(1,1) D(2,2) 0])*V';
    
    
    % undo scaling
    F_8_norm = norm_matrix1' * F_8_norm * norm_matrix2;
    
    
    fprintf('Step 14:\n\tF_8_norm without rank enforcement:\n');
    disp(F_8_norm);
    %% ####################################################################################################################################################################
    %step 9:
    difference2 = sum(abs(F_8_norm-F_8),"all");
    
    
    cam1_fig_noise2 = mvg_show_projected_points(noisy1_cam1_p2d(1:2,:),imageSize,'Step 14: Noise-2 Projected points on image plane 1 with rank 2 enforcement');
    cam2_fig_noise2 = mvg_show_projected_points(noisy2_cam2_p2d(1:2,:),imageSize,'Step 14: Noise-2 Projected points on image plane 2 with rank 2 enforcement');
    
    [~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(noisy1_cam1_p2d,noisy2_cam2_p2d,F_8_norm);
    [cam1_fig_noise2,cam2_fig_noise2] = mvg_show_epipolar_lines(cam1_fig_noise2, cam2_fig_noise2, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');
    
    [U, S, V] = svd(F_8_norm);
    ep_1norm = V(:,end);%[-300 200 1];    % These are invented values just for illustrating. You have to compute them
    ep_2norm = U(:,end);% [200 50 1];      % These are invented values just for illustrating. You have to compute them
    ep_1norm = ep_1norm/ep_1norm(end,1); % Normalizing
    ep_2norm = ep_2norm/ep_2norm(end,1); % Normalizing
    
    [~,~] = mvg_show_epipoles(cam1_fig_noise2, cam2_fig_noise2,ep_1norm',ep_2norm');
%% ####################################################################################################################################################################