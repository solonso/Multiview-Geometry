%% Step 1
img1="imgl01311.jpg"
img2="imgl01396.jpg"

ratio = [0.4, 0.6, 0.8];

for i=ratio
    [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(img1,img2 , i,true,"Internal");
end

% step 2
% for i =  0.3:0.05:0.9
% [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(img1,img2 ,i,true,"Internal");
% 
% error = projectionerrorvec(H12,CL1uv,CL2uv)

% end
%step 3 
% run file lab2_section3_empty.m to draw plot

%section 4
% step 1
% H12 = computeHomographyEmpty(CL1uv,CL2uv,'Affine')%'Translation')

%step2
 homo = ["Translation", "Similarity", "Affine", "Projective"];

I1 = imread("00.png");
I2 = imread("02.png");
numImages = numel(homo);
% Loop over each homography type
for i = 1:numImages
    H12 = computeHomographyEmpty(Features(1).xy, Features(3).xy, homo{i});
    overlay = showwarpedimages(I1, I2, H12);
    
    subplot(2,2,i); % Arrange in a grid
    title( 'First Subplot' )
    imshow(overlay);
    
end
