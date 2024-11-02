function H12 = computeHomographyEmpty(CL1uv,CL2uv, Model)
%% computeHomography : estimate the Homography between two images according to Model 
%           Cluv1    set of points on image1 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Cluv2    set of points on image2 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Model       type of Homography to estimate. It has to be egual
%                       to one of the following strings: 'Translation',
%                       'Rigid', 'Similarity', 'Affine', 'Projective'.
%   Output
%           H12           estimated Homography of Model type. 3x3 matrix.
%


warning('This is an empty function just to help you with the switch command.')

    switch (Model)

        case 'Translation'
            shape_1 = size(CL2uv(:,1));
            % Compute here your 'Translation' homography
            A = []; % Initialize H12 matrix to store each row block
            new_x_prime =[]
            for i = 1:shape_1
                x=CL1uv(i,1)
                y= CL1uv(i,2)
                x_prime = CL2uv(i,1)
                y_prime = CL2uv(i,2)
                rows = [1,0,x;0,1,y]
                A = [A; rows];
                new_x_prime = [new_x_prime;CL2uv(i,1);CL2uv(i,2)]
            end
            unknowns = linsolve(A,new_x_prime)             % 3x3 matrix used to store the Homography
            H12 = [1,0,unknowns(1);  
                   0,1,unknowns(2);
                   0,0,1]
        case 'Similarity'
            
            % Compute here your 'Similarity' homography
            shape_1 = size(CL2uv(:,1));
            A = []; % Initialize H12 matrix to store each row block
            new_x_prime = []
            for i = 1:shape_1
                x=CL1uv(i,1)
                y= CL1uv(i,2)
                % x_prime = CL2uv(i,1)
                % y_prime = CL2uv(i,2)
                rows = [x,-y,1,0;
                    y,x,0,1]
                A = [A;rows]
                new_x_prime = [new_x_prime;CL2uv(i,1);CL2uv(i,2)]
            end
            unknowns = linsolve(A,new_x_prime)             % 3x3 matrix used to store the Homography
            H12 = [unknowns(1),unknowns(2),unknowns(3); ...
                   unknowns(2),unknowns(1),unknowns(4);
                   0,0,1]
            
        case 'Affine'
            
            % Compute here your 'Affine' homography
            
            shape_1 = size(CL2uv(:,1));
            % Compute here your 'Translation' homography
            A = []; % Initialize H12 matrix to store each row block
            new_x_prime = []
            for i = 1:shape_1
                x_prime = CL2uv(i,1)
                y_prime = CL2uv(i,2)
                x=CL1uv(i,1)
                y= CL1uv(i,2)
                rows = [x,y,1,0,0,0;
                        0,0,0,x,y,1]
                % rows= [x_prime, y_prime,1, 0, 0, 0, -x.*x_prime, -x.*y_prime; 
                %       0, 0, 0, x_prime, y_prime,1, -y.*x_prime, -y.*y_prime];
                A = [A; rows];
                new_x_prime = [new_x_prime;CL2uv(i,1);CL2uv(i,2)];

             end 
            unknowns = linsolve(A,new_x_prime)             % 3x3 matrix used to store the Homography
            H12 =  [unknowns(1),unknowns(2),unknowns(3);  
                   unknowns(4),unknowns(5),unknowns(6);
                   0,0,1]

        
        case 'Projective'
            
            % Compute here your 'Projective' homography
            shape_1 = size(CL2uv(:,1));
            % Compute here your 'Translation' homography
            A = []; % Initialize H12 matrix to store each row block
            new_x_prime = []
            for i = 1:shape_1
                x_prime = CL2uv(i,1)
                y_prime = CL2uv(i,2)
                x=CL1uv(i,1)
                y= CL1uv(i,2)
                rows = [x,y,1,0,0,0,0,0,0;
                        0,0,0,x,y,1,0,0,0;
                        0,0,0,0,0,0,x,y,1]
                % rows= [x_prime, y_prime,1, 0, 0, 0, -x.*x_prime, -x.*y_prime; 
                %       0, 0, 0, x_prime, y_prime,1, -y.*x_prime, -y.*y_prime];
                A = [A; rows];
                new_x_prime = [new_x_prime;CL2uv(i,1);CL2uv(i,2);1];

             end 
            unknowns = linsolve(A,new_x_prime)             % 3x3 matrix used to store the Homography
            H12 =  [unknowns(1),unknowns(2),unknowns(3);  
                   unknowns(4),unknowns(5),unknowns(6);
                   unknowns(7),unknowns(8),1]

        otherwise
            warning('Invalid model, returning identity homography');
            H12 = eye(3);
            
    end
    
    
end





