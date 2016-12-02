% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)


%0.0164    0.0178    0.0090


% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
 myResolution = param.resol;
% % the origin of the map in pixels
 myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;

threshold=10;

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M =    800             ;           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
dTheta = .2;
dR= .02 ;

 for j = 2:N % start estimating from j=2 
% 
%     % 1) Propagate the particles 
%   

    Particle= zeros(3,M);
 
    
    for i = 1:M
    Particle(3,i) = myPose(3,j-1) + dTheta*randn; % sample angle
    Radius = dR*randn; % sample radius around point
    Particle(1,i) = myPose(1,j-1) + Radius*cos(Particle(3,i)); 
    Particle(2,i) = myPose(2,j-1) + Radius*sin(Particle(3,i));


    end
    
    
    
    
        
        particleweights= ones(1,M) * (1/M);
        
       
%       
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
%
     
      
      

%     %   2-2) For each particle, calculate the correlation scores of the particles
        
   
    correlation = zeros(1,M);
    
    for p= 1: M
        
        
       P_0 = round( Particle(1:2,p)*param.resol)+param.origin;
       
       xposition= P_0(1,1);
       yposition=P_0(2,1);
       theta = Particle(3,p);
       
       d=ranges(:,j) * param.resol;
       alphaangle= scanAngles(:,1);
       
       angletotal=alphaangle+theta;
       
       cosangle= cos(angletotal);
       sinangle= sin(angletotal);
       
       xchange= d.*cosangle;
       
       ychange= d.*sinangle;
       
       
       
       xmapposition= round(xchange+xposition);
       ymapposition=round(ychange +yposition);
       
     
     
       
       %%%%%%%
       xmapposition(xmapposition>size(map,2)) = size(map,2);
       ymapposition(ymapposition>size(map,1)) = size(map,1);
       xmapposition(xmapposition<1)=1;
       ymapposition(ymapposition<1)=1;
       
       indices = sub2ind(size(map), ymapposition, xmapposition);
       
       
       dirac = map(indices);
        
       % dirac= diag(map(xmapposition,ymapposition));
        
        
        
        correlationscore=sum(sum(dirac));
        
        correlation(1,p)=correlationscore;
        
        
        
        
        
        
        
        
        
        
    end
    
    
   
%
%     %   2-3) Update the particle weights    
        
       % particleweights=particleweights.*correlation;
        
%  
%     %   2-4) Choose the best particle to update the pose
%     
        
        [maximum,I] = max(correlation);
        
        bestpose=Particle(:,I);
        
        myPose(:,j)=bestpose;
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
        %{
        squaredparticles= correlation.^2;
        ssquaredparticles=sum(squaredparticles);
        
        sumsquaredp = sum(correlation)^2;
        
        neffective=ssquaredparticles/sumsquaredp
        
        if neffective<=.02 
            j=j-1;
        end
%}
j

%     % 4) Visualize the pose on the map as needed
%    

  
     
   
     
     
    
     
% 
 end
 
 
 
 %{
figure;
imagesc(map); 
colormap('gray');
axis equal;
hold on;
plot(myPose(1,:)*param.resol+param.origin(1), ...
    myPose(2,:)*param.resol+param.origin(2), 'r.-');
 %}
end

