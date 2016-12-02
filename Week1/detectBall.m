% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%{
mu = [147.5070  142.9538   62.4636];
sig = [212.3682  134.6041 -225.7148;
  134.6041  138.7490 -182.7939;
 -225.7148 -182.7939  366.4080];
 thre = .000009;
%}
mu = [149.7479  144.8709   60.7774];


sig = [236.8499  155.4809 -254.8909;
  155.4809  151.4019 -195.2083;
 -254.8909 -195.2083  425.0172];
thre=.0000046;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 


yellowballimage =I;

[imageheight,imagewidth,imagedim]=size(yellowballimage);

R = I(:,:,1);
G = I(:,:,2);
B = I(:,:,3);


mask =zeros(imageheight,imagewidth);

for i =1:imageheight
    for j=1:imagewidth
        
      
        
   
        leftterm=1/sqrt(((2*pi)^3)*det(sig));
        
        
        
        rvalue=R(i,j);
        gvalue=G(i,j);
        bvalue=B(i,j);
        
        x=[rvalue gvalue bvalue];
        
        xdiff=double(x)-mu;
        xdifft=xdiff';
        sigmainv = sig^-1;
        
        rightterm=exp(-.5*xdiff*sigmainv*xdifft);
        
       
        
        probability=leftterm*rightterm;
        
      
        if(probability>thre)
            mask(i,j)=1;
        end
            
           
        
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops

   ballmask = zeros(imageheight,imagewidth); 

   
   
   connectivity = bwconncomp(mask);
   numOfPixels = cellfun(@numel, connectivity.PixelIdxList);
   [biggest,indexOfMax] = max(numOfPixels);
   ballmask(connectivity.PixelIdxList{indexOfMax}) = 1;
   
   fillball= imfill(ballmask,'holes');
   
   

   centroid=regionprops(fillball,'Centroid');
   centroid=centroid.Centroid;
   
  
   


   

 figure(2);
imshow(fillball)
hold on
plot(centroid(:,1),centroid(:,2), 'b*')
hold off








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = fillball;
 loc = centroid;

end
