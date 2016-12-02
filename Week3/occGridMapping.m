% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
 myResol = param.resol;
% % the initial map size in pixels
 myMap = zeros(param.size);
 map=zeros(param.size);
% % the origin of the map in pixels
 myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;

 %pose : pose (1:2) are the x,y location on the map and pose (3) is the
 %heading angle
 
 
 
 N = size(pose,2);
 angles=size(ranges,1);

 for i = 1:N
% for each time,
%      %convert from body to map
%       
       
    


%     % Find grids hit by the rays (in the gird map coordinate)
%       -bresenham
       P_0 = round( pose(1:2,i)*param.resol)+param.origin;
       
       xposition= P_0(1,1);
       yposition=P_0(2,1);
       theta = pose(3,i);
       
       d=ranges(:,i) *param.resol;
       alphaangle= scanAngles(:,1);
       
       angletotal=alphaangle+theta;
       
       cosangle= cos(angletotal);
       sinangle= sin(angletotal);
       
       xchange= d.*cosangle;
       
       ychange= d.*sinangle;
       
       
       
       xmapposition= xchange+xposition;
       ymapposition=ychange +yposition;
       
       
       %add the origin?
       
       occs= [xmapposition ymapposition];
       
       %divide by resolution?
       indices = ceil(occs); %indices of occupied squares
       
       
       
      
       
       orig= [xposition yposition];
       
      
       
       for j=1:angles
           
       
       
     
       
       
       [freex, freey] = bresenham(orig(1),orig(2),indices(j,1), indices(j,2)); 
       
      
       
       freeindices=[freex freey];
       free = sub2ind(size(myMap),freey,freex);
       occindices=indices;
       occupied= sub2ind(size(myMap),occindices(:,2),occindices(:,1));
     
       
       myMap(free)=myMap(free)-lo_free;
       myMap(occupied)=myMap(occupied)+lo_occ;
       
       
       
        
    
       
 
       end
       
       myMap(myMap<lo_min) = lo_min;
        myMap(myMap>lo_max) = lo_max;
       
       
%     % Find occupied-measurement cells and free-measurement cells
%       distances/ranges
% 
%     % Update the log-odds
%      -calculate
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
       
 end
 
 
      save('map.mat','myMap');

end

