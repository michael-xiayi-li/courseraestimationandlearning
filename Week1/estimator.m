

% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = './train';
Samples = [];
for k=1:19
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end


hsvarray = Samples;
rgbarray=Samples;

averagered = mean(hsvarray(:,1))
averagegreen = mean(hsvarray(:,2))
averageblue = mean(hsvarray(:,3))

varred = var(double(hsvarray(:,1)));
vargreen = mean(double(hsvarray(:,2)));
varblue= var(double(hsvarray(:,3)));



%red-147.8992
%green-143.4107
%blue- 62.6454
%
%varred-222.9328
%vargreen -143.4107
%varblue -379.2861

mu=[averagered averagegreen averageblue];

sumofcov = zeros(3,3);

mu

for k= 1:size(rgbarray,1)
    
    
   xofi = rgbarray(k,:)';
   
   xofi=double(xofi);
    
   diffx = xofi-mu';
   
   times = diffx * diffx';
   
   sumofcov = sumofcov+times;



end


sigma = sumofcov/size(rgbarray,1)

varred = var(double(hsvarray(:,1)))
vargreen = mean(double(hsvarray(:,2)))
varblue= var(double(hsvarray(:,3)))


