function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here

%   Four dimensional state: position_x, position_y, velocity_x, velocity_y


%previous state =state
%current measurement = x,y

% zprevious = C*xprevious
% do i test for C????

%C=eye
%A= [ 1 dt; 0 1]
%P= AP(t-1)A' +sigm          given
%R=CPC' + sigma0
%K=CPC'(R+CPC')^-1
%xt = Ax(t-1) + K (zt - CAx(t-1))

%new param = P - KCP


    %% Place parameters like covarainces, etc. here:
     
    
    if (~isfield(param,'P'))
        param.P=eye(4)*.1;
    end
     dt= t-previous_t;
     A=[1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
     sigm=0;
     
     
     P = A*param.P*A' + sigm;
     
     param.P=P;
     
     sigtheta=10^6;
     
     R=P + sigtheta;
     K=P * inv(R+P);
     
    
     
     
     
     
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    
  
    newstate = A*state' + K *( [x,y,state(3),state(4)]'- A*state');
    Pnew = param.P- K*param.P;
    
    param.P=Pnew;
    
    
    
    
    % Predict 330ms into the future
    predictx = newstate(1) + newstate(3)* 0.330;
    predicty = newstate(2) + newstate(4) * 0.330;
    % State is a four dimensional element
    state = newstate';
end
