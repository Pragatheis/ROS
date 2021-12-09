function [ map] = expandMap( map1,map2 )
% Usage:
%	[ map ] = expandMap( mapA, mapB )
%
% EXPANDMAP uses SSD to match new maps onto already existing maps to create an expanded map.
%
% The function takes two input arguments (map1,map2) and provides one output value (map).
%	map1            - Already existing map
%   map2            - The new map or new scan which need to be matched with map1
% map               - The extended map
%
% By Vishnunandan Venkatesh and Pawan Rao ECET581

%% Initial Declarations
interval = 0.0061; % given scan intervals
thetaMap1 = 0;
lenMap1 = length(map1);
for z = 1:lenMap1
    
    thetaMap1(z)= (z-1)*interval; % theta values for map1
    
end
thetaMap2 = 0;

%% Scale Correction 

map2 = mapScale(map1,map2); % Calls mapScale function which returns scaled map2
%% Calculating SSD and required rowIndex at which match is obtained between map1 and map2

[rowIndex,columnIndex,error,revRot] = mapSsd(double(map1),double(map2)); % Calling function mapSsd

%% Bring map2 into map1 frame by rotation if necessary and expanding map

lenMap2 = length(map2);
for x = 1:lenMap2
    
    thetaMap2(x)= (x-1)*interval; %theta values for map2
    
end


% Rotation transform


if revRot == 0   % if rotation is done clockwise
    
    
    map = [map1(1:rowIndex);map2]; % the expanded map
    rotTrans = interval*rowIndex;  % rotTrans the amount to rotate by or the required transformation
    thetaMap2 = thetaMap2+rotTrans; % Transform theta for map2
    thetaMap = [thetaMap1(1:rowIndex) thetaMap2]; %Create theta matrix for expanded map
    
    polar(thetaMap',map) % plot the extended map
    
    
else   % if rotation is done anti clockwise
    map = [map2(1:rowIndex);map1]; % the expanded map
    rotTrans = interval*rowIndex;% rotTrans the amount to rotate by or the required transformation
    thetaMap2 = thetaMap2-rotTrans;% Transform theta for map2
    thetaMap = [thetaMap2(1:rowIndex) thetaMap1];%Create theta matrix for expanded map
    
    polar(thetaMap',map) % plot the extended map
    title('Matching map6 into labMap')
end





end