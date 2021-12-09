function [ mapS ] = mapScale( map1,map2 )
% Usage:
%	[ mapS ] = expandMap( mapA, mapB )
%
% MAPSCALE scales up or scales down map2 with respect to the scale of map1.
%
% The function takes two input arguments (map1,map2) and provides one output value (mapS).
%	map1            - map of reference 
%   map2            - map that needs to be scaled up or down
%   mapS            - The scaled map2
%
% By Vishnunandan Venkatesh and Pawan Rao ECET581
%% Scale Correction
if min(length(map1),length(map2))==length(map2) % find the smaller map among the two maps
    
    
    avgMap1 = mean(map1(1:length(map2)));% average of map1. Map1 values are considered till the length of map2
    avgMap2 = mean(map2);% average of map2
    scaleFactor = avgMap1/avgMap2; % scale factor is calculated using averages
    
    if scaleFactor>1.9
        
        disp("Scaling required")
        
        mapS = map2.*scaleFactor; %scaling up map2
    else
        mapS = map2;
    end
    scaleFactor = avgMap2/avgMap1; 
    if scaleFactor>1.9
        disp("Scaling required")
        mapS = map2./scaleFactor;% scaling down map2
    else
        mapS = map2;
    end
    
    
else % if map1 was shorter in length 
    
    
    avgMap1 = mean(map1);% average of map1. 
    avgMap2 = mean(map2(1:length(map1)));% average of map2. Map2 values are considered till the length of map2
    scaleFactor = avgMap1/avgMap2;% scale factor is calculated using averages
    
    if scaleFactor>1.9
        disp("Scaling required")
        map2 = map2.*scaleFactor;%scaling up map2
    else
        mapS = map2;
    end
    scaleFactor = avgMap2/avgMap1;
    if scaleFactor>1.9
        disp("Scaling required")
        map2 = map2./scaleFactor;%scaling down map2
    else
        mapS = map2;
    end
    
end


end