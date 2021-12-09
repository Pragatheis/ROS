function [ x,y,sum_values_temp,rev ] = mapSsd( map1, map2 )
% Usage:
%	[ x,y,sum_values,rev ] = mapSsd( mapA, mapB )
%
% MAPSSD searches the larger map for the first instance of a sub-map
%                that is â€œclosestâ€? (least error SSD) to the corresponding sub-image.
%
%       The function takes two input arguments (map1,map2) and provides four output values (x,y,sum_values_temp,rev).
%       map1            - The larger map that is being searched on.
%       map2            - The smaller or equal sized sub-map that is being searched.
%       x               - location of the first Row coordinates in the larger image (image).
%       y               - location of the first Column coordinates in the larger image (image).
%       sum_values_temp - least error SSD value
%       rev             - Flag which gives information on whether rotation should be done clockwise or anticlockwise
%
% By Vishnunandan Venkatesh and Pawan Rao ECET581
%% Initial Declarations
I_size = length(map1);% size of image
P_size = length(map2);% size of imagePatch

sum_values = 0;% variable used for summation
sum_values_temp = Inf; % variable used to find the min error value. This variable stores the previous minimum error value
values = [];% variable used for storing the error and location values of each element
flip_flag = 0;% variable to check if we have to look for a match by flipping the maps i.e rotating the other way around
flip_constant= 2;%variable used to run while loop twice

%% The while loop runs twice. 
%  First iteration ssd is found for rotation assumed to be clockwise and second iteration ssd is found for rotation assumed to be anticlockwise i.e we flip the maps

while (flip_flag < flip_constant)
    
    %% Flip images
    if flip_flag ==1
        map1 = flip(map1);
        map2 = flip(map2);
        
    end
    
    %% Computing the Sum of Absolute Value error and finding the Locations x,y.
    for i = 1:round(I_size*(8/100)) % the limits are chosen such that the i value does not exceed the boundaries and the value i ranges are a percentage of the overalm map provided
        
        
        
        diff_values = (map1(i:P_size)-map2(1:P_size-i+1));% absolute difference values
        sum_values = sum(diff_values.^2); % sum of square of all the absolute differences
        
        if min(sum_values_temp,sum_values)==sum_values % Find the min error value of the current ith and jth area being searched Vs the previous min value
            
            sum_values_temp = sum_values;
            x = i;% the row location being returned
            y = 1;% the column location being returned. 
            if flip_flag ==1
               
                rev = 1; % if min ssd value is found when we assume anticlockwise matching we set this flag to 1.
                
            else
                rev = 0; % if min ssd value is found when we assume clockwise matching we set this flag to 1.
            end
            
            
            
            
            sum_values = 0;
            
        end
        
    end
    flip_flag=flip_flag+1;
end
