function [ BB_range ] = createBoundingBox( obstaclesBooundingbox, index )
%CREATENODE Create the bounding box that best fit the obstaclesBoundingbox
%desidered
%lung = numel(index);
xMax = max(obstaclesBooundingbox(1,2,index));   % max of the max x value
xMin = min(obstaclesBooundingbox(1,1,index));   % min of the min x value

yMax = max(obstaclesBooundingbox(2,2,index));   % max of the max x value
yMin = min(obstaclesBooundingbox(2,1,index));   % min of the min x value

x_lim = [xMin,xMax];
y_lim = [yMin,yMax];

BB_range = [x_lim;y_lim];

end

