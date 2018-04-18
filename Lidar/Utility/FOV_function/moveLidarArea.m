function [LidarScanArea] = moveLidarArea(LidarScanArea,theta,p0)
%MOVELIDARAREA Summary of this function goes here
%   Detailed explanation goes here

LidarScanArea.center = p0;
LidarScanArea.orientation = theta;
LidarScanArea.limits = [-LidarScanArea.radius,+LidarScanArea.radius;-LidarScanArea.radius,+LidarScanArea.radius] + p0(:,[1,1]);
end

