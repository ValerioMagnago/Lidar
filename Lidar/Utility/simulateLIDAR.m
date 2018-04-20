function [lidarScan] = simulateLIDAR(LidarScanArea,obstaclesTree,state)
%SIMULATELIDAR Summary of this function goes here
%   Detailed explanation goes here
    % Traslate the FOV of the main particle
global DEBUG
DEBUG = false;

[ LidarScanArea ] = moveLidarArea( LidarScanArea, state(3), state(1:2,1));    

% Looking for obstacles intersection
[ WallSegmentsInFOV, ~] = findSegmentInLidarArea( LidarScanArea,obstaclesTree);

% Search the segments in foreground
LaneInView = findForegroundSegments(WallSegmentsInFOV,state(1:2,1),LidarScanArea.radius);

% Generate final point
lidarScan = generateLidar(LaneInView,state(3));
    
end

