function [ WallSegmentsInLidarArea,WallObstacleIdS ] = findSegmentInLidarArea( area,obstacle)
% --------------------------------------------------------------
%
%  Function to compute the wall intersection with the LIDAR area range
%
% --------------------------------------------------------------
%======================================================================%
%                                                                      %
%  Autors: Valerio Magnago                                             %
%          University of Trento                                        %
%          valerio.magnago@unitn.it                                    %
%                                                                      %
%  Date:  24/10                                                        %
%  Version: 0.1                                                        %
%======================================================================%

% Find which obstacles bounding box are in the LIDAR bounding box
WallObstacleIdS   = polygonBBinObstacleBB(area.limits,obstacle.obstaclesTree);

% Find which segments bounding box of the founded obstacles are in the
% LIDAR area range
[ WallSegmentsInLidarArea ]  = PolygonsSegmentBBInsideBB( obstacle.obstacles(WallObstacleIdS,2), area.limits );

end

