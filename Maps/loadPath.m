function [ obstaclesTree ] = loadPath(mapNape)

% Locate the map
filename = which(mfilename);
[pathstr,~,~] = fileparts(filename);
fileMap = [pathstr,'/Archive/',mapNape];
scale = 1;

% Read the file
[ WallObstacles,xRangeObstacles,yRangeObstacles ] = loadMapObstacles( fileMap,scale );

% Creeate obstacles tree
[ obstaclesTree ] = createTree( WallObstacles, xRangeObstacles, yRangeObstacles);



end
