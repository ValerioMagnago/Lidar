% --------------------------------------------------------------
%
%   Display the wall in the FOV
%
% --------------------------------------------------------------
%======================================================================%
%                                                                      %
%  Autors: Valerio Magnago                                             %
%          Daniele Fontanelli                                          %
%          University of Trento                                        %
%          valerio.magnago@unitn.it                                    %
%                                                                      %
%  Date:  24/10                                                        %
%  Version: 0.1                                                        %
%======================================================================%


%% Init console
% Go to script folder
filename = which(mfilename);
[pathstr,name,ext] = fileparts(filename);
cd(pathstr);

% Clear console and variables
clc;
clear all;
close all;

% Add path
addpath(genpath('./Utility'));

%set(0,'DefaultFigureWindowStyle','docked')


%**************************************************************************
%% Parameters definition
%**************************************************************************

% Print to screen what the script is doing
sec_numb = 1; fprintf('%d) Loading users params\n',sec_numb); sec_numb = sec_numb + 1;


% Define a circular AREA
fake_lid = generateLidar([],0);
R = fake_lid.range_max;
LidarScanArea = defineLidarArea(R);


% mapName = './MapsAndPaths/Povo1_floor1.txt';
% scale = 3;

mapName = './Maps/Povo2_floor1.txt';
% mapName = './Maps/demoKinektMap.txt';
scale = 1;


%**************************************************************************
%% Load obstacles map and creeate obstacles tree
%**************************************************************************
fprintf('%d) Loading obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;
% Load Obstacles
[ WallObstacles,xRangeObstacles,yRangeObstacles ] = loadMapObstacles( mapName,scale );

%**************************************************************************
%% Plot map
%**************************************************************************
fprintf('%d) Plotting the map\n',sec_numb); sec_numb = sec_numb + 1;
global map_fig;
map_fig = figure(1); clf; hold on;   % care map_fig is used as global variable inside some function
title('Map','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');
ylabel('y [m]','Interpreter','latex');
axis equal;
fig_1_ObstPoly = plotObstacles(WallObstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);
%**************************************************************************
%% Creeate obstacles tree
%**************************************************************************
fprintf('%d) Createing obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;
[ obstaclesTree ] = createTree( WallObstacles, xRangeObstacles, yRangeObstacles);

%pause();
%**************************************************************************
%% Create random state of particle and compute information
%**************************************************************************
fprintf('%d) Create random state \n',sec_numb); sec_numb = sec_numb + 1;
rng(0,'twister');  % initialize the random number generator to make the results in this example repeatable
state = zeros(3,1); % x,y,theta of the system

finalPlot = true;

ripetizioni = 10000;
tic
for i = 1:ripetizioni
    % x, y, theta position of the particle
    state(1) = diff(obstaclesTree.xRange)*rand() + obstaclesTree.xRange(1);  % x
    state(2) = diff(obstaclesTree.yRange)*rand() + obstaclesTree.yRange(1);  % y
    state(3) = 2*pi*rand(); % theta

    % Traslate the FOV of the main particle
    [ LidarScanArea ] = moveLidarArea( LidarScanArea, state(3), state(1:2,1));
    
    
    % Looking for obstacles intersection
    [ WallSegmentsInFOV, WallObstacleIdS] = findSegmentInLidarArea( LidarScanArea,obstaclesTree);

    % Search the segments in foreground
    LaneInView = findForegroundSegments(WallSegmentsInFOV,state(1:2,1),LidarScanArea.radius);
    
    % PLOT THETA COVERAGE
    if false
       figure();       
       hold on;
       for kk=1:size(LaneInView,1)
            plot(LaneInView(kk,2:3),[1 1]*kk);        
       end
    end
    
    lidarScan = generateLidar(LaneInView,state(3));
    
    
    %% PLOTTT
    if finalPlot
        tag_to_del = {'laser_point','laser_range','vehicle'};
        for kk=1:numel(tag_to_del)
            to_del = findobj('Tag',tag_to_del{kk});
            for kkk = 1:numel(to_del)
               delete(to_del(kkk)); 
            end            
        end 

        thetas = state(3)+linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));

        plot(state(1),state(2),'g.','markersize',6,'Tag','vehicle');
        plot(state(1)+[0,cos(state(3))]*1,state(2)+[0,sin(state(3))]*1,'g-','Tag','vehicle');
        plot(lidarScan.ranges.*cos(thetas)+state(1),lidarScan.ranges.*sin(thetas)+state(2),'r*','Tag','laser_point');
        plot(lidarScan.range_max*cos(thetas)+state(1),lidarScan.range_max*sin(thetas)+state(2),'--m','Tag','laser_range');
        plot(lidarScan.range_max*[0,cos(thetas(1))]+state(1),lidarScan.range_max*[0,sin(thetas(1))]+state(2),'m--','Tag','laser_range');
        plot(lidarScan.range_max*[0,cos(thetas(end))]+state(1),lidarScan.range_max*[0,sin(thetas(end))]+state(2),'m--','Tag','laser_range');

        pause();
    end
end
elapsed = toc;
fprintf('tempo medio iterazione %d \n',elapsed/ripetizioni);


