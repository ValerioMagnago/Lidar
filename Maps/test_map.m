% --------------------------------------------------------------
%
%   Test map function
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

use_map();









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
use_map();

% Define the name of the file to load
mapName = 'Povo2_floor1.txt';

%**************************************************************************
%% Load the map and organize it as a graph
%**************************************************************************
fprintf('%d) Loading the map in graph\n',1);
obstaclesTree = loadPath(mapName);

%**************************************************************************
%% Plot map
%**************************************************************************
fprintf('%d) Plotting the map\n',2);
global map_fig;
map_fig = figure(1); clf; hold on;   % care map_fig is used as global variable inside some function
title('Map','Interpreter','latex');
xlabel('x [m]','Interpreter','latex');
ylabel('y [m]','Interpreter','latex');
axis equal;
fig_1_ObstPoly = plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);






