function [lidarArea] = defineLidarArea(R)
%DEFINELIDARAREA Summary of this function goes here
%   Detailed explanation goes here

lidarArea.radius = R;
% lidarArea.theta_min = t_min;
% lidarArea.theta_max = t_max;
% thetas = t_min:t_inc:(t_max+t_inc/2);
% lidarArea.p_x = R*cos(thetas);
% lidarArea.p_y = R*sin(thetas);

lidarArea.limits = [-R,+R;-R,+R];

% Position of the lidar and orientation
lidarArea.center = [0;0];
lidarArea.orientation = 0;

end

