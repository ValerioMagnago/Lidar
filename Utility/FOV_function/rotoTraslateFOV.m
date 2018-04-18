function [ FOV ] = rotoTraslateFOV(  FOV, theta, p )
%ROTOTRASLATEFOV Summary of this function goes here
%   Detailed explanation goes here



% APPROCCIO CON MATRICE DI ROTAZIONE
FOV.theta   = theta;
FOV.points = rotoTranslateP(FOV.points,theta,p);

FOV.limits  = [min(FOV.points(1,:)),max(FOV.points(1,:));...
               min(FOV.points(2,:)), max(FOV.points(2,:))];  % bounding box of the FOV

end

