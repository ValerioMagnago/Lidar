function [ out ] = pointIsOnObstacleBB( point, obstaclesTree)
%ISONOBSTACLE Summary of this function goes here
%   Detailed explanation goes here

x = point(1);
y = point(2);

indici = 1;
out = [];
% check if x and y belong to bounding box
while ~isempty(indici)
    indice = indici(1);
    indici = indici(2:end);
    
    logic = pointInBB( point, obstaclesTree.limits(:,:,indice));
    
    if logic
        if obstaclesTree.Leaf(indice)==0
            indici = [indici, obstaclesTree.Sons{indice}];
        else
            out    = [out,    obstaclesTree.Leaf(indice)];
        end        
    end
    
    % if out = [] no intersection with obstacles
    % if obstacles array of out is not empty --> intersection with that obstacles
end

