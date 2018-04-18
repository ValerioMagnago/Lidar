function [ out ] = polygonBBinObstacleBB( polygonBB, obstaclesTree )
%POLYGONBBINOBSTACLEBB Summary of this function goes here
%   Detailed explanation goes here

xRange = polygonBB(1,:);
yRange = polygonBB(2,:);

indici = 1;
out = [];
% check if x and y belong to bounding box
while ~isempty(indici)
    indice = indici(1);
    indici = indici(2:end);
    if xRange(1)<obstaclesTree.limits(1,2,indice) && xRange(2)>obstaclesTree.limits(1,1,indice) && ...
            yRange(1)<obstaclesTree.limits(2,2,indice) && yRange(2)>obstaclesTree.limits(2,1,indice)
        if obstaclesTree.Leaf(indice)==0
            indici = [indici, obstaclesTree.Sons{indice}];
        else
            out = [out,obstaclesTree.Leaf(indice)];
        end
        
    end
    
    % if out = [] no intersection with obstacles
    % if obstacles array of out is not empty --> intersection with that obstacles
end


end

