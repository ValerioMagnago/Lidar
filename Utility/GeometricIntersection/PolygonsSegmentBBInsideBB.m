function [ segments ] = PolygonsSegmentBBInsideBB( polygons, BB )
%POLYGONSSEGMENTINSIDEBB Summary of this function goes here
%   Detailed explanation goes here

n_polig = numel(polygons);
segments = [];
for i=1:n_polig
    polygon = polygons{i};
    xv = polygon(1,:);
    yv = polygon(2,:);
    
    % Cicliamo sui lati del poligono
    for k=1:(numel(xv)-1)
        x = [min(xv(k:k+1)),max(xv(k:k+1))];
        y = [min(yv(k:k+1)),max(yv(k:k+1))];
        
        % Controllo se almeno uno degli estremi è nell'bounding box
        if BB(1,1)<x(2) && BB(1,2)>x(1) && ...
            BB(2,1)<y(2) && BB(2,2)>y(1)
            segments =  cat(3,segments,[xv(k:k+1);yv(k:k+1)]);
        end
    end    
end

end

