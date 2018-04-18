function [ in ] = pointsInsidePolygons( polygons, point )
%POINTINSIDEPOLYGON Summary of this function goes here
%   Detailed explanation goes here
n_polig = numel(polygons);
in = zeros(1,n_polig);
for i=1:n_polig
    polygon = polygons{i};
    xv = polygon(1,:);
    yv = polygon(2,:);

    in(i) = inpolygon(point(1),point(2),xv,yv);
end

end

