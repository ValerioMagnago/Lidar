function [ logic ] = pointInBB( point, BB )
%POINTINBB Summary of this function goes here
%   Detailed explanation goes here

x = point(1);
y = point(2);

logic = x<BB(1,2) && x>BB(1,1) && ...
            y<BB(2,2) && y>BB(2,1);

end

