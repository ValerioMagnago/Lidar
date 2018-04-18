function [ logic ] = pointInTriangle( point, triang )
%POINTINTRIANGLE Summary of this function goes here
%   Detailed explanation goes here
    b1 = sign(point, triang(:,1), triang(:,2)) < 0;
    b2 = sign(point, triang(:,2), triang(:,3)) < 0;
    b3 = sign(point, triang(:,3), triang(:,1)) < 0;
    
    logic = ((b1 == b2) && (b2 == b3));
end


function [ out] = sign(p1,p2,p3)
    out = (p1(1)-p3(1))*(p2(2)-p3(2)) - (p2(1)-p3(1))*(p1(2)-p3(2));
end