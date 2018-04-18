function [ prova ] = divideTree( xRangeBB, yRangeBB, boundingBoxs, centralPoints, activeBoxIndexs, orientation)
%DIVIDETREE Divide the current bounding box in 2 subboundings box
%   Detailed explanation goes here

if numel(activeBoxIndexs) == 2
    activeBoxIndexs1 = activeBoxIndexs(1);
    xBound1 = boundingBoxs(1,:,activeBoxIndexs1);
    yBound1 = boundingBoxs(2,:,activeBoxIndexs1); 
    
    activeBoxIndexs2 = activeBoxIndexs(2);
    xBound2 = boundingBoxs(1,:,activeBoxIndexs2); 
    yBound2 = boundingBoxs(2,:,activeBoxIndexs2);  
else
    if orientation==0   % divide along x
        xdivision = xRangeBB(1) + (xRangeBB(2)-xRangeBB(1))/2;
        logic = (centralPoints(activeBoxIndexs,1) < xdivision);
    else  % divide along y
        ydivision = yRangeBB(1) + (yRangeBB(2)-yRangeBB(1))/2;
        logic = (centralPoints(activeBoxIndexs,2) < ydivision);
    end
    
    activeBoxIndexs1 = activeBoxIndexs(logic);
    [ xBound1, yBound1 ] = createNode( boundingBoxs, activeBoxIndexs1 );
    
    
    activeBoxIndexs2 = activeBoxIndexs(not(logic));
    [ xBound2, yBound2 ] = createNode( boundingBoxs, activeBoxIndexs2 );
end

prova = [[xRangeBB;yTangeBB],];
% 

end

