function [ logic ] = pointInFOV( p2, FOV, LaneView)
%POINTINFOV  Check if the point is in the FOV
%   Detailed explanation goes here

% Check if the point is in the FOV bounding box, in the smallest rectangle that
% containts the fov
logic = pointInBB(p2,FOV.limits);

if logic  % if point is in bounding box
    
    % Check if is more precisely is in the triangular FOV    
    logic = pointInTriangle( p2 , FOV.points(:,FOV.triangID));
    
    if logic 
        
        % Check if there are some walls that occludes the fov        
        segment1 = [FOV.points(:,FOV.centerID),p2];  % segment from the origin of the fov to the point
        
        for kk=1:numel(LaneView)/4  %  iterate over all segments
            % Walls segment
            segment2 = LaneView(:,:,kk);
            
            % Find if there is any intersection segment1, segment2
            [ intersection ] = segmentIntersection( segment1,segment2 );
            
            % Check if intersection is found
            if ~isempty(intersection) 
                logic = 0; % point is occluded
                break; % stop the for don't need to check others walls
            end
        end
        versore = [cos(FOV.theta),sin(FOV.theta)];
        dist = versore*diff(segment1')';
        
        logic = dist > FOV.r*cos(FOV.alpha/2);
%         logic = inpolygon(p2(1),p2(2),FOV.points(1,FOV.trapezoidalID),FOV.points(2,FOV.trapezoidalID));        
        
%         figure();
%         hold on;
%         plot(FOV.points(1,FOV.trapezoidalID),FOV.points(2,FOV.trapezoidalID));
%         plot(p2(1),p2(2),'*');
    end
end


end

