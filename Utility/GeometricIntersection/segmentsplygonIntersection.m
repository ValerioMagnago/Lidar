function [ LaneView ] = segmentsplygonIntersection( segments, polygon)
%SEGMENTSPLYGONINTERSECTION Summary of this function goes here
%   Detailed explanation goes here

point_number = numel(segments)/2;
xp = reshape(segments(1,:,:),1,point_number);
yp = reshape(segments(2,:,:),1,point_number);

xv = polygon(1,:);
yv = polygon(2,:);

in = inpolygon(xp,yp,xv,yv);
n_lane = numel(polygon)/2-1;
LaneView = [];

for k=1:(point_number/2)
   % Select the segment 
   segment1 =  segments(:,:,k);   
   p = zeros(2,2);
   kk = 1;
   
   if in(2*(k))~=0 && in(2*(k)-1)~=0  % both points are in inside FOV
       p = segment1;
       LaneView = cat(3,LaneView,p);
   else
       % Check if at least 1 point is included
       if in(2*(k)-1)~=0
           p(:,kk) = segment1(:,1);
           kk = 2;
       elseif in(2*(k))~=0
           p(:,kk) = segment1(:,2); 
           kk = 2;
       end

      % Check intersection with polygon edge
        for i = 1:n_lane
            segment2 = polygon(:,i:i+1);
%           [ intP ] = segmentIntersection( segment1, segment2 );
            [xi,yi] = polyxpoly(segment1(1,:),segment1(2,:),segment2(1,:),segment2(2,:));
            intP = [xi;yi];
            if numel(intP)>0
                p(:,kk) = intP;
                kk = kk+1;            
            end
            if kk == 3    % intersection found
               LaneView = cat(3,LaneView,p);
               break
            end
        end
   end

end

end

