function [ LaneView ] = segmentsTrapezioidalFOVintersec( segments, FOV)


% Function to define the FOV of the Kinekt

%     \                 /
%   7  \      4        /   1
%  _____\_____________/_______
%        \***********/              ^
%     8   \****5****/   2           |
%          \*******/                | R
% x_axs ----+-----+-------          |
%  ^         \ 6 /         ^        |
%  |    9     \ /   3      |r       |
%  |           v           v        v
%       alpha is camera angle aperture
segment_number = numel(segments)/4;
LaneView = zeros(2,2,0);
for k=1:segment_number
    % Select the segment
    pts       = segments(:,:,k);
    
    [xi,yi] = polyxpoly(pts(1,:),pts(2,:),FOV.points(1,FOV.trapezoidalID),FOV.points(2,FOV.trapezoidalID));
    
%     figure(100); clf;
%     hold on;
%     plotFOV( FOV );
%     plot(pts(1,:),pts(2,:));
%     text(pts(1,1),pts(2,1),'1');
%     text(pts(1,2),pts(2,2),'2');
%     
    
    if(numel(xi)==2)
        LaneView = cat(3,LaneView,[xi';yi']);        
    else      
        
        pts_trasl = pts - FOV.points(:,FOV.centerID);
        % [start point, end point]
        
        
        % logic if point belong to sector 5
        logic = false(1,2); % first row is mult second is offset        
        for i=1:2
            r = sqrt(pts_trasl(1,i)^2+pts_trasl(2,i)^2); % distance squared
            theta = mod(atan2(pts_trasl(2,i),pts_trasl(1,i)) - FOV.theta,2*pi);

            if(FOV.r<r && r<FOV.R)
                if(FOV.alpha/2 > theta || theta > (2*pi-FOV.alpha/2))                
                    logic(i) = 1;
                end    
            end
        end
        
        
        if(numel(xi)==1)            
           if(sum(logic)==1)
               p1 = pts(:,logic);
               p2 = [xi;yi];
               LaneView = cat(3,LaneView,[p1,p2]); 
           else
               error('bad things happens v1');               
           end
        else
            % no intersection xi founded
            if(sum(logic)==2)
               LaneView = cat(3,LaneView,pts); 
            end               
        end
    end
    
    
    
    
    
end

end