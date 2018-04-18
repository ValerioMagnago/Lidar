function [ LaneView ] = segmentsTrapezioidalFOVintersecAfterTriangularIntersec( segments, FOV)


%     3_____________4
%      \           /      ^
%       \         /       |
%        \_______/        | R
%        2       1    ^   |
%                     |r  |
%            v        v   v
%
%            ^ xAxis
%            |
% yAxis  <---+
%
% alpha is camera angle aperture

% poichè è già stata fatta l'intersezione con il FOV triangolare l'unica
% possibilità rimasta di intersezione è tra punti 1 e 2
% 
versore = [cos(FOV.theta),sin(FOV.theta)]; % versore asse x veicolo in sistema di riferimento globale
min_dist = FOV.r*cos(FOV.alpha/2);

segmentToadd = zeros(2,2,0);  % segments to add 

for z=1:(numel(segments)/4)    
    segment = segments(:,:,z);
    d(1) = versore * (segment(:,1)-FOV.points(:,FOV.centerID));
    d(2) = versore * (segment(:,2)-FOV.points(:,FOV.centerID));
    
    if(d(1)<min_dist)
        if(d(2)>min_dist) 
            seg_1 = FOV.points(:,FOV.trapezoidalID(1:2));
            [point,~,~] = segmentIntersection(seg_1,segment);
            segmentToadd = cat(3,segmentToadd,[point,segment(:,2)]);
        end
    else
        if(d(2)<min_dist)
            seg_1 = FOV.points(:,FOV.trapezoidalID(1:2));
            [point,~,~] = segmentIntersection(seg_1,segment);
            segmentToadd = cat(3,segmentToadd,[point,segment(:,1)]);
        else
            segmentToadd = cat(3,segmentToadd,segment(:,:));
        end
    end
end
LaneView = segmentToadd;
end