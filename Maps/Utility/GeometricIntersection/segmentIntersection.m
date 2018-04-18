function [ out,lambda1,lambda2 ] = segmentIntersection( segment1,segment2 )
%SEGMENTINTERSECTION Summary of this function goes here
%   Detailed explanation goes here

TOL = 1e-6;


v1 = segment1(:,2) - segment1(:,1);
v2 = segment2(:,2) - segment2(:,1);

deltaP = segment2(:,1) - segment1(:,1);
Mat = [v1,-v2];

determinant = Mat(1,1)*Mat(2,2)- Mat(1,2)*Mat(2,1);

if abs(determinant)<1e-6 % se determinand è vicino a 0
    out = [];
    lambda2 = inf;
    lambda1 = inf;
else
%     lambda(1) = 1/determinant*(Mat(2,2)*deltaP(1) - Mat(1,2)*deltaP(2));
%     lambda(2) = 1/determinant*(-Mat(2,1)*deltaP(1) + Mat(2,2)*deltaP(2));

    lambda = Mat\deltaP;    
    lambda1 = lambda(1);
    lambda2 = lambda(2);
    
    if (lambda2 >= 0-TOL && lambda2 <= 1+TOL && lambda1 >= 0-TOL && lambda1 <= 1+TOL)
    %if (s > 0 && s < 1 && t > 0 && t < 1)
        out = segment1(:,1) + lambda1*v1;
    else
        out = [];
    end
end

%
% delta_x = segment1(1,1) - segment2(1,1);
% delta_y = segment1(2,1) - segment2(2,1);
%
% s1_x = segment1(1,2) - segment1(1,1);
% s1_y = segment1(2,2) - segment1(2,1);
%
% s2_x = segment2(1,2) - segment2(1,1);
% s2_y = segment2(1,2) - segment2(1,1);
%
% s = (-s1_y * delta_x + s1_x * delta_y) / (-s2_x * s1_y + s1_x * s2_y);
% t = ( s2_x * delta_y - s2_y * delta_x) / (-s2_x * s1_y + s1_x * s2_y);
%
% if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
%     out = segment1(:,1) + t*[s1_x;s1_y];
% else
%     out = [];
% end

% figure(100);
% hold off;
% plot(segment1(1,:),segment1(2,:),'b')
% hold on;
% plot(segment2(1,:),segment2(2,:),'r')
% plot(out(1),out(2),'*');
% plot([out(1),segment2(1,1)],[out(2),segment2(2,1)],'--r')

end

