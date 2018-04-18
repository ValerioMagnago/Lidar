function [h] = plotObstacles(obstacles, fillFlag, style1)

h = [];
for i=1:size(obstacles,1)
     xV = obstacles{i,1}(1,:);
     yV = obstacles{i,1}(2,:);    
     if fillFlag==0         
        h = [h,plot(xV,yV,style1{1},'linewidth',style1{2})];
     else
        h = [h,fill(xV,yV,style1{1},'facealpha',style1{2})];
     end
end

end
