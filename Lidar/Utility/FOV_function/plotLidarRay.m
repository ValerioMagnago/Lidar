function [ ] = plotFOV( FOVrot, Tag )
%PLOTFOV Summary of this function goes here
%   Detailed explanation goes here

% selectedObjs = findobj('Tag','fovPlot');
% for i=1:numel(selectedObjs)
%     delete(selectedObjs(i));
% end

index = FOVrot.triangID(FOVrot.triangID ~= FOVrot.centerID);
for i=index
   plot([FOVrot.points(1,FOVrot.centerID),FOVrot.points(1,i)],[FOVrot.points(2,FOVrot.centerID),FOVrot.points(2,i)],'y--','Tag',Tag); 
end
plot(FOVrot.points(1,FOVrot.centerID),FOVrot.points(2,FOVrot.centerID),'k.','markersize',15,'Tag',Tag); % camera position
fill(FOVrot.points(1,FOVrot.trapezoidalID),FOVrot.points(2,FOVrot.trapezoidalID),'r','facealpha',.6,'Tag',Tag);
end

