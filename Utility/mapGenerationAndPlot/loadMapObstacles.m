function [ obstacles, x_lim, y_lim ] = loadMapObstacles( file, scale)
%LOADMAPOBSTACLES Summary of this function goes here
%   Detailed explanation goes here

obstacles = {};

fid = fopen(file,'r');

if fid==-1
    error(['Impossible to open the file: ',file]);
end

y_lim = [inf,-inf];
x_lim = [inf,-inf];

% figure; hold on; axis equal;
while ~feof(fid)
     line = fgets(fid);
     sz = sscanf(line, '%d');
     xV = [];
     yV = [];
     for i=1:sz
         line = fgets(fid);
         pts = sscanf(line, '%f %f');
         pts = pts/scale;
         % Create bounding box
         xMin = min(xV);
         yMin = min(yV);
         xMax = max(xV);
         yMax = max(yV);
         
         if xMin<x_lim(1)
             x_lim(1) = xMin;
         end
         
         if yMin<y_lim(1)
             y_lim(1) = yMin;
         end
         
         if xMax>x_lim(2)
             x_lim(2) = xMax;
         end
         
         if yMax>y_lim(2)
             y_lim(2) = yMax;
         end
         
         
         xBound = [xMin,xMin,xMax,xMax,xMin];         
         yBound = [yMin,yMax,yMax,yMin,yMin];         
         xV = [xV pts(1)];
         yV = [yV pts(2)];
     end
     Pmean = [mean(xBound(1:end-1));mean(yBound(1:end-1))];
     obstacles(end+1,1) = {Pmean};    
     obstacles(end,2) = {[xV;yV]};
     obstacles(end,3) = {[xBound;yBound]};     
end
fclose(fid);

%% Inseriamo bordi mappa
% offset_mappa = 2;
% x_lim(1) = x_lim(1)-offset_mappa;
% x_lim(2) = x_lim(2)+offset_mappa;
% y_lim(1) = y_lim(1)-offset_mappa;
% y_lim(2) = y_lim(2)+offset_mappa;
% 
% % Spessore muro
% sp_mr = 3; %[m]
% % Aggiunta bordo nero sopra
% PY = [y_lim(2)-sp_mr,y_lim(2)];
% PX = x_lim;
% P = [PX(1),PX(1),PX(2),PX(2);PY(1),PY(2),PY(2),PY(1)];
% Pmean = mean(P(:,1:end),2);
% obstacles(end+1,1) = {Pmean};
% obstacles(end,2) = {P(:,[1:4,1])};
% obstacles(end,3) = {P(:,[1:4,1])};  
% 
% % Aggiunta bordo nero sotto
% PY = [y_lim(1),y_lim(1)+sp_mr];
% PX = x_lim;
% P = [PX(1),PX(1),PX(2),PX(2);PY(1),PY(2),PY(2),PY(1)];
% Pmean = mean(P(:,1:end),2);
% obstacles(end+1,1) = {Pmean};
% obstacles(end,2) = {P(:,[1:4,1])};
% obstacles(end,3) = {P(:,[1:4,1])};     
% 
% % Aggiunta bordo nero destra
% PX = [x_lim(2)-sp_mr,x_lim(2)];
% PY = y_lim;
% P = [PX(1),PX(1),PX(2),PX(2);PY(1),PY(2),PY(2),PY(1)];
% Pmean = mean(P(:,1:end),2);
% obstacles(end+1,1) = {Pmean};
% obstacles(end,2) = {P(:,[1:4,1])};
% obstacles(end,3) = {P(:,[1:4,1])};     
% 
% % Aggiunta bordo nero sinistra
% PX = [x_lim(1),x_lim(1)+sp_mr];
% PY = y_lim;
% P = [PX(1),PX(1),PX(2),PX(2);PY(1),PY(2),PY(2),PY(1)];
% Pmean = mean(P(:,1:end),2);
% obstacles(end+1,1) = {Pmean};
% obstacles(end,2) = {P(:,[1:4,1])};
% obstacles(end,3) = {P(:,[1:4,1])}; 

end

