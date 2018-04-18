function plotObstaclesOriginal(file, style)

if nargin<2
   style = 'b'; 
end

fid = fopen(file,'r');

% figure; hold on; axis equal;

while ~feof(fid)
     line = fgets(fid);
     sz = sscanf(line, '%d');
     xV = [];
     yV = [];
     for i=1:sz
         line = fgets(fid);
         pts = sscanf(line, '%f %f');
         xV = [xV pts(1)];
         yV = [yV pts(2)];
     end
     plot(xV,yV,style);
end

fclose(fid);

end
