function [ LaneView ] = segmentsTriangFOVintersec( segments, FOV)
%SEGMENTSPLYGONINTERSECTION Summary of this function goes here
%   Detailed explanation goes here

% points of the triangular FOV
triPoints = FOV.points(:,FOV.triangID);

point_number = numel(segments)/2;


LaneView = [];
for k=1:(point_number/2)
    % Select the segment 
    A = segments(:,1,k);  % start point   
    B = segments(:,2,k);  % end point   
   
    delta = B - A;
    deltaPerp = [delta(2);-delta(1)];   % vettore perpendicolare
    
    segnoPunti = zeros(3,1);
    count = zeros(3,1);
    for i = 1:3
        tmp = (triPoints(:,i)-A)'*deltaPerp;
        if(tmp<0)
            segnoPunti(i) = -1;
            count(1) = count(1) + 1;
        elseif(tmp>0)
            segnoPunti(i) = 1;
            count(2) = count(2) + 1;
        else
            segnoPunti(i) = 0;
            count(3) = count(3) + 1;
        end
    end
    
    if count(2)==2 || count(1) == 2
       logic = (segnoPunti==1); % indice dei punti che stanno dalla stessa parte della retta
       if sum(logic)==2
            pointLatoA = triPoints(:,logic); % 2 punti da questo lato
            pointLatoB = triPoints(:,not(logic));
       else
           % swap
            pointLatoB = triPoints(:,logic);  
            pointLatoA = triPoints(:,not(logic));           
       end
       
       % in lato A ci sono 2 punti, in B 1   (totale 3 vertici triangolo)
       
       % Vettore dei 2 segmenti che vengono intersecati 
       v1 = pointLatoA(:,1)-pointLatoB;
       v2 = pointLatoA(:,2)-pointLatoB;
       
       % Perpendicolar Vector
       v1p = [v1(2);-v1(1)];
       v2p = [v2(2);-v2(1)];
       
       % Vector in the middle of the 2;
       midV = (v1+v2)/2;
       
       dirMid1 = midV'*v1p;
       dirA1 = (A-pointLatoB)'*v1p;
       dirB1 = (B-pointLatoB)'*v1p;
       logic1A = ((dirA1*dirMid1) >=0);   % se vero punto A è verso il centro rispetto alla linea 1
       logic1B = ((dirB1*dirMid1) >=0);   % se vero punto B è verso il centro rispetto alla linea 1
       
       dirMid2 = midV'*v2p;
       dirA2 = (A-pointLatoB)'*v2p;
       dirB2 = (B-pointLatoB)'*v2p;
       logic2A = ((dirA2*dirMid2) >=0);   % se vero punto A è verso il centro rispetto alla linea 2
       logic2B = ((dirB2*dirMid2) >=0);   % se vero punto B è verso il centro rispetto alla linea 2
      
       % calcoliamo distanza punto da segmento e valutiamo se è nella
       % direzione dell'interno del triangolo (direzione verso midV)
       if logic1A && logic2A && logic1B && logic2B
           LaneView = cat(3,LaneView,[A,B]);
       % caso tutti a 1 gestito --> qualcuno deve essere a 0
       elseif (logic1A == logic1B) && (logic2A == logic2B)   % entrambi i punti fuori da triangolo da stessa parte
           continue;
           
       elseif logic1B && logic1A   
           % A cavallo linea 2
           % un intersezione con FOV un punto interno sono gli intersec
           % points
           mat2 = [v2,-delta];
           lambda = inv(mat2)*(A - pointLatoB);
           inters2 = v2*lambda(1) + pointLatoB;
           if logic2A     % punto A è dentro
              inters1  = A; 
           else   % punto B è dentro
              inters1  = B;  
           end                      
           LaneView = cat(3,LaneView,[inters1,inters2]);
           
       elseif logic2A && logic2B
           % A cavallo linea 1
           % un intersezione con FOV un punto interno sono gli intersec
           % points
           mat1 = [v1,-delta];
           lambda = inv(mat1)*(A - pointLatoB);
           inters1 = v1*lambda(1) + pointLatoB;
           if logic1A     % punto A è dentro
              inters2  = A; 
           else   % punto B è dentro
              inters2  = B; 
           end
           LaneView = cat(3,LaneView,[inters1,inters2]);           
       else  % intersezione da ambo le parti con FOV
           
           mat1 = [v1,-delta];
           mat2 = [v2,-delta];
           
           % TODO convertire inversa in operazioni semplici
           lambda = inv(mat1)*(A - pointLatoB);
           inters1 = v1*lambda(1) + pointLatoB;
           
           lambda = inv(mat2)*(A - pointLatoB);
           inters2 = v2*lambda(1) + pointLatoB;
           
           LaneView = cat(3,LaneView,[inters1,inters2]);
       end
    elseif count(3)>0        % la linea passa sul vertice del triangolo
%        fprintf('da implementare, la linea passa sul vertice del triangolo --> CASO IGNORATO PER ORA');  
%     else
%         % Lina non interseca
%         fprintf('linea selezionata non interseca FOV');  
    end
    
   
    

end

end

