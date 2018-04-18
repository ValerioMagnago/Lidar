function [lineeViste] = findForegroundSegments(segmentList,p0,maxR)
%FINDFOREGRONDSEGMENT Summary of this function goes here
%   Detailed explanation goes here
global DEBUG

if DEBUG
    fig = figure();
    clf;
    hold on;    
    xlim(([-maxR maxR]*1.2+p0(1)))
    ylim(([-maxR maxR]*1.2+p0(2)));
%     axis equal;
    box
    % Draw center
    plot(p0(1),p0(2),'r.','markersize',10);
    
    thetas = linspace(0,2*pi,100);
    plot(maxR*cos(thetas)+p0(1),maxR*sin(thetas)+p0(2),'m--');
    
	ray = [0, 1; 0, 0]*5.6 + p0;
    plot(ray(1,:),ray(2,:),'m--','Tag','ray')
    
    for i=1:size(segmentList,3)
        plot(segmentList(1,:,i),segmentList(2,:,i),'k');
        % Add number to the lines
        p = mean(segmentList(:,:,i),2);        
        text(p(1),p(2),num2str(i))
    end
end

n_linee = numel(segmentList)/4;
if n_linee==0
    lineeViste = [];
    return;
end

n_linee = numel(segmentList)/4;
if n_linee==0
    lineeViste = [];
    return;
end

% Order segment point by angle and then radius
theta = zeros(1,2);
p = zeros(2,2);
lane_list = zeros(2*n_linee,6); % laneIndx, lane_theta, lane_d, point_indx, theta, start_final_point
d = zeros(1,2);

LaneActiveIndx = [];  % the index with the thetas of the lane in foreground
for laneIndx=1:n_linee
    for col=1:2
        p(:,col) = segmentList(:,col,laneIndx)-p0;
        theta(col) = atan2(p(2,col),p(1,col)); % Theta i betwheen -pi and pi
        d(col) = sqrt(p(1,col)*p(1,col)+p(2,col)*p(2,col));
    end
    
    % Order the data
    theta = theta + (theta<0)*2*pi; % Theta e' tra 0 e2*pi
    if(theta(2)<theta(1))
        p(:,[2,1]) = p(:,[1,2]);
        theta([2,1]) = theta([1,2]);
        d([2,1]) = d([1,2]);
        cols = [2,1];
    else
        cols = [1,2];
    end
    
    vett = p(:,1) - p(:,2);
    % Theta seg e' nell intervallo [0 pi]
    theta_seg = atan2(vett(2),vett(1));
    if theta_seg<0
        theta_seg = theta_seg + 2*pi;
    end    
    vers_perp = [cos(theta_seg+pi/2);sin(theta_seg+pi/2)];
    lane_d = (p(:,1)'*vers_perp); % Compute the distance
%     if lane_d < 0
%        warning("non dovrebbero esistere d negative!!") 
%     end

    logic = d>maxR;
    % se sum(logic) == 1 vuol dire che un punto e' dentro uno e 'fuori
    % se sum(logic) == 2 entrambi sono fuori e bisogna controllare se
    % uno e' a destra e uno e' a sx del cerchio
        
    % Control if the segment is parallel to the LIDAR ray without crossing
    % the origin or is outside the lidar range
    if((abs(diff(theta)) < 1e-4 && abs(lane_d)<1e-3)) || (abs(lane_d)>maxR) || (sum(logic) == 2  && not(theta(1)< (pi/2+theta_seg) && theta(2)> (pi/2+theta_seg)))
        % Discard the segments
        lane_list((laneIndx-1)*2+(1:2),1) = [laneIndx,laneIndx]';
        lane_list((laneIndx-1)*2+(1:2),2) = inf * [1,1]';
        lane_list((laneIndx-1)*2+(1:2),3) = inf * [1,1]';
        lane_list((laneIndx-1)*2+(1:2),4) = [1,2]';
        lane_list((laneIndx-1)*2+(1:2),5) = inf * [1,1]';
        lane_list((laneIndx-1)*2+(1:2),6) = d';
        lane_list((laneIndx-1)*2+(1:2),7) = [0,1]';
    else      
        
        

% %         if sum(logic) > 0
% %             % devo saturare sono uno o 2 angoli
% %             [theta_tmp] = findThetafromD(theta_seg,lane_d,maxR);
% %             tmp = [maxR,maxR];
% %             d(logic) = tmp(logic);
% %             theta(logic) = theta_tmp(logic);
% % %             p(:,1) = d(1) * [cos(theta(1));sin(theta(1))];
% % %             p(:,2) = d(2) * [cos(theta(2));sin(theta(2))];
% %         end
        insert_remove = [0,1];
        
        % Segment cross theta=0, clip it, controlliamo che 
        if(theta(1) + pi < theta(2))
            % Store index of lane, theta of the segment, distance of the
            % segment, distance of the point in theta=0 of the segment            
            d_p = findDfromTheta(theta_seg,lane_d,0);            
            LaneActiveIndx = [LaneActiveIndx;[laneIndx,theta_seg,lane_d,d_p]];  
            insert_remove = [1,0];
        end
        
        lane_list((laneIndx-1)*2+(1:2),1) = [laneIndx,laneIndx]';
        lane_list((laneIndx-1)*2+(1:2),2) = theta_seg * [1,1]';
        lane_list((laneIndx-1)*2+(1:2),3) = lane_d * [1,1]';
        lane_list((laneIndx-1)*2+(1:2),4) = cols';
        lane_list((laneIndx-1)*2+(1:2),5) = theta';
        lane_list((laneIndx-1)*2+(1:2),6) = d';
        lane_list((laneIndx-1)*2+(1:2),7) = insert_remove';  
    end
end


%*****************************************************************
% Debug plot linee attualmente attive con numero
%*****************************************************************
if DEBUG
    % Plot clipped lane
    for i=1:2:(numel(lane_list(:,1))-1)
        indx = i + [0,1];       
        if not(lane_list(indx(1),2) == inf)
            P(:,1) = lane_list(indx(1),6)*[cos(lane_list(indx(1),5)); sin(lane_list(indx(1),5)) ] + p0;
            P(:,2) = lane_list(indx(2),6)*[cos(lane_list(indx(2),5)); sin(lane_list(indx(2),5)) ] + p0;
            plot( P(1,:), P(2,:) ,'g','Tag','selected');
        end
        
    end

    for kk = 1:size(LaneActiveIndx,1)
        activ = LaneActiveIndx(kk,1)';
        plot(segmentList(1,:,activ),segmentList(2,:,activ),'r','Tag','active')
    end
    
   pause();
end

lane_list_sorted = sortrows(lane_list,5); % Order the point reling on points theta



%*****************************************************************
% Detect the segment in foreground
%*****************************************************************
if not(isempty(LaneActiveIndx))
    LaneActiveIndx   = sortrows(LaneActiveIndx,4); % Order the point reling on distance in theta = 0 of the segment
    LaneViewedIndx = [LaneActiveIndx(1,1),0];  % the list of the active lane, with the angle at which they are active
else
    LaneViewedIndx = [];
end
sortIndx = 0; % index to move on the sorted data

EPS = 1e-4;

% loop on the sorted data
esci = false;
while sortIndx < numel(lane_list_sorted(:,1))    
    % Extract theta from lane_list
    while true
        sortIndx = sortIndx + 1;
        theta = lane_list_sorted(sortIndx,5);
        if theta> 2*pi
            esci = true;
            break;
        end
        insert_remove_flag = lane_list_sorted(sortIndx,7);
        laneIndx = lane_list_sorted(sortIndx,1);        
        if(insert_remove_flag == 1) % remove the index from the active lane
          toRemove = LaneActiveIndx(:,1)==laneIndx;
          if sum(toRemove)~=1
          	warning('Non dovrebbe succedere che ho un numero diverso da 1 elemento da rimuovere')   
          end
          LaneActiveIndx = LaneActiveIndx(not(toRemove),:);
        else % add the new lane
            theta_seg = lane_list_sorted(sortIndx,2);
            lane_d = lane_list_sorted(sortIndx,3);
%             if laneIndx == 75
%                pause(0.1); 
%             end
            LaneActiveIndx = [LaneActiveIndx;[laneIndx,theta_seg,lane_d,0]]; % 0 is a fake init.. we will compute it after
        end
        if not(sortIndx < numel(lane_list_sorted(:,1)) && (lane_list_sorted(sortIndx+1,5)-theta)<EPS)
            break;
        end        
    end

    if esci
        break;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%% TODO: SI POTREBBE RISPARMIARE IL CALCOLO PER
    %%%%%%%%%%%%%%%%%%%%%%%% QUELLI GIA CALCOLATI
    if(not(isempty(LaneActiveIndx)))
        for i=1:numel(LaneActiveIndx(:,1))
            LaneActiveIndx(i,4) = findDfromTheta(LaneActiveIndx(i,2),LaneActiveIndx(i,3),theta);
        end
        LaneActiveIndx   = sortrows(LaneActiveIndx,4); 
        if false
            px = [0, cos(theta)]*5.9 + p0(1);
            py = [0, sin(theta)]*5.9 + p0(2);
            plot(px,py,'b');
            px = [0, cos(theta+0.1*pi/180)]*5.9 + p0(1);
            py = [0, sin(theta+0.1*pi/180)]*5.9 + p0(2);
            plot(px,py,'r');
        end
        % Vediamo se ci sono con pari distanza a pari theta
        if(size(LaneActiveIndx,1)>1 && abs(LaneActiveIndx(1,4)-LaneActiveIndx(2,4))<1e-3)
            minore =  LaneActiveIndx(1,4);
            kk = 1;
            while (kk<=numel(LaneActiveIndx(:,4))) && (LaneActiveIndx(kk,4)-minore) < 1e-3 
                LaneActiveIndx(kk,4) = findDfromTheta(LaneActiveIndx(kk,2),LaneActiveIndx(kk,3),theta+0.1*pi/180);
                kk = kk+1;
            end
            kk = kk - 1;


            % Order the point reling on distance in theta = 0 of the segment
            if kk>1
                LaneActiveIndx(1:kk,:)   = sortrows(LaneActiveIndx(1:kk,:),4); 
            end
        end
        LaneViewedIndx = [LaneViewedIndx;[LaneActiveIndx(1,1),theta]];
        
        if DEBUG
            tag_to_del = {'ray','active'};
            for kk=1:numel(tag_to_del)
                to_del = findobj('Tag',tag_to_del{kk});
                for kkk = 1:numel(to_del)
                   delete(to_del(kkk)); 
                end
            end
            
            % Plot clipped lane
            for i=1:size(LaneActiveIndx,1)
                indx = (LaneActiveIndx(i,1)-1)*2 + [1,2];
                P(:,1) = lane_list(indx(1),6)*[cos(lane_list(indx(1),5)); sin(lane_list(indx(1),5)) ] + p0;
                P(:,2) = lane_list(indx(2),6)*[cos(lane_list(indx(2),5)); sin(lane_list(indx(2),5)) ] + p0;
                plot( P(1,:), P(2,:) ,'r','Tag','active');
            end


            ray = [0, cos(theta); 0, sin(theta)]*5.6*sqrt(2) + p0; 
            plot(ray(1,:),ray(2,:),'m--','Tag','ray') 
            drawnow();
            display(LaneActiveIndx)   % la quarta colonna e' la distanza
            pause();
            
        end        
    end
end

lineeViste = [];
if isempty(LaneViewedIndx)
   return; 
end
numeroPezziLane = numel(LaneViewedIndx(:,1));
k = 0;
while k<numeroPezziLane && LaneViewedIndx(numeroPezziLane-k,1) == LaneViewedIndx(1,1)
    k = k+1;
end

if k>0
    k = k - 1;
    toAdd   = LaneViewedIndx(end-k,:); % keep memory of last point
    LaneViewedIndx(end-k:end,2)  = LaneViewedIndx(end-k:end,2) - 2*pi;
    LaneViewedIndx(end+1,:) = toAdd;
else    
    LaneViewedIndx(end+1,:) = [LaneViewedIndx(end,1), 2*pi];
end
LaneViewedIndx = sortrows(LaneViewedIndx,2);
    

i = 1;

while i<numel(LaneViewedIndx(:,1))
    inizio = i;
    lane_id_start = LaneViewedIndx(inizio,1);

%     if lane_id_start==1
%         pause();
%     end
    while( i<numel(LaneViewedIndx(:,1)) && LaneViewedIndx(i+1,1)==lane_id_start)
        i = i + 1;
    end
    i = 1 + i;
    fine = i;
    
    if fine > numel(LaneViewedIndx(:,1))
       fine =  numel(LaneViewedIndx(:,1));
    end
    
    theta_limit_seg = lane_list((lane_id_start-1)*2+(1:2),5);
    theta_seg = lane_list((lane_id_start-1)*2+1,2);
    d_seg = lane_list((lane_id_start-1)*2+1,3);
    
    theta_proc = [LaneViewedIndx(inizio,2),LaneViewedIndx(fine,2)];
    if theta_proc(1) < 0
         theta_limit_seg([2,1]) = theta_limit_seg([1,2]);
         theta_limit_seg(1) = theta_limit_seg(1) - 2*pi;
    end
    
    theta_lim = [max(theta_proc(1),theta_limit_seg(1)),min(theta_proc(2),theta_limit_seg(2))];
    
    lineeViste = [lineeViste; [lane_id_start,theta_lim,theta_seg,d_seg]];
end



end
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               

function [theta] = findThetafromD(theta_lane,d_lane,d_point)
theta1 = acos(d_lane/d_point);

% Ritorna theta min e theta max che escono da funzione
theta = theta_lane + pi/2 + [-1,1]*theta1;
end

function [d_point] = findDfromTheta(theta_lane,d_lane,theta_point)
theta1 = theta_point - theta_lane - pi/2;
d_point = abs(d_lane/cos(theta1));
end