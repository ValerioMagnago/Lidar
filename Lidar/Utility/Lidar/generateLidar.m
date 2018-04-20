function [lidar] = generateLidar(LaneInView,theta0)

global DEBUG

lidar = defaultLaserData();
if(isempty(LaneInView))
   return; 
end
thetas = theta0 +linspace(lidar.AngleMin,lidar.AngleMax,numel(lidar.Ranges));
% thetas = theta0 + (lidar.AngleMin:lidar.angle_increment:(lidar.AngleMax+lidar.angle_increment/2));

if DEBUG
    figure();
    hold on;
    box;
    axis equal;
    plot([0,cos(theta0)],[0,sin(theta0)],'k');
    plot(0,0,'.k','markersize',16);
    plot(5.6*cos(thetas),5.6*sin(thetas),'--m');
    plot(5.6*[0,cos(thetas(1))],5.6*[0,sin(thetas(1))],'m--');
    plot(5.6*[0,cos(thetas(end))],5.6*[0,sin(thetas(end))],'m--');

    for i = 1:size(LaneInView,1)
        p1 = findDfromTheta(LaneInView(i,4),LaneInView(i,5),LaneInView(i,2))*[cos(LaneInView(i,2));sin(LaneInView(i,2))];
        p2 = findDfromTheta(LaneInView(i,4),LaneInView(i,5),LaneInView(i,3))*[cos(LaneInView(i,3));sin(LaneInView(i,3))];
        plot([p1(1),p2(1)],[p1(2),p2(2)],'b');
        text(mean([p1(1),p2(1)]),mean([p1(2),p2(2)]),num2str(LaneInView(i,1)))
    end
end
% Troviamo indice da cui iniziare
lane_id = 1;
k_start = 1;

minTheta = thetas(1);
maxTheta = thetas(end);

while lane_id<=size(LaneInView,1)
    thetaStart = LaneInView(lane_id,2);
    while thetaStart-theta0 < -pi
       thetaStart = thetaStart + 2*pi; 
    end
    while thetaStart-theta0 > pi
       thetaStart = thetaStart - 2*pi; 
    end
    
    % Check if theta start is in the range
    if(thetaStart>minTheta && thetaStart<maxTheta)
        % if theta is in the range find the nearest k        
        while normalizeDistance(thetas(k_start),thetaStart)<0
            k_start = k_start + 1;
        end   
        
        % we inizialize k_start we can go!!!!
        break;
        
    else
        thetaEnd = LaneInView(lane_id,3);
        while thetaEnd-theta0 < -pi
           thetaEnd = thetaEnd + 2*pi; 
        end
        while thetaEnd-theta0 > pi
           thetaEnd = thetaEnd - 2*pi;           
        end
        
        if(thetaEnd>minTheta && thetaEnd<maxTheta)
            % if theta is in the range find the nearest k        
            while normalizeDistance(thetas(k_start),minTheta)<0
                k_start = k_start + 1;
            end   

            % we inizialize k_start we can go!!!!
            break;
        end        
    end
    
    lane_id = lane_id + 1;
end

% We have no lane in the range
if(lane_id>size(LaneInView,1))
   return; 
end

passi = -1;
while passi<(numel(thetas)-1)
    k = 1 + rem(k_start+passi,numel(thetas));
    
    theta = thetas(k);    
    % Vediamo che theta sia minore del massimo theta del segmento
%     if LaneInView(lane_id,1)==74
%        pause(0.1); 
%     end
    while(lane_id<=size(LaneInView,1) && normalizeDistance(theta,LaneInView(lane_id,3))>0)
        lane_id = lane_id + 1;
    end   
   
    
    if(lane_id>size(LaneInView,1))
        break;
    end
    
    % Vediamo che theta sia maggiore del minimo theta del segmeneto
    if(normalizeDistance(theta,LaneInView(lane_id,2))>0)
        range = findDfromTheta(LaneInView(lane_id,4),LaneInView(lane_id,5),theta);
        if range<lidar.RangeMin
            lidar.Ranges(k) = NaN;
        elseif range<lidar.RangeMax
            lidar.Ranges(k) = range;
        end
    end    
    
    if DEBUG
       tag_to_del = {'active','laser_point','theta'};
        for kk=1:numel(tag_to_del)
            to_del = findobj('Tag',tag_to_del{kk});
            for kkk = 1:numel(to_del)
               delete(to_del(kkk)); 
            end            
        end 
        plot(5.6*[0,cos(theta)],5.6*[0,sin(theta)],'r','Tag','theta');
                
        p1 = findDfromTheta(LaneInView(lane_id,4),LaneInView(lane_id,5),LaneInView(lane_id,2))*[cos(LaneInView(lane_id,2));sin(LaneInView(lane_id,2))];
        p2 = findDfromTheta(LaneInView(lane_id,4),LaneInView(lane_id,5),LaneInView(lane_id,3))*[cos(LaneInView(lane_id,3));sin(LaneInView(lane_id,3))];
        plot([p1(1),p2(1)],[p1(2),p2(2)],'g','Tag','active');
        
        px = lidar.Ranges(k)*[0,cos(theta)];
        py = lidar.Ranges(k)*[0,sin(theta)];
        plot(px,py,'xk','Tag','laser_point','markersize',10);
        plot(px,py,'.k','markersize',5);
        
        drawnow(); 
%        pause()
    end
    passi = passi + 1;
end
end


function default = defaultLaserData()
persistent id;
if isempty(id)
    id = 0;
end
id = id + 1;
default.Header.MessageType  = 'std_msgs/Header';
default.Header.Seq  = id;
default.Header.Stamp.Sec = 0;
default.Header.Stamp.Nsec = 0;
default.AngleMin = -1.5708;
default.AngleMax = 1.5647;
default.AngleIncrement = 0.0061;
default.TimeIncrement = 9.7656e-05; 
default.ScanTime = 0.1000;
default.RangeMin = 0.02;
default.RangeMax = 5.6;
default.Ranges = Inf(515,1);
end


function [d_point] = findDfromTheta(theta_lane,d_lane,theta_point)
theta1 = theta_point - theta_lane - pi/2;
d_point = abs(d_lane/cos(theta1));
end


function dThetaNorm = normalizeDistance(first_theta, second_theta)
    dThetaNorm = first_theta - second_theta;
    while(dThetaNorm>pi)
        dThetaNorm = dThetaNorm - 2*pi;
    end
    
    while(dThetaNorm<-pi)
        dThetaNorm = dThetaNorm + 2*pi;
    end
end