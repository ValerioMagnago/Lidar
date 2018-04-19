% --------------------------------------------------------------
%
%   Display the wall in the FOV
%
% --------------------------------------------------------------
%======================================================================%
%                                                                      %
%  Autors: Valerio Magnago                                             %
%          Daniele Fontanelli                                          %
%          University of Trento                                        %
%          valerio.magnago@unitn.it                                    %
%                                                                      %
%  Date:  24/10                                                        %
%  Version: 0.1                                                        %
%======================================================================%


%% Init console
% Go to script folder
filename = which(mfilename);
[pathstr,name,ext] = fileparts(filename);
cd(pathstr);

% Clear console and variables
clc;
clear all; %#ok<CLALL>
close all;

% Add path
run ../Lidar/use_lidar
run ../Maps/use_map


%**************************************************************************
%% Parameters definition
%**************************************************************************

% Print to screen what the script is doing
sec_numb = 1; fprintf('%d) Loading users params\n',sec_numb); sec_numb = sec_numb + 1;


% Define lidar parameter are in generateLIdar function.
fake_lid = generateLidar([],0);
R = fake_lid.range_max;
LidarScanArea = defineLidarArea(R);


TART_D = 0.5;   % Diametro Tartufino
K_ang = 8;
K_d = 1/5;


mapName = 'Povo2_floor1.txt';

%s0 = [301.5;-55;+pi/2];
s0 = [305;-16;0];

%**************************************************************************
%% Load obstacles map and creeate obstacles tree
%**************************************************************************
fprintf('%d) Loading obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;
obstaclesTree = loadPath(mapName);

%**************************************************************************
%% Plot map
%**************************************************************************
fprintf('%d) Plotting the map\n',sec_numb); sec_numb = sec_numb + 1;
thetas_tartuf = linspace(-pi,pi,30);
tartuf_ingombro.x = TART_D/2*cos(thetas_tartuf);
tartuf_ingombro.y = TART_D/2*sin(thetas_tartuf);

global map_fig;
map_fig = figure(1); clf;    % care map_fig is used as global variable inside some function
subplot(1,2,1);
hold on;
view(-90,90);
title('Map','Interpreter','latex');
xlabel('$x_{glob} [m] $','Interpreter','latex');
ylabel('$y_{glob} [m] $','Interpreter','latex');
axis equal;
fig_1_ObstPoly = plotObstacles(obstaclesTree.obstacles(:,2), 1 ,{[0.7,0.7,0.65],1});
axis tight;
limiti_x = xlim();
limiti_y = ylim();
xlim(limiti_x);
ylim(limiti_y);

subplot(2,2,2);
hold on;
axis equal;
box
title('on board','Interpreter','latex');
fill(tartuf_ingombro.x([1:end,1]), tartuf_ingombro.y([1:end,1]),[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2);
xlim([-1 1]*fake_lid.range_max);
ylim([-1 1]*fake_lid.range_max);
xlabel('$x_{tartuf} [m] $','Interpreter','latex');
ylabel('$y_{tartuf} [m] $','Interpreter','latex');

subplot(2,2,4);
hold on;
axis equal;
box
xlim([-1 1]*pi);
ylim([-0.2 fake_lid.range_max]);
set(gca, 'XDir','reverse')   % per ordinare e mettere sinistra a sinistra e destra a destra
title('on board','Interpreter','latex');
plot(fake_lid.angle_min*[1 1],[-0.2 fake_lid.range_max],'k-.');
plot(fake_lid.angle_max*[1 1],[-0.2 fake_lid.range_max],'k-.');
fill(thetas_tartuf([1:end,end:-1:1]),[ones(1,numel(thetas_tartuf)),-ones(1,numel(thetas_tartuf))]*TART_D/2,[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2);
xlabel('$\theta_{las} [m] $','Interpreter','latex');
ylabel('$d_{las} [m] $','Interpreter','latex');


%**************************************************************************
%% Create random state of particle and compute information
%**************************************************************************
fprintf('%d) Create random state \n',sec_numb); sec_numb = sec_numb + 1;
rng(0,'twister');  % initialize the random number generator to make the results in this example repeatable


state = s0; % x,y,theta of the system

larg_porta = 1;
dt = 0.1;
sim_time = 1000;  % time to simulate in sec
step_numb = sim_time/dt;
start_port = 1;
end_port = 0;
for step_id = 1:step_numb
    tic
    %% Simulate LIDAR
    [lidarScan] = simulateLIDAR(LidarScanArea,obstaclesTree,state);    
    angles = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    
    %% Check port presence    
    prev_d = min(lidarScan.ranges(1),lidarScan.range_max);
    passaggi = [];
    for ang_id=1:numel(lidarScan.ranges)
        angolo = angles(ang_id);
        d = lidarScan.ranges(ang_id);
        if(isnan(d))
            continue;
        end
        
        if(isinf(d))
            d = lidarScan.range_max;
            end_port = ang_id;
        else
            if(((end_port - start_port)*lidarScan.angle_increment*lidarScan.range_max)>larg_porta)
                passaggi = [passaggi; [start_port,end_port]];
            end
            start_port = ang_id+1;
            end_port = ang_id;
        end
        
        if(abs(d-prev_d)>larg_porta)
            passaggi = [passaggi; ang_id - [1,0]];
        end

        % Update last d
        prev_d = d;
    end
    if(((end_port - start_port)*lidarScan.angle_increment*lidarScan.range_max)>larg_porta)
        passaggi = [passaggi; [start_port,end_port]];
    end
    
    %% Compute control    
    Atot = 0;
    sum_angle = 0;
    sum_d = 0;
    stop = false;
    
    list_d = [];
    
    for ang_id=1:numel(lidarScan.ranges)
        angolo = angles(ang_id);
        d = lidarScan.ranges(ang_id);
        if(isnan(d))
            continue;
        end
        if(isinf(d))
            d = lidarScan.range_max;
        end
        
        
        if(d < TART_D)
            stop = true;
        end
        
        
        dA = d*d;
        list_d = [list_d,d];
        
        Atot = Atot + dA;
        sum_angle = sum_angle + dA*angolo;
        sum_d     = sum_d     + dA*d;
        
        % Update last d
        prev_d = d;
    end
    
    % Compute the mean
    theta = sum_angle / Atot;
    dist  = sum_d / Atot;
    
    
    if(stop || Atot < 300)
        v = 0;
        omega = -0.5; % turn right
    else
        v = K_d*dist;
        omega = K_ang*theta;
    end
    
    %% PLOT
    forward_time = 2;
    forward_step = ceil(forward_time/dt);
    
    distss = lidarScan.ranges;
    logic = isinf(distss);
    distss(logic) = lidarScan.range_max;
    
    subplot(1,2,1);
    tag_to_del = {'laser_point','laser_range','vehicle','speeds','path'};
    for kk=1:numel(tag_to_del)
        to_del = findobj('Tag',tag_to_del{kk});
        for kkk = 1:numel(to_del)
            delete(to_del(kkk));
        end
    end
    state_tmp = zeros(3,forward_step);
    state_tmp(:,1) = state;
    for prev=2:forward_step
        state_tmp(:,prev) = updateState(state_tmp(:,prev-1),v,omega,dt);
    end
    thetas = state(3)+linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    fill(tartuf_ingombro.x([1:end,1])+state(1), tartuf_ingombro.y([1:end,1])+state(2),[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2,'Tag','vehicle');
    plot(state(1),state(2),'g.','markersize',6,'Tag','vehicle');
    plot(state(1)+[0,cos(state(3))]*TART_D/2*1.2,state(2)+[0,sin(state(3))]*TART_D/2*1.2,'k-','Tag','vehicle','linewidth',2);
    plot(lidarScan.ranges.*cos(thetas)+state(1),lidarScan.ranges.*sin(thetas)+state(2),'r*','Tag','laser_point');
    plot(lidarScan.range_max*cos(thetas)+state(1),lidarScan.range_max*sin(thetas)+state(2),'--m','Tag','laser_range');
    plot(lidarScan.range_max*[0,cos(thetas(1))]+state(1),lidarScan.range_max*[0,sin(thetas(1))]+state(2),'m--','Tag','laser_range');
    plot(lidarScan.range_max*[0,cos(thetas(end))]+state(1),lidarScan.range_max*[0,sin(thetas(end))]+state(2),'m--','Tag','laser_range');
    plot(state(1),state(2),'k.','markersize',8);
    plot(state_tmp(1,:),state_tmp(2,:),'--g','Tag','path')
    
    subplot(2,2,2);
    state_tmp = zeros(3,forward_step);
    state_tmp(:,1) = [0;0;state(3)];
    for prev=2:forward_step
        state_tmp(:,prev) = updateState(state_tmp(:,prev-1),v,omega,dt);
    end
    plot(lidarScan.ranges.*cos(thetas),lidarScan.ranges.*sin(thetas),'r*','Tag','laser_point');
    plot(lidarScan.range_max*cos(thetas),lidarScan.range_max*sin(thetas),'--m','Tag','laser_range');
    plot(lidarScan.range_max*[0,cos(thetas(1))],lidarScan.range_max*[0,sin(thetas(1))],'m--','Tag','laser_range');
    plot(lidarScan.range_max*[0,cos(thetas(end))],lidarScan.range_max*[0,sin(thetas(end))],'m--','Tag','laser_range');
    plot(0,0,'.k','markersize',10,'Tag','laser_range');
    plot(state_tmp(1,:),state_tmp(2,:),'--g','Tag','path')
    for i=1:size(passaggi,1)
        plot(distss(passaggi(i,:)).*cos(thetas(passaggi(i,:))),distss(passaggi(i,:)).*sin(thetas(passaggi(i,:))),'bx--','linewidth',2,'Tag','laser_range');
    end
    
    subplot(2,2,4);
    thetas = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    state_polar_theta = atan2(state_tmp(2,:),state_tmp(1,:));
    state_polar_r     = sqrt(state_tmp(2,:).^2+state_tmp(1,:).^2);
    plot(thetas,distss,'r--','Tag','laser_range');
    plot(thetas(not(logic)),distss(not(logic)),'r*','Tag','laser_range');
    plot(thetas(logic),distss(logic),'c*','Tag','laser_range');
    plot([-1 1]*pi,[1 1]*dist,'m-.','Tag','laser_range');
    plot(theta*[1 1],[-0.2 fake_lid.range_max],'m-.','Tag','laser_range');
    plot(theta, dist,'mo','Tag','laser_range');
    plot(state_polar_theta,state_polar_r,'--g','Tag','path')
    
    %[valli,picchi] = trovaPicchi(distss);
    %     media = mean(distss);
    %     logic_jump = (abs(diff(distss))>0.8);
    %     logic_high = (distss>5.5);
    %     plot(thetas(logic_jump),distss(logic_jump) ,'b.','markersize',10,'Tag','laser_range');
    %     plot(thetas(logic_high),distss(logic_high) ,'b.','markersize',10,'Tag','laser_range');
    %     plot(thetas(picchi),distss(picchi),'bx','markersize',3,'Tag','laser_range');
    for i=1:size(passaggi,1)
        plot(thetas(passaggi(i,:)),distss(passaggi(i,:)),'bx--','linewidth',2,'Tag','laser_range');
    end
    
    drawnow();
%     pause();
    endTime = toc;
    pause(dt-endTime);
    
    %% Integrate the controls
    state = updateState(state,v,omega,dt);
end


