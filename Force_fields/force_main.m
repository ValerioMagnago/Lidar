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
run  use_force


%**************************************************************************
%% Parameters definition
%**************************************************************************
% Print to screen what the script is doing
sec_numb = 1; fprintf('%d) Loading users params\n',sec_numb); sec_numb = sec_numb + 1;

mapName = 'Povo2_floor1.txt';

% Define lidar parameter are in generateLIdar function.
fake_lid = generateLidar([],0);
R = fake_lid.range_max;
LidarScanArea = defineLidarArea(R);


TART_D = 0.5;   % Diametro Tartufino


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
arrow = 1; % initialization force plot





subplot(2,2,4);
hold on;
axis equal;
box
xlim([-1 1]*pi);
ylim([-0.2 fake_lid.range_max]);
title('on board','Interpreter','latex');
plot(fake_lid.angle_min*[1 1],[-0.2 fake_lid.range_max],'k-.');
plot(fake_lid.angle_max*[1 1],[-0.2 fake_lid.range_max],'k-.');
fill(thetas_tartuf([1:end,end:-1:1]),[ones(1,numel(thetas_tartuf)),-ones(1,numel(thetas_tartuf))]*TART_D/2,[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2);
xlabel('$\theta_{las} [m] $','Interpreter','latex');
ylabel('$d_{las} [m] $','Interpreter','latex');


%**************************************************************************
%% Force field parameters
%**************************************************************************
%% Define potential repulsive functions
% Define repulsive force U_alpha_B (eq.13 SFM)
syms U_0_alpha_B r_alpha_B_x r_alpha_B_y R

r_alpha_B = [r_alpha_B_x; r_alpha_B_y];
U_alpha_B = U_0_alpha_B * exp(-(norm(r_alpha_B))/R);
Grad_U    = gradient(U_alpha_B,[r_alpha_B_x, r_alpha_B_y]);
Grad_U    = matlabFunction(Grad_U);

clear U_0_alpha_B
clear r_alpha_B_x
clear r_alpha_B_y
clear R
% symObj    = syms;
%
% cellfun(@clear,symObj)
% use like this: Grad_U(R,U_0_alpha_B,r_alpha_B_x,r_alpha_B_y)
% Parameters:
% R_potential           = .3;
% U_0_alpha_B_potential = 200;
% R_vortex              = 10;
% U_0_alpha_B_vortex    = 10;
[R_potential, U_0_alpha_B_potential] = tune_potential(0.5, 1, 10, 0.01);
[R_vortex, U_0_alpha_B_vortex]       = tune_potential(0.5, 3, 3, 1);

dist                  = 0.1;
% walker_positions      = {[dist; 0], [-dist; 0], [0; dist], [0; -dist]}; % filter the potential arond the walker position
walker_positions      = {[0;0]};

%% Define control data
omega_max        = 45 * pi / 180; % max angula velocity for the robot
k_i_vel          = 1; % integral gain on velocity
v_0              = 0;   % initial velocity (integrator)
v_max            = 1; % max forward velocity for the robot
v_des            = 1;
k_prj_vel        = 1; % gain for the projection of the total force along the x axis of the vehicle
k_prj_omega      = 2; % gain for the projection of the total force along the y axis of the vehicle
braking_angle    = 75*pi/180;
max_deceleration = 10;
vortex_force_old = [0;0];

%**************************************************************************
%% Create random state of particle and compute information
%**************************************************************************
fprintf('%d) Create random state \n',sec_numb); sec_numb = sec_numb + 1;
rng(0,'twister');  % initialize the random number generator to make the results in this example repeatable

% s0 = [380;-16;0];
s0 = [3001.5;-55;-pi/2];
state = s0; % x,y,theta of the system


dt = 0.1;
sim_time = 1000;  % time to simulate in sec
step_numb = sim_time/dt;
for step_id = 1:step_numb
    tic
    %% Simulate LIDAR
    [lidarScan] = simulateLIDAR(LidarScanArea,obstaclesTree,state);
    
    %% Compute control
    angles = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    
    %% Elaborate lidar data
    % put the obstacles measured in groups
    obst_group = group_obstacles(angles, lidarScan.ranges, 0.15);
    
    %% Filtered potential fields
    points           = point_obst(obst_group, angles, lidarScan.ranges, [0;0]); % the zero is correct
    repulsive_force  = [0;0];
    vortex_force     = [0;0];
    for kk = 1:length(walker_positions)
        points         = point_obst(obst_group, angles, lidarScan.ranges, walker_positions{kk}); % find the closest point for each obstacle
        %% Repulsive actions
        tmp             = size(points);
        for j = 1:tmp(2) % number of column of points
            r_alpha         = [0;0];
            r_B             = [points(1,j); points(2,j)];
            r_alpha_B       = r_alpha - r_B;
            r_alpha_B_x     = r_alpha_B(1);
            r_alpha_B_y     = r_alpha_B(2);
            repulsive_force = repulsive_force - Grad_U(R_potential, U_0_alpha_B_potential,r_alpha_B_x, r_alpha_B_y);
        end
        %% Vortex force       
        for j = 1:tmp(2) % number of column of points
            r_alpha         = [0;0];
            r_B             = [points(1,j); points(2,j)];
            r_alpha_B       = r_alpha - r_B;
            r_alpha_B_x     = r_alpha_B(1);
            r_alpha_B_y     = r_alpha_B(2);
            tmp             = Grad_U(R_vortex, U_0_alpha_B_vortex, r_alpha_B_x, r_alpha_B_y);
            grad_x          = tmp(1);
            grad_y          = tmp(2);
            positive_vortex = [grad_y; -grad_x];
            
            
            vortex_dot_robot = dot([1;0], positive_vortex);
            vortex_dot_force = dot(positive_vortex, vortex_force_old);
            if (step_id == 1 &&  vortex_dot_robot < 0) 
                positive_vortex = -positive_vortex; % initialize in vehicle direction
            elseif  vortex_dot_robot < 0
                positive_vortex = -positive_vortex;
            end       
            vortex_force = vortex_force + positive_vortex;
        end
        
    end
    vortex_force     = vortex_force    / length(walker_positions); 
    repulsive_force  = repulsive_force / length(walker_positions);
    
    %% Add attractive force
    % Compute acceleration term
    attractive_force = [0;0];
    %% Total force
    total_force  = attractive_force + repulsive_force + vortex_force * 1;
    vortex_force_old = total_force;
    %% Run controller
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % choose the forward vel
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if step_id == 1
        v = v_0; % i.e., in the initial time move
    else
        force_prj     = dot([1;0], total_force); 
        force_angle   = dot([1;0],total_force / (eps + norm(total_force)));   % cos(angle) between force and vehicle
        if force_angle >= cos(braking_angle) % then acelerate using a PI to the desired speed
            v = v_old + dt * k_i_vel * (v_des - v_old);
            v = min(v, v_max); % this should be useless
            v = max(0, v);
        else % then brake
            braking_action = max_deceleration *  (force_angle - cos(braking_angle));
            v              = v_old + dt * braking_action;
            v              = max(0, v);
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % choose the angular vel
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    omega = k_prj_omega * dot([0;1], total_force);
    omega = min(omega_max, max(-omega_max, omega));
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
%     v     = 1;
    v_old = v;
%     omega = 1;
    
    
 
    %% PLOT
    forward_time = 2;
    forward_step = ceil(forward_time/dt);
    subplot(1,2,1);
    tag_to_del = {'laser_point','laser_range','vehicle','speeds','path'};
    tmp = [cos(state(3)), -sin(state(3)); sin(state(3)), cos(state(3))]*[total_force(1);total_force(2)];
%     arrow = quiver(state(1), state(2), tmp(1), tmp(2), 'color', 'g', 'linewidth', 2);
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
    thetas = state(3) + linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
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
%     arrow = quiver(0,0, total_force(1), total_force(2), 'color', 'g', 'linewidth', 2);

    
    subplot(2,2,4);
    thetas = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    plot(thetas,lidarScan.ranges,'r*--','Tag','laser_range');
    drawnow();
    
    endTime = toc;
    pause(dt-endTime);
    
    %% Integrate the controls
    state = updateState(state,v,omega,dt);
end


