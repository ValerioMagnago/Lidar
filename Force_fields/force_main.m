%% Init consoletruetrue
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
R = fake_lid.RangeMax;
LidarScanArea = defineLidarArea(R);


TART_D = 0.5;   % Diametro Tartufino
plot_on = true;
mex_on = true;
%**************************************************************************
%% Load obstacles map and creeate obstacles tree
%**************************************************************************
fprintf('%d) Loading obstacles tree\n',sec_numb); sec_numb = sec_numb + 1;
obstaclesTree = loadPath(mapName);

%**************************************************************************
%% Plot map
%**************************************************************************
if plot_on
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
    xlim([-1 1]*fake_lid.RangeMax);
    ylim([-1 1]*fake_lid.RangeMax);
    xlabel('$x_{tartuf} [m] $','Interpreter','latex');
    ylabel('$y_{tartuf} [m] $','Interpreter','latex');
    arrow = 1; % initialization force plot
    
    
    subplot(2,2,4);
    hold on;
    axis equal;
    box
    xlim([-1 1]*pi);
    ylim([-0.2 fake_lid.RangeMax]);
    title('on board','Interpreter','latex');
    plot(fake_lid.AngleMin*[1 1],[-0.2 fake_lid.RangeMax],'k-.');
    plot(fake_lid.AngleMax*[1 1],[-0.2 fake_lid.RangeMax],'k-.');
    fill(thetas_tartuf([1:end,end:-1:1]),[ones(1,numel(thetas_tartuf)),-ones(1,numel(thetas_tartuf))]*TART_D/2,[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2);
    xlabel('$\theta_{las} [m] $','Interpreter','latex');
    ylabel('$d_{las} [m] $','Interpreter','latex');
end

%**************************************************************************
%% Force field parameters
%**************************************************************************
%% Define potential repulsive functions
% Define repulsive force U_alpha_B (eq.13 SFM)
syms U_0_alpha_B r_alpha_B_x r_alpha_B_y R

r_alpha_B = [r_alpha_B_x; r_alpha_B_y];
U_alpha_B = U_0_alpha_B * exp(-(norm(r_alpha_B))/R);
Grad_U    = gradient(U_alpha_B,[r_alpha_B_x, r_alpha_B_y]);
% Grad_U    = matlabFunction(Grad_U);
matlabFunction(Grad_U, 'File','Grad_U');

clear U_0_alpha_B
clear r_alpha_B_x
clear r_alpha_B_y
clear R




%% Define control data
[R_potential, U_0_alpha_B_potential] = tune_potential(0.5, 1, 10, 0.01);
[R_vortex, U_0_alpha_B_vortex]       = tune_potential(0.5, 3, 3, 1);

force_param.R_potential = R_potential;
force_param.U_0_alpha_B_potential = U_0_alpha_B_potential;
force_param.R_vortex = R_vortex;
force_param.U_0_alpha_B_vortex = U_0_alpha_B_vortex;
force_param.max_deceleration = 10;
force_param.braking_angle = 75 * pi/180;
force_param.k_prj_omega = 2;
force_param.omega_max = 45 * pi /180;
force_param.v_des = 1;
force_param.v_max = 1;
force_param.k_i_vel = 1;



%**************************************************************************
%% Create random state of particle and compute information
%**************************************************************************
fprintf('%d) Create random state \n',sec_numb); sec_numb = sec_numb + 1;
rng(0,'twister');  % initialize the random number generator to make the results in this example repeatable

s0          = [380;-16;0];
state       = s0; % x,y,theta of the system
v_old       = 0; % initial linear velocity
omega_old   = 0; % initial angular velocity



dt = 0.1;
sim_time = 100;  % time to simulate in sec
step_numb = sim_time/dt;

for step_id = 1:step_numb
    tic
    %% Simulate LIDAR
    [lidarScan] = simulateLIDAR(LidarScanArea,obstaclesTree,state);
    
    if mex_on
        [v, omega]  = lidar2force_field_mex(lidarScan, state, v_old, omega_old, dt,force_param);
    else
        [v, omega]  = lidar2force_field(lidarScan, state, v_old, omega_old, dt,force_param);
    end
    
    v_old       = v;
    omega_old   = omega;
    
    %% PLOT
    if plot_on
        forward_time = 2;
        forward_step = ceil(forward_time/dt);
        subplot(1,2,1);
        tag_to_del = {'laser_point','laser_range','vehicle','speeds','path'};
        %     tmp = [cos(state(3)), -sin(state(3)); sin(state(3)), cos(state(3))]*[total_force(1);total_force(2)];
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
        thetas = state(3) + linspace(lidarScan.AngleMin,lidarScan.AngleMax,numel(lidarScan.Ranges));
        fill(tartuf_ingombro.x([1:end,1])+state(1), tartuf_ingombro.y([1:end,1])+state(2),[0, 100, 255]/255,'FaceAlpha',0.7,'EdgeAlpha',1,'EdgeColor','k','LineWidth',2,'Tag','vehicle');
        plot(state(1),state(2),'g.','markersize',6,'Tag','vehicle');
        plot(state(1)+[0,cos(state(3))]*TART_D/2*1.2,state(2)+[0,sin(state(3))]*TART_D/2*1.2,'k-','Tag','vehicle','linewidth',2);
        plot(lidarScan.Ranges'.*cos(thetas)+state(1),lidarScan.Ranges'.*sin(thetas)+state(2),'r*','Tag','laser_point');
        plot(lidarScan.RangeMax*cos(thetas)+state(1),lidarScan.RangeMax*sin(thetas)+state(2),'--m','Tag','laser_range');
        plot(lidarScan.RangeMax*[0,cos(thetas(1))]+state(1),lidarScan.RangeMax*[0,sin(thetas(1))]+state(2),'m--','Tag','laser_range');
        plot(lidarScan.RangeMax*[0,cos(thetas(end))]+state(1),lidarScan.RangeMax*[0,sin(thetas(end))]+state(2),'m--','Tag','laser_range');
        plot(state(1),state(2),'k.','markersize',8);
        plot(state_tmp(1,:),state_tmp(2,:),'--g','Tag','path')
        
        
        
        subplot(2,2,2);
        state_tmp = zeros(3,forward_step);
        state_tmp(:,1) = [0;0;state(3)];
        for prev=2:forward_step
            state_tmp(:,prev) = updateState(state_tmp(:,prev-1),v,omega,dt);
        end
        
        plot(lidarScan.Ranges'.*cos(thetas),lidarScan.Ranges'.*sin(thetas),'r*','Tag','laser_point');
        plot(lidarScan.RangeMax*cos(thetas),lidarScan.RangeMax*sin(thetas),'--m','Tag','laser_range');
        plot(lidarScan.RangeMax*[0,cos(thetas(1))],lidarScan.RangeMax*[0,sin(thetas(1))],'m--','Tag','laser_range');
        plot(lidarScan.RangeMax*[0,cos(thetas(end))],lidarScan.RangeMax*[0,sin(thetas(end))],'m--','Tag','laser_range');
        plot(0,0,'.k','markersize',10,'Tag','laser_range');
        plot(state_tmp(1,:),state_tmp(2,:),'--g','Tag','path')
        %     arrow = quiver(0,0, total_force(1), total_force(2), 'color', 'g', 'linewidth', 2);
        
        
        subplot(2,2,4);
        thetas = linspace(lidarScan.AngleMin,lidarScan.AngleMax,numel(lidarScan.Ranges));
        plot(thetas,lidarScan.Ranges','r*--','Tag','laser_range');
        drawnow();
        
        endTime = toc;
        pause(dt-endTime);
    end
    %% Integrate the controls
    state = updateState(state,v,omega,dt);
end


