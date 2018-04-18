function [fusion ,H_new, R_new ] = wallKalmanUpdate( state, LaneView, var_theta, var_r)
%ROTOTRANSLATEP Summary of this function goes here
%   Detailed explanation goes here


% TODO: try an UKF

% Tranalation need for rototraslation
trasl = -rotoTranslateP( state(1:2), -state(3), [0;0]);

%% Parameters
min_length = 0.1;  % min length of a wall to consider

% Uncertanty in theta and r of the point grabbed from the kinekt
% var_theta = 1*pi/180;   % 1 grado di vari%%anza su theta
% var_r = 0.10; % 10 cm variance along r
cov_r_theta = [var_theta^2,0; 0,var_r^2];

%% Loop for
fusion = false;
H = zeros(2,5,0);
R = zeros(2,2,0);
for k = 1:(numel(LaneView)/4)
    lane_global = LaneView(:,:,k);
    
    % Rototranslate the segment in onboard coordinate
    lane_onboard = rotoTranslateP( lane_global, -state(3),  trasl);
    
    if(norm(diff(lane_onboard,1,2))<min_length)
        continue;
    end
    fusion = true;
    
    % Compute the deltas in onboard and global frame
    %     delta_onboard = diff(lane_onboard,1,2);
    delta_global  = diff(lane_global,1,2);
    
    %% Simulate the measured data
    % Theta measured in onboard system
    %     theta_measured = atan(delta_onboard(2)/delta_onboard(1));   % NOT NEEDED for uncertanty propagation
    
    % Distance (wall-vehicle) squared based on measured data
    %     d_sq_measured = [-sin(theta_measured+pi/2),cos(theta_measured)]*lane_onboard(:,1);
    %     d_sq_measured = d_sq_measured*d_sq_measured;   % NOT NEEDED for uncertanty propagation
    
    %    % Measure vector
    %     measures = [theta_measured;d_sq_measured];  % Not needed for uncertanty propagation
    
    
    %% Compute the global information
    % Knowing at which wall refers the measure we can compute d and theta
    % of the vehicle respect to this walls
    
    % Theta global
    theta_global = atan(delta_global(2)/delta_global(1));   % h1
    %     h1 = theta_global-state(3); % NOT NEEDED for uncertanty propagation
    % from the estimated state of the system, reling on global information
    % we extimate that theta_measured is around h1
    
    % Distance squared in global reference frame
    %     d_global  = [cos(theta_global+pi/2),sin(theta_global+pi/2)]*(lane_global(:,1)-state(1:2));         % NOT NEEDED for uncertanty propagation
    %     h2 = d_global*d_global;     % NOT NEEDED for uncertanty propagation
    % from the estimated state of the system, reling on global information
    % we extimate that d_sq_measured is around h2
    
    %%  Compute kalman error  % NOT NEEDED FOR UNCERTANTY PROPAGATION
    %     eTheta = theta_measured - h1;  % measure - predicted measure
    %     while(eTheta>pi/2)  % normalize the anglle betwehen
    %         eTheta = eTheta - pi;  % gli angoli possono differire di pi xk non discriminiamo la direzione
    %     end
    %     delta_sq_d = d_sq_measured - h2; % measure - predicted measure
    %
    %     error = [eTheta;delta_sq_d]; % NOT NEEDED FOR UNCERTANTY PROPAGATION
    
    
    %% Compute covariance of the segments point
    
    
    
    % Find position of start and end point of the segment
    x_a = lane_onboard(1,1);
    x_b = lane_onboard(1,2);
    y_a = lane_onboard(2,1);
    y_b = lane_onboard(2,2);
    
    % Convert point in r-theta coordinate
    theta_a = atan2(y_a,x_a);
    r_a = sqrt(x_a^2+y_a^2);
    theta_b = atan2(y_b,x_b);
    %     r_b = sqrt(x_b^2+y_b^2);   % NOT NEEDED
    
    % Compute the jacobian of the trasformation [x,y] = f(r,theta)
    % | x |      | cos(theta) |
    % |   | = r* |            |
    % | y | =    | sin(theta) |
    
    % diff(x,r), diff(x,theta)
    j_a1 = [cos(theta_a),-r_a*sin(theta_a)];
    j_a2 = [sin(theta_a),r_a*cos(theta_a)];
    j_a  = [j_a1;j_a2];
    
    j_b1 = [cos(theta_b),-r_a*sin(theta_b)];
    j_b2 = [sin(theta_b),r_a*cos(theta_b)];
    j_b = [j_b1;j_b2];
    
    % Covariance of the point of the segment
    cov_a = j_a * cov_r_theta * j_a';
    cov_b = j_b * cov_r_theta * j_b';
    
    cov_points = zeros(4,4);
    cov_points(1:2,1:2) = cov_a;
    cov_points(3:4,3:4) = cov_b;
    
    
    %% Propagate the uncertanty of segment point (cov_a, cov_b) on theta_measured and d_sq_measured
    
    % Prop uncertanty on THETA MEASURED
    % The kinekt give us the point of the segment, here we assume that
    % the kinekt give us only the extreme of the segment. From this point
    % we can compute the theta vehicle wall as follow:
    % atan((x_a - x_b)/(y_a - y_b))
    % Remark that we have no the orientation of the wall, i.e. we have to
    % know position with less than 180° angular uncertanty
    
    diff_theta_xa = (y_a - y_b)/(x_b^2-2*x_a*x_b+y_a^2-2*y_a*y_b + y_b^2 + x_a^2);
    diff_theta_xb = (y_b - y_a)/(x_a^2-2*x_a*x_b+y_a^2-2*y_a*y_b + y_b^2 + x_b^2);
    
    diff_theta_ya = (x_b - x_a)/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_b-y_a)^2);
    diff_theta_yb = (x_a - x_b)/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_b-y_a)^2);
    
    
    % Prop uncertanty on D_SQ_MEASURED
    % Compute the uncertanty in distance measure from point A and B ( the
    % extreme of the segment)
    diff_dist_xa = ( 2*(x_a^2 - x_a*x_b + y_a*(-y_a + y_b))*(x_a^3 - 3*x_b*x_a^2 - x_b*(x_b^2 + 2*y_a^2 - 3*y_a*y_b + y_b^2) + x_a*(3*x_b^2 + 3*y_a^2 - 5*y_a*y_b + 2*y_b^2)))/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_a - y_b)^2)^2;
    diff_dist_xb = ( 2*(-x_a^2 + x_a*x_b + y_a*( y_a - y_b))*(y_a - y_b)*(2*x_a*y_a - x_b*y_a - x_a*y_b))/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_a - y_b)^2)^2;
    diff_dist_ya = (-2*( x_a^2 - x_a*x_b + y_a*(-y_a + y_b))*(x_a^2*(3*y_a - 2*y_b) + (y_a - y_b)^3 + x_b^2*(2*y_a - y_b) + x_a*x_b*(-5*y_a + 3*y_b)))/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_a - y_b)^2)^2;
    diff_dist_yb = ( 2*(x_a - x_b)*(2*x_a*y_a - x_b*y_a - x_a*y_b)*(x_a^2 - x_a*x_b + y_a*(-y_a + y_b)))/(x_a^2 - 2*x_a*x_b + x_b^2 + (y_a - y_b)^2)^2;
    
    
    % Jacobian matrix
    propMat = [diff_theta_xa,diff_theta_xb,diff_theta_ya,diff_theta_yb;...
        diff_dist_xa,diff_dist_xb,diff_dist_ya,diff_dist_yb];
    
    % Putting all togheder as measure covariance:
    % measure = [theta, dist_squared]    
    R = cat(3,R,propMat * cov_points * propMat');
    
    % Linearize the measure function h
    grad_h1 = [0,0,1,0,0];     % diff h1 respect to x,y,theta,omega,v
    grad_h2 = zeros(1,5);  % diff h2 respect to x,y,theta,omega,v
    
    
    % Se definiamo
    %    --> a = cos(theta_global+pi/2);
    %    --> b = sin(theta_global+pi/2);
    a = cos(theta_global+pi/2);
    b = sin(theta_global+pi/2);
    % Trascurando i termini che non dipendono dallo stato otterremo che
    % la distanza sarà data da:
    %  a^2*x^2 + 2*a*b*x*y + b^2*y^2   <-- x = stato(1) y = stato(2)
    
    % Differenziando la sopra in x otteniamo
    % 2*a^2*x + 2*a*b*y
    grad_h2(1,1) = 2*a^2*(state(1)-lane_global(1,1)) + 2*a*b*(state(2)-lane_global(2,1));
    
    % Differenziando per y analogamente otteniamo
    % 2*a*b*x + 2*b^2*y
    grad_h2(1,2) = 2*a*b*(state(1)-lane_global(1,1)) + 2*b^2*(state(2)-lane_global(2,1));
    
    % y = H*(x   <- linearization of the measure
    H = cat(3,H,[grad_h1;grad_h2]);
    
%     S = H*P*H' + R;
    
%     K = (P*H')/S;
    
    % Update the state uncertanty
%     P = (eye(3,3)-K*H)*P;
end

n = size(H,3);
H_new = zeros(2*n,5);
R_new = zeros(2*n,2*n);
for k=1:n
    H_new(2*(k-1)+1:2*k,:) = H(:,:,k);
    R_new(2*(k-1)+1:2*k,2*(k-1)+1:2*k) = R(:,:,k);
end

end




