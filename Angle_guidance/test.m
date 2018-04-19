  %% Compute control
    angles = linspace(lidarScan.angle_min,lidarScan.angle_max,numel(lidarScan.ranges));
    Atot = 0;
    sum_angle = 0;
    sum_d = 0;
    stop = false;
    
    list_d = [];
    prev_d = min(lidarScan.ranges(1),lidarScan.range_max);
    porte = [];
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
                porte = [porte; [start_port,end_port]];
            end
            start_port = ang_id+1;
            end_port = ang_id;
        end
        
        if(d < TART_D)
            stop = true;
        end
        
        if(abs(d-prev_d)>larg_porta)
            porte = [porte; ang_id - [1,0]];
        end
        
        dA = d*d;
        list_d = [list_d,d];
        
        Atot = Atot + dA;
        sum_angle = sum_angle + dA*angolo;
        sum_d     = sum_d     + dA*d;
        
        % Update last d
        prev_d = d;
    end
    
    theta = sum_angle / Atot;
    dist  = sum_d / Atot;
    
    
    if(stop || Atot < 300)
        v = 0;
        omega = -0.5; % turn right
    else
        v = K_d*dist;
        omega = K_ang*theta;
    end