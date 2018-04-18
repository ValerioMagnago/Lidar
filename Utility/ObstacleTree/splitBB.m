function [ BB_range, Leaf, Padre, id ] = splitBB( BB_range, objActive,orient, objBB, objCenter, id,padre)
%SPLITBB Summary of this function goes here
%   Detailed explanation goes here

debug_plot = 0;

orient = rem(orient+1,2);
nObj = numel(objActive);



if std(objCenter(1,objActive))>std(objCenter(2,objActive))  
    % if points are more spread along x axis
    meanX = mean(objCenter(1,objActive));
    logic = objCenter(1,objActive) < meanX;
else
    % if points are more spread along y axis
    meanY = mean(objCenter(2,objActive));
    logic = objCenter(2,objActive) < meanY;
end


if debug_plot
    global map_fig
    figure(map_fig);
    bbPlot = findobj('Tag','bbPlot');
    selectedObjs = findobj('Tag','selectedObj');
    
    for i=1:numel(selectedObjs)
        delete(selectedObjs(i));
    end
    
    fill(BB_range(1,[1,2,2,1]),BB_range(2,[1,1,2,2]),'r','facealpha',0.1,'Tag','bbPlot');
    for i=objActive
        xV = objBB(1,[1,2,2,1,1],i);
        yV = objBB(2,[1,1,2,2,1],i);
        plot(xV,yV,'g','Tag','selectedObj');
    end
    
    if sum(logic)==0 || sum(logic)==nObj 
      plot(BB_range(1,[1,2,2,1,1]),BB_range(2,[1,1,2,2,1]),'b','linewidth',2,'Tag','bbDone');  
    end
    
    drawnow;
    pause();
    
end


% check if we are splitting the data or not
if sum(logic)==0 || sum(logic)==nObj %% add all the data
    % We are done in the splitting part
    BB_range = [];
    Padre  = padre *ones(1,numel(objActive));
    Leaf = zeros(1,nObj);
    for i=1:numel(logic)
        BB_range =  cat(3,BB_range,createBoundingBox( objBB, objActive(i) ));
        Leaf(i) = objActive(i);
        id = [id,id(end)+1];   
    end
else
    id   = [id,id(end)+1];
    myId = id(end);
    
    % Divide the object in 2
    objActive1 = objActive(logic);
    BB_range1_tmp = createBoundingBox( objBB, objActive1 );
    
    objActive2 = objActive(not(logic));
    BB_range2_tmp = createBoundingBox( objBB, objActive2 );
    [BB_range1, Leaf1, Padre1, id]  = splitBB(BB_range1_tmp, objActive1,orient,objBB, objCenter,  id, myId);
    [BB_range2, Leaf2, Padre2, id]  = splitBB(BB_range2_tmp, objActive2,orient,objBB, objCenter,  id, myId);
    
    Padre    = [padre,Padre1 , Padre2];
    Leaf     = [0, Leaf1, Leaf2];   % if is not a leaf add 0
    BB_range = cat(3,BB_range,BB_range1,BB_range2);
end


end

