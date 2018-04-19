function [valle,picco] = trovaPicchi(list_d)
%TROVAPICCHI Summary of this function goes here
%   Detailed explanation goes here

min_d = 1.5;
last_d = min_d;

i = 1;
sali = true;
last_sali = true;
picco = [];
valle = [];

forse_picco = false;
forse_valle = false;
if(list_d(3)>list_d(2))
    sali = false;
    last_sali = false;
    salto = -1.1;
else
    sali = true;
    last_sali = true;
    salto = 1.1;
end 
  
 
for i=2:numel(list_d)
  d = list_d(i);
  
%   % If d is too small skip it
%   if d<min_d
%       continue;
%   end
  
  % Detect if we are increasing or decreasing the value
  if(d>(last_d+1e-4))
     sali = true;
  elseif(d<(last_d-1e-4))
     sali = false;
  end  
  
  % Controllare se siamo in un massimo 
  if(last_sali) % prima saliva
     if(not(sali) && salto>1) % ora scende, e controlliamo il salto
         if(not(forse_picco))
            picco = [picco,[i-1;salto]];
         else
             if(salto>picco(2,end))
                 picco(:,end) = [i-1;salto];
             end
         end         
         salto = 0;  
         forse_valle = false;         
         
     elseif(not(sali) && salto<=1) 
         % Se il picco e' salito poco mettiamo in dubbio valle precedente
         if not(isempty(valle))
            forse_valle = true;
            salto = salto + valle(2,end);
         end         
     end
     
     
  % Controllare se siamo in un minimo
  else % prima scendeva
      if(sali && salto < -1) % ora sale
         if(not(forse_valle))
            valle = [valle,[i-1;salto]];
         else
             if(salto<valle(2,end))
                 valle(:,end) = [i-1;salto];
             end
         end          
         salto = 0;  
         forse_picco = false;
         
         
      elseif(sali && salto >= -1)
          % Se valle e' scesa di poco mettiamo in dubbio picco precedente
         if not(isempty(picco))            
            forse_picco = true;
            salto = salto + picco(2,end);
         end
      end
  end
  
  salto = salto + d-last_d;
  last_d = d;
  last_sali = sali;
  
%   sali
%    delete(a);
%    delete(b);
%    a = plot([i,i],[0,1]*list_d(i),'r');
%    b = plot(i,list_d(i),'xr');
%    drawnow();
%    pause();
  
end

if(salto < -1) % ora sale
     if(not(forse_valle))
        valle = [valle,[i-1;salto]];
     else
         if(salto<valle(2,end))
             valle(:,end) = [i-1;salto];
         end
     end          
end

% delete(a);
% delete(b);
% 
% for i=picco(1,:)
%     plot(i,list_d(i),'xr');
% end
% for i=valle(1,:)
%     plot(i,list_d(i),'or');
% end
if(isempty(valle))
    valle = [];
else    
    valle = valle(1,:);
end


if(isempty(valle))
    picco = [];
else    
    picco = picco(1,:);
end

end

