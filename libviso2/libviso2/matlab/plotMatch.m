function plotMatch(I,p_matched,method,inliers)

if nargin<3
  method = 2;
end

if nargin<4
  inliers = 1:size(p_matched,2);
end

p_matched = p_matched';

% show image
cla,imshow(uint8(I)),hold on;

% show matches
if method==0

  for i=1:size(p_matched,1)
    col = [1 0 0];
    if ~any(inliers==i)
      col = [0 0 1];
    end
    line([p_matched(i,1) p_matched(i,3)], ...
         [p_matched(i,2) p_matched(i,4)], 'Color', col,'LineWidth',1);
    plot(p_matched(i,3),p_matched(i,4),'s', 'Color', col,'LineWidth',1,'MarkerSize',2);
    %text(p_matched(i,3),p_matched(i,4),sprintf('%d',i));
  end
  
elseif method==1
  
  disp      = p_matched(:,1)-p_matched(:,3);
  disp = min(disp,50);
  max_disp  = max(disp(inliers));
  %max_disp  = min(max_disp,50);

  for i=1:size(p_matched,1)
    c = abs(disp(i)/max_disp);
    col = [c 1-c 0];
    if ~any(inliers==i)
      col = [0 0 1];
    end
    line([p_matched(i,1) p_matched(i,1)], ...
         [p_matched(i,2) p_matched(i,4)], 'Color', col,'LineWidth',2);
    plot(p_matched(i,1),p_matched(i,2),'s', 'Color', col,'LineWidth',2,'MarkerSize',2);
  end
  
else
  
  disp      = p_matched(:,1)-p_matched(:,3);
  max_disp  = max(disp(inliers));
  %max_disp  = 80;

  for i=1:size(p_matched,1)
    c = min(abs(disp(i)/(max_disp+0.1)),1);
    col = [c 1-c 0];
    if ~any(inliers==i)
      col = [0 0 1];
    end
    line([p_matched(i,1) p_matched(i,5)], ...
         [p_matched(i,2) p_matched(i,6)], 'Color', col,'LineWidth',2);
    plot(p_matched(i,5),p_matched(i,6),'s', 'Color', col,'LineWidth',2,'MarkerSize',3);
  end
  
end


