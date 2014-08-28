function plotTrack(I,p_matched,i_matched)

% show image
figure;axes('Position',[0,0,1,1]);
imshow(uint8(I)),hold on;

% for all matches in the last frame do
for i=1:size(i_matched{end},2)
  
  ind = i;
  
  % init track with latest matches
  p1 = p_matched{end}(3:4,ind);
  p2 = p_matched{end}(1:2,ind);
  p  = [p1 p2];
  
  % augment track into the past
  for j=length(p_matched)-1:-1:1
    
    % find backwards
    ind = find(i_matched{j}(2,:)==i_matched{j+1}(1,ind));
    if isempty(ind)
      break;
    end
    
    p3 = p_matched{j}(1:2,ind);
    p  = [p p3];
  end
  
  track_length = length(p_matched)-j;
  if track_length<2
    continue;
  end
  
  % plot track (if long enough)
  v1 = p(:,1)-p(:,end);
  v2 = p(:,1)-p(:,end);
  if norm(v1) >= 8
    col = hsv2rgb([atan2(v2(2),v2(1))/(2*pi)+0.5 1 0.9]);
    plot(p(1,:),p(2,:),'-s','Color',col,'LineWidth',1,'MarkerSize',2);
  end
end
