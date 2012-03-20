% takes a monocular input sequence and computes 3d point clouds
disp('===========================');
clear all; close all; dbstop error;

% parameter settings (for an example, please download
% sequence '2010_03_09_drive_0019' from www.cvlibs.net)
%img_dir      = '/home/geiger/5_Data/karlsruhe_dataset/2011_stereo/2010_03_09_drive_0019';
img_dir     = 'C:\Users\geiger\Desktop\2010_03_09_drive_0019';
param.f      = 645.2;
param.cu     = 635.9;
param.cv     = 194.1;
param.height = 1.6;
param.pitch  = -0.08;
param.max_features = 1000; % disable bucketing
first_frame  = 0;
last_frame   = 372;

% init matcher + odometry objects
visualOdometryMonoMex('init',param);
reconstructionMex('init',param.f,param.cu,param.cv);

% init transformation matrix array
Tr_total{1} = eye(4);

% create figure
figure('Color',[1 1 1]),axes('Position',[0,0,1,1]); axis equal, axis off;

% for all frames do
replace = 0;
for frame=first_frame:last_frame
  
  % 1-index
  k = frame-first_frame+1;
  
  % read current images
  I = imread([img_dir '/I1_' num2str(frame,'%06d') '.png']);

  % compute and accumulate egomotion
  Tr = visualOdometryMonoMex('process',I,replace);
  
  % from second frame on
  if k<=1
    continue;
  end
    
  % if something went wrong (e.g., small motion)
  if isempty(Tr)
    
    % copy pose
    Tr_total{k} = Tr_total{k-1};
    
    % replace current frame in next iteration
    replace = 1;

  % otherwise
  else

    % update pose
    Tr_total{k} = Tr_total{k-1}*inv(Tr);

    % reconstruction (only elements above road plane)
    p_matched = visualOdometryMonoMex('get_matches');
    i_matched = visualOdometryMonoMex('get_indices');
    reconstructionMex('update',p_matched,i_matched,Tr,2,2,30,3);
    
    % cycle ring buffer in next iteration
    replace = 0;
  end

  % get reconstructed 3d points
  p = reconstructionMex('getpoints');

  % update point cloud and draw trajectory
  cla; hold on; view(0,0);
  cla,plot3(p(1,:),p(2,:),p(3,:),'.b','MarkerSize',1),axis equal;
  for i=2:frame
    plot3([Tr_total{i-1}(1,4) Tr_total{i}(1,4)], ...
          [Tr_total{i-1}(2,4) Tr_total{i}(2,4)], ...
          [Tr_total{i-1}(3,4) Tr_total{i}(3,4)],'-xr','LineWidth',1);
  end
  pause(0.05);
  refresh;

  % output statistics
  num_matches = visualOdometryMonoMex('num_matches');
  num_inliers = visualOdometryMonoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
end

% release objects
visualOdometryMex('close');
reconstructionMex('close');
