% demonstrates stereo visual odometry on an image sequence
disp('===========================');
clear all; close all; dbstop error;

% parameter settings (for an example, please download
% sequence '2010_03_09_drive_0019' from www.cvlibs.net)
img_dir     = '/home/geiger/5_Data/kitti/2011_stereo/2010_03_09_drive_0019';
%img_dir     = 'C:\Users\geiger\Desktop\2010_03_09_drive_0019';
param.f     = 645.2;
param.cu    = 635.9;
param.cv    = 194.1;
param.base  = 0.571;
first_frame = 0;
last_frame  = 372;

% init visual odometry
visualOdometryStereoMex('init',param);

% init transformation matrix array
Tr_total{1} = eye(4);

% create figure
figure('Color',[1 1 1]);
ha1 = axes('Position',[0.05,0.7,0.9,0.25]);
axis off;
ha2 = axes('Position',[0.05,0.05,0.9,0.6]);
set(gca,'XTick',-500:10:500);
set(gca,'YTick',-500:10:500);
axis equal, grid on, hold on;

% for all frames do
for frame=first_frame:last_frame
  
  % 1-index
  k = frame-first_frame+1;
  
  % read current images
  I1 = imread([img_dir '/I1_' num2str(frame,'%06d') '.png']);
  I2 = imread([img_dir '/I2_' num2str(frame,'%06d') '.png']);

  % compute and accumulate egomotion
  Tr = visualOdometryStereoMex('process',I1,I2);
  if k>1
    Tr_total{k} = Tr_total{k-1}*inv(Tr);
  end

  % update image
  axes(ha1); cla;
  imagesc(I1); colormap(gray);
  axis off;
  
  % update trajectory
  axes(ha2);
  if k>1
    plot([Tr_total{k-1}(1,4) Tr_total{k}(1,4)], ...
         [Tr_total{k-1}(3,4) Tr_total{k}(3,4)],'-xb','LineWidth',1);
  end
  pause(0.05); refresh;

  % output statistics
  num_matches = visualOdometryStereoMex('num_matches');
  num_inliers = visualOdometryStereoMex('num_inliers');
  disp(['Frame: ' num2str(frame) ...
        ', Matches: ' num2str(num_matches) ...
        ', Inliers: ' num2str(100*num_inliers/num_matches,'%.1f') ,' %']);
end

% release visual odometry
visualOdometryStereoMex('close');
