% demonstrates monocular feature tracking (via feature indices)
disp('===========================');
clear all; dbstop error; close all;

% matching parameters
param.nms_n                  = 2;   % non-max-suppression: min. distance between maxima (in pixels)
param.nms_tau                = 50;  % non-max-suppression: interest point peakiness threshold
param.match_binsize          = 50;  % matching bin width/height (affects efficiency only)
param.match_radius           = 200; % matching radius (du/dv in pixels)
param.match_disp_tolerance   = 1;   % du tolerance for stereo matches (in pixels)
param.outlier_disp_tolerance = 5;   % outlier removal: disparity tolerance (in pixels)
param.outlier_flow_tolerance = 5;   % outlier removal: flow tolerance (in pixels)
param.multi_stage            = 1;   % 0=disabled,1=multistage matching (denser and faster)
param.half_resolution        = 1;   % 0=disabled,1=match at half resolution, refine at full resolution
param.refinement             = 2;   % refinement (0=none,1=pixel,2=subpixel)

% init matcher
matcherMex('init',param);

% push back first image
I = imread('../img/I1_000000.png');
matcherMex('push',I);

% feature tracks
tracks = {};

% start matching
for i=1:6
  I = imread(['../img/I1_' num2str(i,'%06d') '.png']);
  tic; matcherMex('push',I);
  disp(['Feature detection: ' num2str(toc) ' seconds']);
  tic; matcherMex('match',0);
  p_matched{i} = matcherMex('get_matches',0);
  i_matched{i} = matcherMex('get_indices',0);
  disp(['Feature matching:  ' num2str(toc) ' seconds']);
end

% close matcher
matcherMex('close');

% show matching results
disp('Plotting ...');
plotTrack(I,p_matched,i_matched);
