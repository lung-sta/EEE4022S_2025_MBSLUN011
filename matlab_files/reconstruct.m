%% parameter values
fx = 3.040455013917193e+03; fy = 3.034642712884210e+03; cx = 2.059069102193600e+03; cy = 1.102880143893566e+03;

% plane
nx = -0.85185341; ny = 0.00173490; nz = 0.52377739;
plane_d = -213.73541010;

%% matrices
% intrinsic
K = [fx,  0, cx;
     0,  fy, cy;
     0,   0,  1];

% Laser plane in world coordinates: n'*X+d=0
n = [nx; ny; nz];   % normal
d = plane_d;        % offset

% binary curve images
img_folder = 'bw_g2_curve';
img_pattern = fullfile(img_folder, 'bw*_blank_2g_com.png');

% translation step between images
dx = 1;   % mm
shift_dir = +1;

output_file = 'recon_points_blank_2g.txt';

%% gather
% inverse intrinsics
Kinv = inv(K);

% find files
files = dir(img_pattern);
[~, idx] = sort_nat({files.name}); % natural sort
files = files(idx);

all_points = [];

fprintf('Found %d images.\n', numel(files));

%% main loop
for i = 1:numel(files)
    fname = fullfile(img_folder, files(i).name); % get file name
    bw = imread(fname); % read to variable

    [v, u] = find(bw);  % (row, col)
    pixels = [u, v];

    pts = nan(size(pixels,1), 3);
    for j = 1:size(pixels,1)
        uv = [pixels(j,1); pixels(j,2); 1];
        d_c = Kinv * uv;      % ray direction in camera

        denom = dot(n, d_c);
        if abs(denom) < 1e-12 % dont divide by 0
            continue;
        end

        lambda = -d / denom;  % find lambda
        Xc = lambda * d_c;    % 3D point in camera
        pts(j,:) = Xc';       % store points
    end

    pts = pts(~any(isnan(pts),2), :); % remove NaNs

    % shift object 1mm 
    shift_vec = [-(i-1)*dx, 0, 0];
    pts_shifted = pts + shift_vec; % store again 

    all_points = [all_points; pts_shifted]; % store globally
    fprintf('Processed %s: %d pts, shift = [%g %g %g]\n', ...
            files(i).name, size(pts,1), shift_vec);
end

%% save to .ply
num_frames = numel(files);
colors = uint8(255*jet(num_frames));  % generate a colormap for x values

RGB = zeros(size(all_points,1),3,'uint8');

start_idx = 1;
for i = 1:num_frames
    fname = fullfile(img_folder, files(i).name);
    bw = imread(fname);
    [v,u] = find(bw);
    n_pts = numel(u);

    RGB(start_idx:start_idx+n_pts-1,:) = repmat(colors(i,:), n_pts,1);
    start_idx = start_idx + n_pts;
end

ptCloud = pointCloud(all_points,'Color',RGB);
pcwrite(ptCloud, 'recon_points_blank_2g_colored.ply','Encoding','ascii');
fprintf('saved ply: recon_points_blank_2g_colored.ply\n');

%% save
fprintf('Total points: %d\n', size(all_points,1));
writematrix(all_points, output_file, 'Delimiter', ' ', 'FileType', 'text');
fprintf('Saved to %s\n', output_file);

%% graph
figure; scatter3(all_points(:,1), all_points(:,2), all_points(:,3), 3, '.');
axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
title('Reconstructed 3D Points');

%% sort file names
function [sorted, index] = sort_nat(cellArray)
    [~, index] = sort( regexprep(cellArray, '\d+', '${num2str(str2double($0),''%06d'')}' ) );
    sorted = cellArray(index);
end
