clear; close all; clc;

dataFolder = 'processed_output_hres'; % processed files
extrinsicFile = 'cal_laser_ex_hres.m'; % extrinsics file
intrinsicFile = 'calibrationhres.m'; % %instrinsics file
imgFolder = 'ex_images_hres'; % bw line images

% load instrinsics
run(intrinsicFile);

K = cameraParamsInhres.IntrinsicMatrix';

% load extrinsics
run(extrinsicFile);

% access images processed images
imageFiles = dir(fullfile(imgFolder, 'imgLaser_*.jpg'));

% storings
laserPoints3D = [];
imageIndices = []; % image indexes

disp('starting triangulation');
% choose minimum number of images in folder to use
numExtrinsics = length(cameraParamsLaserhres.PatternExtrinsics);
if numExtrinsics ~= length(imageFiles)
    loopLimit = min(numExtrinsics, length(imageFiles));
else
    loopLimit = length(imageFiles);
end

for k = 1:loopLimit
    currentFilename = imageFiles(k).name;

    % match images
    token = regexp(currentFilename, 'imgLaser_(\d+)\.jpg', 'tokens', 'once');
    if isempty(token)
        continue;
    end
    imageNumberStr = token{1};
    imageNumberNum = str2double(imageNumberStr);
    
    comDataFilename = fullfile(dataFolder, ['laser', imageNumberStr, '_com_data.txt']);
    if ~exist(comDataFilename, 'file')
        continue;
    end
    
    % load CoM
    comDataMatrix = readmatrix(comDataFilename, 'Delimiter', '\t');
    if isempty(comDataMatrix)
        continue;
    end
    
    v_pixels_1based = comDataMatrix(:,1);         % row (v) in 1-based
    u_pixels_0based = comDataMatrix(:,2);         % column (u) in 0-based
    validPoints = ~isnan(u_pixels_0based);
    v_pixels_1based = v_pixels_1based(validPoints);
    u_pixels_0based = u_pixels_0based(validPoints);
    if isempty(u_pixels_0based)
        continue;
    end
    
    % convert to 1-based column pixel coordinate
    u_pixels_1based = u_pixels_0based + 1;
    p_2D_distorted = [u_pixels_1based, v_pixels_1based]; % [col, row]
    
    % get patterns from extrinsics
    A_stored = cameraParamsLaserhres.PatternExtrinsics(k).A;
    
    R_pattern_to_cam = A_stored(1:3,1:3);
    P0_pattern_origin_in_cam = A_stored(1:3,4); % camera coordinates

    % pattern's normal in camera coords
    n_RC = R_pattern_to_cam * [0;0;1];
    n_RC = n_RC / norm(n_RC); % normalize
    
    % undistort
    p_2D_undistorted = undistortPoints(p_2D_distorted, cameraParamsInhres);
    
    % homogeneous pixel coords (3xN)
    Npts = size(p_2D_undistorted, 1);
    homog = [p_2D_undistorted, ones(Npts,1)]';
    
    % Compute normalized image coordinates: inv(K) * homog  -> use backslash
    p_norm_homog = (K \ homog)';                   % N x 3
    p_norm = p_norm_homog(:, 1:2);                 % [x_norm, y_norm]
    
    % Ray directions (camera frame): each ray v = [x_norm; y_norm; 1]
    ray_directions_RC = [p_norm, ones(Npts,1)]';  % 3 x N
    
    % --------------------- Ray-plane intersection ---------------------------
    % Solve for t such that n' * (t * v) = n' * P0  => t = (n' * P0) / (n' * v)
    numerator = n_RC' * P0_pattern_origin_in_cam;  % scalar
    denominator = n_RC' * ray_directions_RC;       % 1 x N
    
    eps_den = 1e-9;
    valid = abs(denominator) > eps_den;            % avoid near-parallel rays
    t_vals = nan(1, Npts);
    t_vals(valid) = numerator ./ denominator(valid);
    
    % Build 3D points (camera coordinates) and filter invalid
    P_RC_current = nan(Npts, 3);
    P_RC_current(valid, :) = (ray_directions_RC(:, valid) .* t_vals(valid))';  % N x 3
    % Remove any rows with NaN (parallel rays)
    good_mask = ~any(isnan(P_RC_current), 2);
    P_RC_current = P_RC_current(good_mask, :);
    if isempty(P_RC_current)
        warning('No valid 3D points for image %s after filtering parallel rays.', currentFilename);
        continue;
    end
    
    % check errors
    % reprojection
    proj_h = (K * P_RC_current')'; % Nx3
    proj_px = proj_h(:,1:2) ./ proj_h(:,3);

    undist_kept = p_2D_undistorted(good_mask, :);
    reproj_err = sqrt(sum((proj_px - undist_kept).^2, 2));
    median_reproj = median(reproj_err);
    fprintf('Image %s: %d pts, median reproj err = %.3f px\n', currentFilename, size(P_RC_current,1), median_reproj);
    
    % remove point with large errors
    reproj_thresh_px = 5; 
    keep_reproj = reproj_err <= reproj_thresh_px;
    if ~all(keep_reproj)
        P_RC_current = P_RC_current(keep_reproj, :);
        if isempty(P_RC_current)
            warning('After reproj filtering, no points remain for image %s.', currentFilename);
            continue;
        end
    end
    
    % make cloud line 
    numPoints = size(P_RC_current, 1);
    laserPoints3D = [laserPoints3D; P_RC_current];
    imageIndices = [imageIndices; repmat(imageNumberNum, numPoints, 1)];
    
    fprintf('  Processed image #%s: added %d points (after filtering).\n', imageNumberStr, numPoints);
end

% fitting
use_pcfitplane = exist('pcfitplane', 'file') == 2;

if use_pcfitplane
    ptCloud = pointCloud(laserPoints3D);
    maxDistance = 0.5; 

    try
        [model, inlierIndices] = pcfitplane(ptCloud, maxDistance);
        fprintf('pcfitplane found %d inliers / %d points.\n', numel(inlierIndices), ptCloud.Count);
        % model.Parameters [A B C D] for plane Ax+By+Cz+D=0
        plane_params = model.Parameters(:);
        
        % normalise
        normal_norm = norm(plane_params(1:3));
        if normal_norm > 0
            plane_params = plane_params / normal_norm;
        end
    catch ME
        disp('fitting failed')
    end
end


% make c > 0
if plane_params(3) < 0
    plane_params = -plane_params;
end

A = plane_params(1);
B = plane_params(2);
C = plane_params(3);
D = plane_params(4);

disp('Final Fitted Laser Plane Equation (A X_C + B Y_C + C Z_C + D = 0)');
fprintf('A: %.8f\n', A);
fprintf('B: %.8f\n', B);
fprintf('C: %.8f\n', C);
fprintf('D: %.8f\n', D);

% plot
figure('Name', '3D Laser Plane Reconstruction');
max_k = max(imageIndices);
if isempty(max_k) || isnan(max_k)
    max_k = 1;
end
colormap(jet(max_k));
pcshow(laserPoints3D, imageIndices);
xlabel('X_C (mm)');
ylabel('Y_C (mm)');
zlabel('Z_C (mm)');
title('Triangulated Laser Points');
c = colorbar;
c.Label.String = 'Image Index (k)';
caxis([min(imageIndices) max(imageIndices)]);
if max_k <= 20
    c.Ticks = unique(imageIndices);
end
hold on;

% Plot fitted plane
if abs(C) > 1e-9
    x_lim = [min(laserPoints3D(:,1)), max(laserPoints3D(:,1))];
    y_lim = [min(laserPoints3D(:,2)), max(laserPoints3D(:,2))];
    [X_grid, Y_grid] = meshgrid(linspace(x_lim(1), x_lim(2), 20), linspace(y_lim(1), y_lim(2), 20));
    Z_grid = (-A*X_grid - B*Y_grid - D) / C;
    surf(X_grid, Y_grid, Z_grid, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'blue');
end
axis equal; grid on; hold off;

% check precision
P_homog_all = [laserPoints3D, ones(size(laserPoints3D,1),1)];
signedDistancesAll = P_homog_all * plane_params;
residualErrors = abs(signedDistancesAll);

maxError = max(residualErrors);
meanError = mean(residualErrors);
stdDevError = std(residualErrors);

fprintf('Precision Metrics (Residual Errors)\n');
fprintf('Maximum Residual Error: %.6f %s\n', maxError, cameraParamsLaserhres.WorldUnits);
fprintf('Mean Residual Error:    %.6f %s\n', meanError, cameraParamsLaserhres.WorldUnits);
fprintf('Std Dev (Precision):    %.6f %s\n', stdDevError, cameraParamsLaserhres.WorldUnits);
