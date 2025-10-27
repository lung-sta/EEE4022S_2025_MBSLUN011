clear; close all; clc;
% green image tilted by 1.9 (0.3 for day pic) degrees to make straight 
% red image tilted by 1.3 (0.2 for day pic) degrees to make straight 
%% picture
imageFile = 'green_laser_day_1_vert.jpg';

%% load image
img = im2double(imread(imageFile));

I = img(:,:,2); % extract channel

figure; imshow(I,[]);
disp('Draw a box around the laser line and double-click to confirm.');
roi = imcrop(I);  % interactive crop
close;

figure; imshow(roi,[]); title('ROI');
[nRows, nCols] = size(roi);

%% focus on column's intensities
fwhm_px = nan(1, nRows); 
centroid_px = nan(1, nRows);
% max_intensity_row = nan(1, nRows);
x = (1:nCols)'; % col index

for row = 1:nRows 
    profile = roi(row, :)';

    profile = movmean(profile,3);   % smooth before fitting
    % normalize to 0 – 1
    profile = profile - min(profile);
    if max(profile) > 0
        profile = profile / max(profile);
    end
    % guassian fit
    try
        gaussEqn = 'a*exp(-0.5*((x-b)/c)^2)+d';
        xdata = x;
        ydata = profile;
        s = fitoptions('Method','NonlinearLeastSquares',...
                       'StartPoint',[1, mean(x), 2, 0],... 
                       'Lower',[0,1,0,0],...
                       'Upper',[2,max(x),Inf,1]); 
        f = fit(xdata, ydata, gaussEqn, s);
        sigma = f.c;
        centroid_px(row) = f.b; % store centroid
        fwhm_px(row) = 2.3548 * sigma;
    catch
        fwhm_px(row) = NaN;
        centroid_px(row) = NaN;
    end
end

%% gather
validWidth = ~isnan(fwhm_px);
validCentroid = ~isnan(centroid_px);

mean_width = mean(fwhm_px(~isnan(fwhm_px)));
std_width  = std(fwhm_px(~isnan(fwhm_px)));
mean_centroid = mean(centroid_px(validCentroid));
straightness = std(centroid_px(~isnan(centroid_px)));

fprintf('--- RESULTS (PIXELS) ---\n');
fprintf('Mean FWHM: %.2f px ± %.2f px\n', mean_width, std_width);
fprintf('Straightness: %.2f px\n', straightness);

%% plot
figure;
subplot(2,1,1);
plot(fwhm_px, 'LineWidth',1.2);
xlabel('Position along line (pixels)');
ylabel('FWHM (pixels)');
title('Laser line width');
grid on;

subplot(2,1,2);
plot(centroid_px - mean_centroid, 'LineWidth',1.2);
xlabel('Position along line (pixels)');
ylabel('Centroid deviation (pixels)');
title('Line straightness');
grid on;