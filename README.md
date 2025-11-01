# EEE4022S_2025_MBSLUN011
## 3D Reconstruction of Leading Edge Erosion (LEE)

This document presents the methodology used to perform three-dimensional (3D) reconstruction of Leading Edge Erosion (LEE) damage on wind turbine blades using a striped light projection technique (a form of structured light). The method combines theoretical principles of structured light imaging with a practical experimental setup for capturing and analysing the damage topography of the wind turbine (WT) leading edge.

## System Setup and Calibration
### Camera Calibration

An 8 × 6 checkerboard pattern with 25 mm squares was used as the calibration target, printed on a rigid surface to avoid bending.
Images were captured as the checkerboard was moved and rotated across different parts of the camera’s field of view (FoV) to ensure complete coverage and calibration accuracy.
Images used: 20–30
Tools: OpenCV and MATLAB
Checkerboard square size: 25 mm
Outputs: Intrinsic parameters (focal length, principal point, and distortion coefficients)

### Laser Line Calibration
After camera calibration, the laser plane was determined relative to the camera coordinates.
The line laser was projected onto the checkerboard at five distances.
Each image was processed to obtain the center of mass (CoM) of the laser line.
Pixel coordinates were transformed into 3D coordinates using the known camera parameters.
The combined 3D points from all five images formed several intersecting lines within the laser plane.
A RANSAC-based plane fitting was applied to compute the best-fit laser plane.
The result represents the real-world laser plane equation, enabling triangulation of 3D points during reconstruction.

## Data Acquisition
### Pi Camera Configuration

A Raspberry Pi camera and the libcamera software were used. Automatic adjustments were disabled to maintain consistent image quality.
- Exposure time	100 ms
- Gain	1.0
- Exposure mode	Normal
- Focus	Manual (2.5)
- White balance	Auto

Two image resolutions were tested to analyse reconstruction accuracy:
- Low resolution: 640 × 480 (307,200 pixels)
- High resolution: 4096 × 2160 (8,847,360 pixels)

## Data Capture
147 images were captured per scan, with the blade manually translated 1 mm per frame.
Images were stored in .jpg format and labeled according to calibration type, blade ID, laser color, and resolution.
There were 5 blades, 2 resolutions, and 2 laser modules, giving 15 datasets (plus 4 calibration sets).

Configuration	Laser	Resolution
- 1	Red Laser	Low Resolution
- 2	Red Laser	High Resolution
- 3	Green Laser	High Resolution
This configuration minimised total data volume while maintaining analysis completeness.

## Data Processing and Reconstruction
### Image Processing
Each RGB image was processed in MATLAB to isolate the laser line and prepare data for 3D reconstruction.
Processing steps:
- Colour Channel Extraction
- Red laser → Red channel
- Green laser → Green channel
- Intensity Thresholding
- Convert to binary using a threshold value (pixels above threshold = laser line).
- Noise Removal
- Remove specks using area-based filtering.
- Limit analysis to a Region of Interest (ROI) where the blade appears.
- Horizontal Center of Mass (HCoM)
- For each image row, compute the horizontal center of mass of white pixels.
Produces a single-pixel-width line representing the laser trace.

## 3D Reconstruction
Using the camera’s intrinsic parameters (fx, fy, cx, cy) and the laser plane equation (a, b, c, d), each image is converted into 3D coordinates.
Reconstruction steps:
- Identify all non-zero (laser) pixels.
- Compute the corresponding camera ray for each pixel.
- Calculate where the ray intersects the laser plane using:
- - λ = -d / (nᵀ · d_c)
- Compute the 3D coordinates for each pixel and shift by 1 mm along the x-axis per image.
- Combine all results into a single point cloud and export as a .ply file.
This was repeated for all 15 blade datasets.

## Summary

This methodology provides a complete workflow for structured light 3D reconstruction of wind turbine blade erosion:
Precise geometric calibration (camera and laser).
Controlled data acquisition under consistent lighting and focus.
Robust image processing for accurate laser line detection.
3D triangulation to reconstruct surface topography.
The resulting 3D data enables:
Quantitative analysis of erosion severity

Comparison across resolutions and laser types

Use in predictive maintenance and blade surface modeling
