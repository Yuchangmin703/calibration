>> %% ==========================================================================
%  HW40 카메라 캘리브레이션 스크립트 (ROS2 노드 맞춤형)
%% ==========================================================================
clear; clc; close all;

%% =========================================
%  사용자 설정
%% =========================================
% 체커보드 사각형 한 칸 크기 (5cm = 0.05m)
squareSize = 0.05;   

% 이미지 폴더 경로
intrinsicDir  = '~/calibration/intrinsic_images';   % 내부 파라미터용 
extrinsicDir  = '~/calibration/extrinsic_images';   % 외부 파라미터용 
outputDir     = '~/calibration/results';
if ~exist(outputDir, 'dir'), mkdir(outputDir); end

%% ==========================================================================
%  Phase 1: 내부 파라미터 (Intrinsics) 캘리브레이션
%% ==========================================================================
fprintf('\n===== Phase 1: 내부 파라미터 캘리브레이션 =====\n\n');
intrinsicImages = imageDatastore(intrinsicDir);
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(intrinsicImages.Files);
fprintf('체커보드 검출 성공: %d / %d 장\n', sum(imagesUsed), numel(imagesUsed));

worldPoints = patternWorldPoints('checkerboard', boardSize, squareSize);

I = readimage(intrinsicImages, find(imagesUsed, 1));
imageSize = [size(I, 1), size(I, 2)];
[cameraParams, ~, ~] = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);

intrinsics = cameraParams.Intrinsics;
fprintf('평균 reprojection error = %.4f pixels\n', cameraParams.MeanReprojectionError);

%% ==========================================================================
%  Phase 2: 외부 파라미터 (Extrinsics) 캘리브레이션
%% ==========================================================================
fprintf('\n===== Phase 2: 외부 파라미터 캘리브레이션 =====\n\n');
extrinsicImages = imageDatastore(extrinsicDir);
pitches = []; yaws = []; rolls = []; heights = [];

for i = 1:numel(extrinsicImages.Files)
    I = readimage(extrinsicImages, i);
    [imgPts, bSize] = detectCheckerboardPoints(I);
    if isempty(imgPts), continue; end
    wPts = patternWorldPoints('checkerboard', bSize, squareSize);
    
    [p, y, r, h] = estimateMonoCameraParameters(intrinsics, imgPts, wPts, 0);
    pitches(end+1) = p; yaws(end+1) = y; rolls(end+1) = r; heights(end+1) = h;
end

pitch  = mean(pitches); yaw = mean(yaws); roll = mean(rolls); height = mean(heights);
fprintf('Pitch = %.4f deg, Height = %.4f m\n', pitch, height);

%% ==========================================================================
%  Phase 3: BEV 검증 (ROS2 BevWarpNode 설정과 동일하게 세팅)
%% ==========================================================================
fprintf('\n===== Phase 3: BEV 검증 =====\n\n');
sensor = monoCamera(intrinsics, height, 'Pitch', pitch, 'Yaw', yaw, 'Roll', roll);

% ROS2 노드 설정값 반영: x(0.12~2.0), y(-0.85~0.85)
outView = [0.12, 2.00, -0.85, 0.85];  
% ROS2 노드 설정값 반영: 480x640 (MATLAB은 [height, width] 순서이므로 640x480)
outImageSize = [640, 480];            

birdsEye = birdsEyeView(sensor, outView, outImageSize);
I = readimage(extrinsicImages, 1);
undist = undistortImage(I, intrinsics);
BEV = transformImage(birdsEye, undist);

figure('Name', 'BEV Verification');
subplot(1,2,1); imshow(undist); title('Undistorted');
subplot(1,2,2); imshow(BEV);   title('C++ 노드와 동일한 BEV');

%% ==========================================================================
%  Phase 4: ROS2 노드 적용용 결과 출력
%% ==========================================================================
fprintf('\n======================================================\n');
fprintf('  [1] UndistortNode를 위해 hw40_params.yaml 에 복사할 내용\n');
fprintf('======================================================\n');
fprintf('camera:\n');
fprintf('  frame_id: camera_link\n');
fprintf('  width: %d\n', imageSize(2));
fprintf('  height: %d\n', imageSize(1));
fprintf('intrinsics:\n');
fprintf('  fx: %.5f\n', intrinsics.FocalLength(1));
fprintf('  fy: %.5f\n', intrinsics.FocalLength(2));
fprintf('  cx: %.5f\n', intrinsics.PrincipalPoint(1));
fprintf('  cy: %.5f\n', intrinsics.PrincipalPoint(2));
fprintf('  k1: %.6f\n', intrinsics.RadialDistortion(1));
fprintf('  k2: %.6f\n', intrinsics.RadialDistortion(2));
fprintf('  p1: %.6f\n', intrinsics.TangentialDistortion(1));
fprintf('  p2: %.6f\n', intrinsics.TangentialDistortion(2));
if numel(intrinsics.RadialDistortion) >= 3
    fprintf('  k3: %.6f\n', intrinsics.RadialDistortion(3));
else
    fprintf('  k3: 0.000000\n');
end

fprintf('\n======================================================\n');
fprintf('  [2] BevWarpNode.cpp 14~15번째 줄에 하드코딩 교체할 내용\n');
fprintf('======================================================\n');
fprintf('double fx = %.5f, fy = %.5f, cx = %.5f, cy = %.5f;\n', ...
    intrinsics.FocalLength(1), intrinsics.FocalLength(2), ...
    intrinsics.PrincipalPoint(1), intrinsics.PrincipalPoint(2));
fprintf('double h_m = %.4f, pitch_deg = %.4f;\n', height, pitch);
fprintf('======================================================\n');
