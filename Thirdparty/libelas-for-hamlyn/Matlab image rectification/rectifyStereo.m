% This script rectifies stereo videos. Made by David Recasens
% Input: left and right stereo .avi videos, and calibration parameters
% Output: rectified left and right stereo .png frames. 

clear variables; close all;

%% Select the left and right video and the stereo parameters files
%The structure of the data has to be the following:
%   The name of the left stereo video has to end with _L
%   The name of the left stereo video has to end with _R
%   The folder with the stereo parameters has to be located in the same
%       folder as the videos and its name must me the same as the videos but
%       without the _L or _R
%   The .txt files with the calibration parameters of the left and right camera
%       must have the same name as the folder but with _L or _R in the end
%   The .txt file with the transformation matrix must have the same name as
%       the folder but with _stereo in the end
%   Look at inside of each .txt file to see their internal structure
%   Watch out! This script is made for a video with a number under 100

% Manually select the left stereo video
[videoLeftName, pathVideo] = uigetfile('*.avi', 'Select left stereo video');
if isequal(videoLeftName, 0)
   disp('User selected Cancel');
   return
else
   videoLeft =  fullfile(pathVideo, videoLeftName);
   disp(['User selected ', videoLeft]);
end

% Automatically select the right stereo video and the stereo calibration parameters. 
videoRightName = strcat(videoLeftName(1:end-5), 'R.avi'); % Remove the L.avi from the original name
videoRight = fullfile(pathVideo, videoRightName);
camParamL = fullfile(pathVideo, videoLeftName(1:end-6), strcat(videoLeftName(1:end-4), '.txt'));
camParamR = fullfile(pathVideo, videoLeftName(1:end-6), strcat(videoRightName(1:end-4), '.txt'));
extrinsicParam = fullfile(pathVideo, videoLeftName(1:end-6), strcat(videoLeftName(1:end-5), 'stereo', '.txt'));

%% Load the selected data
objL = VideoReader(videoLeft);
vidL = read(objL);
objR = VideoReader(videoRight);
vidR = read(objR);
frames = objL.NumFrames;
[rows, columns, numberOfColorChannels] = size(vidL(:,:,:,1));
imageSize = [rows, columns];

% Left camera parameters
camParamL_ID = fopen(camParamL);
intrinsicMatrixL(1,:) = sscanf(fgetl(camParamL_ID), '%f'); % fgetl gives the first line of the .txt as a char. sscanf transforms the char into a vector of doubles
intrinsicMatrixL(2,:) = sscanf(fgetl(camParamL_ID), '%f');
intrinsicMatrixL(3,:) = sscanf(fgetl(camParamL_ID), '%f');
intrinsicMatrixL = intrinsicMatrixL'; % The intrinsic matrix in the .txt is transposed regarding the matlab preferencies
line = sscanf(fgetl(camParamL_ID), '%f');
[radialDistortion1L, radialDistortion2L, tangentialDistortion1L, tangentialDistortion2L] = deal(line(1), line(2), line(3), line(4));
fclose(camParamL_ID);
radialDistortionL = [radialDistortion1L, radialDistortion2L];
tangentialDistortionL = [tangentialDistortion1L, tangentialDistortion2L];

% Right camera parameters
camParamR_ID = fopen(camParamR);
intrinsicMatrixR(1,:) = sscanf(fgetl(camParamR_ID), '%f'); % fgetl gives the first line of the .txt as a char. sscanf transforms the char into a vector of doubles
intrinsicMatrixR(2,:) = sscanf(fgetl(camParamR_ID), '%f');
intrinsicMatrixR(3,:) = sscanf(fgetl(camParamR_ID), '%f');
intrinsicMatrixR = intrinsicMatrixR'; % The intrinsic matrix in the .txt is transposed regarding the matlab preferencies
line = sscanf(fgetl(camParamR_ID), '%f');
[radialDistortion1R, radialDistortion2R, tangentialDistortion1R, tangentialDistortion2R] = deal(line(1), line(2), line(3), line(4));
fclose(camParamR_ID);
radialDistortionR = [radialDistortion1R, radialDistortion2R];
tangentialDistortionR = [tangentialDistortion1R, tangentialDistortion2R];

% Extrinsic parameters of camera2 respect camera1
extrinsicParam_ID = fopen(extrinsicParam);
rotationOfCamera2(1,:) = sscanf(fgetl(extrinsicParam_ID), '%f'); % fgetl gives the first line of the .txt as a char. sscanf transforms the char into a vector of doubles
rotationOfCamera2(2,:) = sscanf(fgetl(extrinsicParam_ID), '%f');
rotationOfCamera2(3,:) = sscanf(fgetl(extrinsicParam_ID), '%f');
rotationOfCamera2 = rotationOfCamera2'; % The rotation matrix in the .txt is transposed
translationOfCamera2 = sscanf(fgetl(extrinsicParam_ID), '%f');
fclose(extrinsicParam_ID);

%% Create a new folder with two folders inside to save the rectified images
newFolderName = strcat('rectified', videoLeftName(end-7:end-6));
newFolder = fullfile(pathVideo, newFolderName);
if ~exist(newFolder , 'dir')
    % Folder does not exist so create it.
    mkdir(newFolder);
else
    disp(['Error creating the new folder to save the frames. The folder already exists.']);
    return
end       

newSubFolderNameL = strcat('rectified', videoLeftName(end-7:end-6), '_L');
newSubFolderL = fullfile(pathVideo, newFolderName, newSubFolderNameL);
if ~exist(newSubFolderL , 'dir')
    % Folder does not exist so create it.
    mkdir(newSubFolderL);
else
    disp(['Error creating the new left sub folder to save the frames. The folder already exists.']);
    return
end       

newSubFolderNameR = strcat('rectified', videoLeftName(end-7:end-6), '_R');
newSubFolderR = fullfile(pathVideo, newFolderName, newSubFolderNameR);
if ~exist(newSubFolderR , 'dir')
    % Folder does not exist so create it.
    mkdir(newSubFolderR);
else
    disp(['Error creating the new right sub folder to save the frames. The folder already exists.']);
    return
end       

%% Rectify the images and save them in the respective folder
cameraParameters1 = cameraParameters('IntrinsicMatrix', intrinsicMatrixL, 'RadialDistortion', radialDistortionL, 'TangentialDistortion', tangentialDistortionL);
cameraParameters2 = cameraParameters('IntrinsicMatrix', intrinsicMatrixR, 'RadialDistortion', radialDistortionR, 'TangentialDistortion', tangentialDistortionR);
stereoParams = stereoParameters(cameraParameters1, cameraParameters2, rotationOfCamera2, translationOfCamera2);

for x = 1 : frames
    [J1, J2] = rectifyStereoImages(vidL(:,:,:,x), vidR(:,:,:,x), stereoParams, 'OutputView','full'); % With 'OutputView','full' the rectified images include all pixels from the original images
    imwrite(J1, fullfile(newSubFolderL, strcat('frameL-',num2str(x),'.png')) ); 
    imwrite(J2, fullfile(newSubFolderR, strcat('frameR-',num2str(x),'.png')) );
end