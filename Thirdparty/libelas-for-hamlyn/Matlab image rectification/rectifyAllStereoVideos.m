% This script automatically rectifies all the stereo videos inside the
% selected folder. Made by David Recasens
% Input: one left stereo video in the folder. Left and right stereo videos with the names:
%   calibratedXX_L.avi and calibratedXX_R.avi
% Output: a new folder with two folders saving each one the left and right
%   rectified stereo frames for each stereo pair video

% Watch out! This script is made for less than 100 videos

clear variables; close all;

% Manually select the last left stereo video in the folder
pathVideo = uigetdir('Select parent folder with all the stereo videos and parameters inside');
if isequal(pathVideo, 0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', pathVideo]);
end

% Automatically select all the left stereo videos and rectify all the
% left and right videos in the selected folder
filePattern = fullfile(pathVideo, '*calibrated*.avi');
files = dir(filePattern);
for k = 1 : length(files)          
    if ~isempty(strfind(files(k).name, '_L')) % True if the file name has a _L
        videoLeftName = files(k).name;
        videoLeft = fullfile(pathVideo, videoLeftName);
        try            
            rectifyStereoFunction;
        catch
            warning('Probably the video ' + convertCharsToStrings(videoLeftName) + ' has not the proper name style: calibratedXX_L, or there are not proper calibration parameters for the video. If not, the problem is in the script rectifyStereoFunction');
            continue
        end
    end
end


