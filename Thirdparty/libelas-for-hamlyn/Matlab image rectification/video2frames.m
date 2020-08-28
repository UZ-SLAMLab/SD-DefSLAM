% This script extracts the frames from the monocular input video. Made by David Recasens
% Watch out! It is specifically made for .avi video

clear variables; close all;

[inputVideo,path] = uigetfile('*.avi');
if isequal(inputVideo,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,inputVideo)]);
end

% Create a new folder in the same folder as the video to save the frames
newFolderName = strcat(inputVideo(1:end-4), '-frames'); % Remove the .avi from the name
newFolderDirectionAndName = fullfile(path, newFolderName);
if ~exist(newFolderDirectionAndName , 'dir')
    % Folder does not exist so create it.
    mkdir(newFolderDirectionAndName);
else
    disp(['Error creating the new folder to save the frames. The folder already exists.']);
end

obj = VideoReader(fullfile(path,inputVideo));
vid = read(obj);
frames = obj.NumFrames;
for x = 1 : frames
    imwrite(vid(:,:,:,x), fullfile(newFolderDirectionAndName, strcat('frame-',num2str(x),'.png')) ); 
end