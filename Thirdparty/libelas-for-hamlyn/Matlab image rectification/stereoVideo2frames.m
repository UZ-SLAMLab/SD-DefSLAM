% This script extracts the frames from the stereo input video. This divides the video in two parts (left and right) of the same size, cropping exactly in the middle. Made by David Recasens
% Watch out! It is specifically made for .avi video

clear variables; close all;

[inputVideo,path] = uigetfile('*.avi');
if isequal(inputVideo,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,inputVideo)]);
end

% Get the frames from the input video
obj = VideoReader(fullfile(path,inputVideo));
vid = read(obj);
frames = obj.NumFrames;

% Create a new folder in the same folder as the video to save the frames,
% one for the left camera and another for the right one
for ii = 1 : 2
    
    % In the first iteration, the loop works with the left side of the video. After, with the right side
    if ii == 1
        side = 'L';
    else
        side = 'R';
    end
    
    newFolderName = strcat(inputVideo(1:end-4), '_', side, '-frames'); % Remove the .avi from the name
    newFolderDirectionAndName = fullfile(path, newFolderName);
    if ~exist(newFolderDirectionAndName , 'dir')
        % Folder does not exist so create it.
        mkdir(newFolderDirectionAndName);
    else
        disp(['Error creating the new folder to save the frames. The folder already exists.']);
    end       
    
    % Calculate the section of the image to crop
    [rows, columns, numberOfColorChannels] = size(vid(:,:,:,1));
    if ii == 1 % Left side        
        rect = [0 0 columns/2 rows ]; % [xmin ymin width height]
    else % Right side
        rect = [columns/2+1 0 columns/2 rows ];
    end
    
    % Crop the respective side of each frame and write it in the respective new folder
    for x = 1 : frames
        J = imcrop(vid(:,:,:,x), rect);
        imwrite(J, fullfile(newFolderDirectionAndName, strcat('frame-',num2str(x),'.png')) ); 
    end

end
