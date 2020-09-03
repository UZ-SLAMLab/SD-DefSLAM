#!/usr/bin/env bash
#
##################
# Include section.
##################


##########################################################
# Process Hamlyn dataset
# Globals:
#   None
# Arguments:
#   Local directory to download to.
# Returns:
#   0 if thing was deleted, non-zero on error.
##########################################################

main() { (
    local local_dir=${1:-null}

    local folder_images_left="$local_dir/camera0"
    local folder_images_right="$local_dir/camera1"
    local pattern_image_right="stereo_im_l_"
    local pattern_image_left="stereo_im_r_"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"
    local output_folder="$local_dir/output"

    echo "Searching left images in : " $folder_images_left " with pattern " $pattern_image_right
    echo "Searching right images in : " $folder_images_right " with pattern " $pattern_image_left
    local args="$folder_images_left $folder_images_right $pattern_image_right $pattern_image_left $leftCalibration $rightCalibration $extrinsicCalibration $output_folder"
    echo $args
    echo $folder_images_left $folder_images_right $pattern_image_right $pattern_image_left $leftCalibration $rightCalibration $extrinsicCalibration $output_folder
    ./build/rectifier $args
    return 0
); }

main $@

