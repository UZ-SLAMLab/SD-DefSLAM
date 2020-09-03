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

    local video="$local_dir/stereo.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"
    local output_folder="$local_dir/output"

    echo "Processing Video : " $video
    echo $video $leftCalibration $rightCalibration $extrinsicCalibration $output_folder
    ./build/rectifier $video $leftCalibration $rightCalibration $extrinsicCalibration $output_folder
    return 0
); }

main $@

