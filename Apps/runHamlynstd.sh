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

<<<<<<< HEAD
    local vocabulary_file="/home/jose/DefKLTSLAM/Vocabulary/ORBvoc.txt"
=======
    local calibration_file="/home/jose/DefKLTSLAM/Vocabulary/ORBvoc.txt"
>>>>>>> cebc210467716a6ff28fd0f431b7a32d099886a9
    local video="$local_dir/stereo.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"

    echo "Processing Video : " $video
    echo $video $leftCalibration $rightCalibration $extrinsicCalibration
    ./DefSLAMHamyln $vocabulary_file $video $leftCalibration $rightCalibration $extrinsicCalibration
    return 0
); }

main $@

