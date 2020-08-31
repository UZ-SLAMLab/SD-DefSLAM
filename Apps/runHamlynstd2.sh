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


    local calibration_file="/home/jose/DefKLTSLAM/Vocabulary/ORBvoc.txt"
    #local videoleft="$local_dir/f7_dynamic_deint_L.avi"
    #local videoright="$local_dir/f7_dynamic_deint_R.avi"
    local videoleft="$local_dir/left.avi"
    local videoright="$local_dir/right.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"

    echo "Processing Videos : " $videoleft
    local args2="$calibration_file $videoleft $videoright $leftCalibration $rightCalibration  $extrinsicCalibration"
    echo $args2
    ./DefSLAMHamyln $args2
    return 0
); }

main $@

