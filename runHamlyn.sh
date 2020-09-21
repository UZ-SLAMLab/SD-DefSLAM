#!/bin/bash
# /media/jose/NuevoVol/Dataset/hamlyn.doc.ic.ac.uk/vision/data

vocabulary_file="/home/jose/DefKLTSLAM/Vocabulary/ORBvoc.txt"

function runstd {
    local local_dir=${1:-null}
    local video="$local_dir/*.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"
    echo "Processing Video with std : " $video
    echo $video $leftCalibration $rightCalibration $extrinsicCalibration
    local args="$video $leftCalibration $rightCalibration $extrinsicCalibration"
    cp /home/jose/DefKLTSLAM/Apps/DefSLAMHamyln tmp
    ./tmp $vocabulary_file $video $leftCalibration $rightCalibration $extrinsicCalibration  >> log.txt
    rm tmp
    return 0
}  

function runstdinv {
    local local_dir=${1:-null}
    local video="$local_dir/*.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"

    echo "Processing Video with stdinv : " $video
    echo $video $leftCalibration $rightCalibration $extrinsicCalibration
    local args="$video $leftCalibration $rightCalibration $extrinsicCalibration"
    cp /home/jose/DefKLTSLAM/Apps/DefSLAMHamylnINV tmp
    ./tmp $vocabulary_file $video $leftCalibration $rightCalibration $extrinsicCalibration  >> log.txt
    rm tmp
    return 0
}  

function runstd2 {
    local local_dir=${1:-null}
    #local videoleft="$local_dir/f7_dynamic_deint_L.avi"
    #local videoright="$local_dir/f7_dynamic_deint_R.avi"
    local videoleft="$local_dir/left.avi"
    local videoright="$local_dir/right.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"
    echo "Processing Videos : " $videoleft
    local args="$vocabulary_file $videoleft $videoright $leftCalibration $rightCalibration  $extrinsicCalibration"
    echo $args
    cp /home/jose/DefKLTSLAM/Apps/DefSLAMHamyln tmp
    ./tmp $args >> log.txt
    rm tmp
    echo $1 
}  

function runstd2inv {
    local local_dir=${1:-null}
    #local videoleft="$local_dir/f7_dynamic_deint_L.avi"
    #local videoright="$local_dir/f7_dynamic_deint_R.avi"
    local videoleft="$local_dir/left.avi"
    local videoright="$local_dir/right.avi"
    local leftCalibration="$local_dir/Left_Camera_Calibration_Intrinsic.txt"
    local rightCalibration="$local_dir/Right_Camera_Calibration_Intrinsic.txt"
    local extrinsicCalibration="$local_dir/camera_extrinsic.txt"
    echo "Processing Videos : " $videoleft
    local args="$vocabulary_file $videoleft $videoright $leftCalibration $rightCalibration  $extrinsicCalibration"
    echo $args
    cp /home/jose/DefKLTSLAM/Apps/DefSLAMHamylnINV tmp
    ./tmp $args >> log.txt
    rm tmp
    echo $1 
}  

function countavi {
    local directory=${1:-null}
    local arg="$directory/*.avi"
    NUMBEROF=$(ls -l $arg | grep -v ^l | wc -l)
    echo ${NUMBEROF}
}

function run {
    echo "I receive" $1 $2
if [[ "$2" =~ ^('Dataset1'|'Dataset6'|'Dataset7'|'Dataset8'|'Dataset9')$ ]]; then
    echo "$2 is in the list"
rm -r $2
mkdir $2
cd $2
for i in 1 2 3 4 5
do
mkdir $i
cd $i
runstdinv "$1/$2"
cd ..
done
cd ..
else
rm -r $2
mkdir $2
cd $2
for i in 1 2 3 4 5
do
mkdir $i
cd $i
runstd "$1/$2"
cd ..
done
cd ..
fi
}

function run2 {
    echo "I receive" $1 $2
if [[ "$2" =~ ^('Dataset1'|'Dataset6'|'Dataset7'|'Dataset8'|'Dataset9')$ ]]; then
rm -r $2
mkdir $2
cd $2
for i in 1 2 3 4 5
do
mkdir $i
cd $i
runstd2inv "$1/$2"
cd ..
done
cd ..
else
rm -r $2
mkdir $2
cd $2
for i in 1 2 3 4 5
do
mkdir $i
cd $i
runstd2 "$1/$2"
cd ..
done
cd ..
fi
}

##########################################################
# Process the Mandala dataset
#############################
main() { (
folders=$(ls $1)
for VARIABLE in $folders
do
NUMBEROF=$(countavi $1/$VARIABLE)
if [ $NUMBEROF -eq 1 ]
then
echo "FOR $VARIABLE THE NUMBER IS $NUMBEROF"
run $1 $VARIABLE 
else
if [ $NUMBEROF -eq 2 ]
then
echo "FOR $VARIABLE THE NUMBER IS $NUMBEROF"
run2 $1 $VARIABLE
fi
fi
done
)}

main $@
