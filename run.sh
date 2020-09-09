#!/bin/bash
#

##########################################################
# Process the Mandala dataset
#############################
main() { (
for VARIABLE in Mandala0 Mandala1 Mandala2 Mandala3 Mandala4 MandalaC
do
mkdir $VARIABLE
cd $VARIABLE
local ARG="/home/jose/DefKLTSLAM/Vocabulary/ORBvoc.txt /home/jose/DeformableSLAM/calibration_files/stereo0.yaml /media/jose/NuevoVol/videosDataset/Jose/$VARIABLE/images /media/jose/NuevoVol/videosDataset/Jose/$VARIABLE/images /media/jose/NuevoVol/videosDataset/Jose/$VARIABLE/timestamps/timestamps.txt"
for i in 1 2 3 4 5
do
mkdir $i
cd $i
cp /home/jose/DefKLTSLAM/Apps/DefSLAMGT tmp
./tmp $ARG
rm tmp
cd ..
done
cd ..
done
)}

main $@
