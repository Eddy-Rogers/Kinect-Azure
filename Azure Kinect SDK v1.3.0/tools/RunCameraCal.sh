#Script to run two Kinect cameras (The Master and one Subordinate) at the same time.

DEST_PATH="Desktop/Kinect Azure/Azure Kinect SDK v1.3.0/tools/"

CAL_TAG="Cal"

SUB1_COMMAND="./k4arecorder.exe --device 1 --external-sync Subordinate -d WFOV_UNBINNED "

MASTER_COMMAND="./k4arecorder.exe --device 0 --external-sync Master -d WFOV_UNBINNED "

cd "$DEST_PATH"

$SUB1_COMMAND$CAL_TAG$SUB2_TAG

echo $SUB2_COMMAND$CAL_TAG$SUB2_TAG

$SUB3_COMMAND$CAL_TAG$SUB3_TAG

echo $SUB3_COMMAND$CAL_TAG$SUB3_TAG

exit $?

