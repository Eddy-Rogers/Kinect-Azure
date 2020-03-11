#!/bin/bash

# Script to run two Kinect cameras (The Master and one Subordinate) at the same time.

DEST_PATH="Desktop/Kinect Azure/Azure Kinect SDK v1.3.0/tools/"

USER_IN="$1"

SUB2_COMMAND="./k4arecorder.exe --device 1 --external-sync Subordinate -d NFOV_UNBINNED "

SUB3_COMMAND="./k4arecorder.exe --device 0 --external-sync Subordinate -d WFOV_2X2BINNED "

SUB2_TAG="_Sub2.mkv"

SUB3_TAG="_Sub3.mkv"

cd "$DEST_PATH"

$SUB2_COMMAND$USER_IN$SUB2_TAG

echo $SUB2_COMMAND$USER_IN$SUB2_TAG

$SUB3_COMMAND$USER_IN$SUB3_TAG

echo $SUB3_COMMAND$USER_IN$SUB3_TAG

exit $?
