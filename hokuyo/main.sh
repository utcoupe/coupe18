#!/bin/sh
# echo "Launching Hokuyo Node.JS app..."
sudo LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH node $UTCOUPE_WORKSPACE/hokuyo/main.js &
