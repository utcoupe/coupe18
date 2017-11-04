#!/bin/bash

roslaunch rosbridge_server rosbridge_websocket.launch &
node_modules/http-server/bin/http-server -o &
