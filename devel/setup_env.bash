#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

declare GRID_MODEL_PATH="$HOME/grid_ws/src/bot_description/models"
declare GRID_WORLD_PATH="$HOME/grid_ws/src/bot_description/worlds"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${GRID_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${GRID_WORLD_PATH}