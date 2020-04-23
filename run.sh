#!/bin/bash

conda activate cadrl

killall -9 roscore
killall -9 rosmaster

{
PIDS[1]=$!;
roslaunch menge_gazebo_worlds turtlebot.launch > turtlebot.log
}&

sleep 10

{ 
PIDS[2]=$!;
roslaunch menge_gazebo_worlds start_bot.launch
}&

sleep 20

{ 
PIDS[3]=$!;
python ppo_gazebo.py > ppo_gazebo.log
}&

trap "kill ${PIDS[*]}; killall -9 rosmaster; killall -9 roscore" SIGINT
wait
