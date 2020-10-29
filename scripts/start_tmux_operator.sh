#!/bin/bash

tmux new -d -s project11
tmux send-keys "roscore" C-m
tmux splitw -p 90
tmux send-keys "rosrun rosmon rosmon project11 operator_mystique_mobile_lab.launch" C-m
tmux splitw -p 50
tmux send-keys "rosrun rosmon rosmon project11 operator_ui.launch" C-m
tmux a -t project11
