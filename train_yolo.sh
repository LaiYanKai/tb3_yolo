#!/bin/bash
cd $HOME/tb3_yolo/train_yolo

# remove the cache
rm -rf datasets/traffic_signs/labels.cache # delete cache to refresh the labels.
rm -rf $HOME/.config/Ultralytics/settings.yaml # delete the global yaml in case the path is unexpectedly wrong in the code.

# run the training
python3 train_yolo.py
