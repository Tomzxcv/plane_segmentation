#!/bin/zsh

basic_path=$1
config_file=$2
cloud_file=$3

echo "basic_path = $basic_path"
echo "config_file = $config_file"
echo "cloud_file = $cloud_file"

#./build/cloud_cut $basic_path $config_file $cloud_file

#python3 app/lidar_cut.py --basic_path $basic_path --epsilon 0.04

./build/plane_segmentation_main $basic_path $config_file $cloud_file
