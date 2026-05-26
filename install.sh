#!/bin/sh
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y python3-pip ros-jazzy-nlohmann-json-schema-validator-vendor ros-jazzy-tinyxml2_vendor
pip install --user --break-system-packages vcstool
vcs import --recursive --input dependencies.repos ../
