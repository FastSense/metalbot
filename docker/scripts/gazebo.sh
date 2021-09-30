#!/bin/bash

sudo curl -sSL http://get.gazebosim.org | sh

sudo apt-get update && apt-get install -y --no-install-recommends \
     ros-foxy-gazebo-ros-pkgs && \
     rm -rf /var/lib/apt/lists/*

