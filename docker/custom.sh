#!/bin/bash

# Download and install Rust using rustup
curl https://sh.rustup.rs -sSf | sh -s -- -y

# Install the repo as pip package
pip install /docker-ros/ws/src/target