#!/bin/bash
#
# Script to setup build environment on ubuntu
# note: this must be run from the root directory of the project

git submodule update --init

# add repo for updated version of cmake
sudo apt-add-repository -y ppa:george-edison55/cmake-3.x

sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install \
    qt5-default \
    libeigen3-dev \
    g++ \
    ninja-build \
    cmake \
    clang-format-3.6 \
    python3 \
    python3-pip \

# code formatting tool
sudo pip3 install stylize>=0.2.7
