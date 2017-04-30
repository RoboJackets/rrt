#!/bin/bash
#
# Script to setup build environment on ubuntu
# note: this must be run from the root directory of the project

git submodule update --init

sudo apt-get -y update
# sudo apt-get -y upgrade

sudo apt-get -y install \
    qt5-default \
    qtdeclarative5-dev qtdeclarative5-qtquick2-plugin \
    qml-module-qtquick-{controls,dialogs} \
    libeigen3-dev \
    g++ \
    ninja-build \
    cmake \
    clang-format-3.6 \
    python3 \
    python3-pip \
    ccache \
    libflann-dev

# code formatting tool
sudo pip3 install stylize>=0.2.7
