#!/bin/bash
#
# Script to setup build environment on ubuntu
# note: this must be run from the root directory of the project

git submodule update --init

sudo apt-get -y install cmake \
    qt5-default \
    libeigen3-dev \
    g++
