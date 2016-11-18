# Use phusion/baseimage if problems arise
FROM jgkamat/locale:16.04
MAINTAINER Jay Kamat jaygkamat@gmail.com

# This dockerimage will serve as a 'static' base for the robocup dockerimage to reduce build time

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# Use UTF-8
# RUN locale-gen en_US.UTF-8 ## TODO UNCOMMENT WHEN LOCALES ARE FIXED IN CIRCLECI ##
ENV LANG en_US.UTF-8

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install apt-utils wget curl htop iputils-ping vim-nox less debconf-utils w3m git software-properties-common sudo scons screen && apt-get clean

# set up user <this is for running soccer later on>
# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer && mkdir -p /etc/udev/rules.d/

USER developer
ENV HOME /home/developer

# Set max cache size
RUN ccache -M 2G

# do everything in root's home
RUN mkdir -p /home/developer/project
WORKDIR /home/developer/project

# This image is not meant to be run directly, it has not been compiled yet!
# In addition, it does not contain any source code, only dependencies
