# Use phusion/baseimage if problems arise
FROM ubuntu:18.04
MAINTAINER Jay Kamat jaygkamat@gmail.com

# Setup apt to be happy with no console input
ENV DEBIAN_FRONTEND noninteractive

# setup apt tools and other goodies we want
RUN apt-get update --fix-missing && apt-get -y install udev locales git software-properties-common sudo tzdata && apt-get clean

# Prevent bugging us later about timezones
RUN ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime && dpkg-reconfigure --frontend noninteractive tzdata

# Use UTF-8
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# set up user <this is for running soccer later on>
# Replace 1000 with your user / group id
# RUN export uid=1000 gid=1000 && \
#     mkdir -p /home/developer && \
#     echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
#     echo "developer:x:${uid}:" >> /etc/group && \
#     echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
#     chmod 0440 /etc/sudoers.d/developer && \
#     chown ${uid}:${gid} -R /home/developer && mkdir -p /etc/udev/rules.d/

# USER developer
# ENV HOME /home/developer

# # do everything in developers's home
# RUN mkdir -p /home/developer

COPY . ~/rrt
WORKDIR ~/rrt

RUN sudo ./ci/ubuntu-setup --yes
