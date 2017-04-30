# RRT [![CircleCI](https://circleci.com/gh/RoboJackets/rrt.svg?style=svg)](https://circleci.com/gh/RoboJackets/rrt)

C++ RRT (Rapidly-exploring Random Tree) implementation


## Interactive RRT Viewer

This project contains an interactive RRT viewer.  The source and destination points can be dragged with a mouse.  Here's a screenshot:

![Interactive RRT](doc/images/rrt-viewer-screenshot.png)


## Dependencies

The following are required in order to build this project:

* cmake >= 3.2.0
* Qt 5.5+
    - (The following dependencies are only needed for the rrt viewer)
    - QtDeclarative 5
    - QtQuick2.5+
    - QtQuick Controls
    - QtQuick Dialogs
* Eigen
* ninja
* ccache

To install all needed dependencies on Ubuntu 16.04, run:

```{.sh}
sudo apt-get -y install qt5-default libeigen3-dev g++ ninja-build cmake clang-format-3.6 ccache qtdeclarative5-dev qtdeclarative5-qtquick2-plugin qml-module-qtquick-{controls,dialogs}
```


## Building

Run `make` in the main directory to build the `rrt-viewer` program and the `test-runner` program, which are placed in the `build/` folder.

~~~{.sh}
git submodule update --init
make
build/rrt-viewer
~~~


## Resources

Here are some good resources for learning more about RRTs:

* http://msl.cs.uiuc.edu/rrt/
* [Wikipedia](http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree)
* http://www.cs.cmu.edu/~15780/readings/02iros-errt.pdf


## License

This project is licensed under the Apache License v2.0.  See the [LICENSE](LICENSE) file for more information.
