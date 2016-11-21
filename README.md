# RRT ![Circle CI](https://circleci.com/gh/RoboJackets/rrt.svg?style=svg)

C++ RRT (Rapidly-exploring Random Tree) implementation


## Interactive RRT Viewer

This project contains an interactive RRT viewer.  The source and destination points can be dragged with a mouse.  Here's a screenshot:

![Interactive RRT](doc/images/rrt-viewer-screenshot.png)


## Dependencies

The following are required in order to build this project:

* cmake >= 3.2.0
* Qt 5
* Eigen
* ninja
* ccache

To install all needed dependencies on Ubuntu 16.04, run:

```{.sh}
sudo apt-get -y install qt5-default libeigen3-dev g++ ninja-build cmake clang-format-3.6 ccache
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
