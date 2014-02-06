# RRT

[![Build Status](https://travis-ci.org/RoboJackets/rrt.png?branch=master)](https://travis-ci.org/RoboJackets/rrt)

C++ RRT (Rapidly-exploring Random Tree) implementation


Here's a gif of an RRT from wikipedia:
![RRT Animation from Wikipedia](http://upload.wikimedia.org/wikipedia/commons/6/62/Rapidly-exploring_Random_Tree_%28RRT%29_500x373.gif)


Interactive RRT Viewer
----------------------

This project contains an interactive RRT viewer.  The source and destination points can be dragged with a mouse.  Here's a screenshot:

![Interactive RRT](doc/images/rrt-viewer-screenshot.png)


Status
------

This isn't fully implemented yet, but is underway.  Feel free to contribute!, but realize that it's not ready for use.


Testing
-------

There are google-test unit tests setup in the test/ directory.  Run `make test` from the main directory to compile and run them.


## Resources

Here are some good resources for learning more about RRTs:

* http://msl.cs.uiuc.edu/rrt/
* [Wikipedia](http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree)
