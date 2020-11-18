# Urquhart Tesselations for 2D data association

### Overview

Our method defines a set of polygons based on what we call "Urquhart Tessellations".
In summary, these polygons are used to define descriptors for sub-regions of a set of observed landmarks (observation).
This enables us to detect loops and compute landmark associations in 2D using only their position. [See the paper for more details](https://arxiv.org/abs/2010.03026).

### Dependencies
The code was developed and tested using the following packages and versions
```
QHull 2020.2 (Delaunay Triangulation)
Boost 1.71.0 (Tree structure that stores the polygons)
OpenCV 4.2.0 (Discrete fourier transform)
```

`test.cpp` gives a simple example on how to use this package.

### ROS
An example ROS node will be provided soon.


### Citing
If you use this code, please cite it

```
@inproceedings{nardari2020,
title={Place Recognition in Forest with Urquhart Tessellations},
author={Nardari, Guilherme V. and Cohen, Avraham and Chen, Steven W.
and Liu, Xu and Arcot, Vaibhav and Romero, Roseli A. F. and Kumar, Vijay},
booktitle={IEEE Robotics and Automation Letters (RA-L)},
year={2020}
}
```

##### This is an adaptation of the original Python code. We decided to reimplement it in C++ to provide a faster and more organized option for benchmarking. If you spot any bugs please create an issue or pull-request.
