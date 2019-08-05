# LINE-MOD-Pipeline for object detection and pose estimation

The pipeline uses the OpenCV LINE-MOD Detector to estimate the pose of a known Object. 
Necessary Pre and Postprocessing tools are included:
* Generation of Templates from CAD-Models with OpenGL
* Color and depth checks to validate a match
* ICP algorithm from OpenCV for pose refinement

Included libraries for testing are:
* Kinect V2 tool to extract depth and rgb images
* Implementation of the LINE-MOD Benchmark for validation
* A class to generate RGB-D Images with a ground truth pose of a Aruco marker

## Getting Started

TODO

### Prerequisites

For compilation the following libraries must be included and linked
* [SDL2](https://www.libsdl.org/download-2.0.php) - Creates a window for OpenGL
* [GLEW](http://glew.sourceforge.net/) - OpenGL Extension Library
* [OPENCV4](https://opencv.org/) - Computer vision library
* [GLM](https://glm.g-truc.net/0.9.9/index.html) - Graphics math library
* [libfreenect2](https://github.com/OpenKinect/libfreenect2) - Kinect V2 driver (only if Kinect is used)
* [assimp](http://www.assimp.org/) - Open Asset Importer library

Currently the Windows ppl.h library is used for parallelism. This will be changed to the platform independant OpenMP.

### Installing

#### Dependencies
For Linux the following command can be used to install most dependencies

```bash
sudo apt-get install libglew-dev libglm-dev libassimp-dev libsdl2-dev
```
freenect2 has to be compiled from source with the help of cmake.

OpenCV 4 needs to be built with:
* additional contrib libraries for LINE-MOD and ICP
* OpenMP

To compile OpenCV the following command can be used. The path of the contrib modules has to be replaced.
```bash
cmake -D OPENCV_EXTRA_MODULES_PATH=<contrib modules path> -D WITH_OPENMP=ON -D CMAKE_BUILD_TYPE=Release
```

## Authors

* **Anton Elmiger** - *Initial work* - [GitLab](https://gitlab.tubit.tu-berlin.de/antone)

## License

This project is licensed under the BSD License - see the [LICENSE.md](LICENSE.md) file for details
