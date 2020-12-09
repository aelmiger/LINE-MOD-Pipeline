
<!-- PROJECT LOGO -->
<br />
<p align="center">

  <h1 align="center">LINE-MOD-Pipeline for object detection and pose estimation</h3>

  <p align="center">
    Real-time 6DOF pose estimation for an industrial bin picking scenario
    <br />
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#method">Method</a></li>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project
<p float="left">
<img src="https://i.ibb.co/1npYc5g/korrekt-Double-Pose.png" alt="korrekt-Double-Pose" border="0" width="300">
<img src="https://i.ibb.co/QpjDh10/correct-Pose-Estimation2.png" alt="correct-Pose-Estimation2" border="0" width="300">
<img src="https://i.ibb.co/D1TSMHd/correct-Pose-Estimation.png" alt="correct-Pose-Estimation" border="0" width="300">
</p>

The pipeline implements the OpenCV LINE-MOD Detector to estimate the pose of a known object. Postprocessing steps for detection of shiny objects with corrupted depth information are added. The goal of this pipeline is to locate and detect the pose of industrial objects like bearings to aide a robotic arm during assembly (bin picking). The pipeline implements the creation of templates from CAD objects, which makes it possible to detect new objects in a matter of minutes.

The accompanying bachelor thesis can be read here (german): [Bachelor Thesis (german)](bachelor_thesis.pdf)

Necessary Pre- and Postprocessing tools that are implemented:
* Generation of Templates from CAD-Models with OpenGL
* Color and depth checks to validate a match
* Steps to correct for corrupted depth information on reflective objects
* ICP algorithm for pose refinement

Included libraries for testing and validation are:
* Kinect V2 tool to extract RGB-D images
* Implementation of the LINE-MOD Benchmark and the Benchmark for 6D Object Pose Estimation evaluation methods
* A class to generate RGB-D Images with a ground truth pose of an Aruco marker

### Built With

For compilation the following libraries must be included and linked
* [SDL2](https://www.libsdl.org/download-2.0.php) - Creates a window for OpenGL
* [GLEW](http://glew.sourceforge.net/) - OpenGL Extension Library
* [OPENCV4](https://opencv.org/) - Computer vision library
* [GLM](https://glm.g-truc.net/0.9.9/index.html) - Graphics math library
* [libfreenect2](https://github.com/OpenKinect/libfreenect2) - Kinect V2 driver (only if Kinect is used)
* [assimp](http://www.assimp.org/) - Open Asset Importer library

<!-- GETTING STARTED -->
## Getting Started

### Dependencies
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

### Installation

#### Compiling the pipeline
In a new folder start by cloning the repo.
```bash
git clone https://github.com/aelmiger/LINE-MOD-Pipeline.git LINE-MOD
cd LINE-MOD
```
Next step is to run cmake and compile the binaries.

For cmake to properly locate the SDL2 libraries it needs a FindSDL2.cmake module which is not yet part of cmake. 

[FindSDL2](https://github.com/aminosbh/sdl2-cmake-modules) can be downloaded here and has to be placed in the cmake modules folder.

```bash
cmake -H. -B build
cmake --build build --config Release --target all -- -j4
```

<!-- USAGE EXAMPLES -->
## Usage

To test the applications run the template generator first.
```bash
./build/Template_Generator
```
A window should open and display the rendered object in white under different poses and save the template generation files.

Now run the detector. An example image in the benchmark folder will be used.
```bash
./build/Detector
``` 
<!-- LICENSE -->
## License

This project is licensed under the BSD License - see the [LICENSE.md](LICENSE.md) file for details

<!-- CONTACT -->
## Contact

Anton Elmiger - [anton.elmiger@gmail.com](mailto:anton.elmiger@gmail.com) - email
