# Aruco Detection 

This package is used to detect Aruco markers in the environment. The package uses the `aruco` package to detect the markers. The package is used to detect the markers.

## Installation

1. Clone the repository into your catkin workspace
2. Build the package using catkin_make

## Usage

1. Set the path to video file in the `detection.launch` file and the marker type
```xml
<param name="marker_type" value="6x6" />
<param name="video_path" value="$(find aruco_detection)/videos/video1.mp4" />
```
2. Launch the detection.launch file
```bash
roslaunch aruco_detection detection.launch
```

## Tutorial

Video Link: [Aruco Detection](https://youtu.be/n_1AMCHPBYs)