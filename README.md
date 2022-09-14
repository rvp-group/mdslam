<h1 align="center">MD-SLAM: Multi-cue Direct SLAM</h1>

<p align="center">
Versatile direct SLAM pipeline that works for RGB-D and LiDAR
</p>

<p align="center">
    <a href="https://digiamm.github.io">Luca Di Giammarino</a><sup>1</sup> &emsp;
    <a href="https://www.linkedin.com/in/leobrizi">Leonardo Brizi</a><sup>1</sup> &emsp;
    <a href="https://www.linkedin.com/in/tiziano-guadagnino-087119170/?originalSubdomain=it">Tiziano Guadagnino</a><sup>1</sup> &emsp;
    <a href="https://www.ipb.uni-bonn.de/people/cyrill-stachniss/">Cyrill Stachniss</a><sup>2</sup> &emsp;
    <a href="https://sites.google.com/dis.uniroma1.it/grisetti/home">Giorgio Grisetti</a><sup>1</sup> &emsp;
</p>

<p align="center">
    <sup>1</sup>Sapienza University of Rome&emsp;&emsp;
    <sup>2</sup>University of Bonn<br>
</p>

<p align="center">
    Accepted: International Conference on Intelligent Robots and Systems (IROS) 2022, Kyoto, Japan
</p>

<p align="center">
    <a href="https://arxiv.org/abs/2203.13237">arXiv</a>
</p>

<p align="center">
  <img src="assets/md_rgbd.gif" width="400" />
  <img src="assets/md_lidar.gif" width="400" /> 
</p>

<h2>Abstract</h2>
<p align="justify">Simultaneous Localization and Mapping (SLAM) systems are fundamental building blocks for any autonomous robot navigating in unknown environments. The SLAM implementation heavily depends on the sensor modality employed on the mobile platform. For this reason, assumptions on the scene's structure are often made to maximize estimation accuracy. This paper presents a novel direct 3D SLAM pipeline that works independently for RGB-D and LiDAR sensors. Building upon prior work on multi-cue photometric frame-to-frame alignment, our proposed approach provides an easy-to-extend and generic SLAM system. Our pipeline requires only minor adaptations within the projection model to handle different sensor modalities. We couple a position tracking system with an appearance-based relocalization mechanism that handles large loop closures. Loop closures are validated by the same direct registration algorithm used for odometry estimation. We present comparative experiments with state-of-the-art approaches on publicly available benchmarks using RGB-D cameras and 3D LiDARs. Our system performs well in heterogeneous datasets compared to other sensor-specific methods while making no assumptions about the environment. Finally, we release an open-source C++ implementation of our system.</p>

<h2>Data download</h2>

[Download](https://drive.google.com/drive/folders/1tq3R881MS7IyD-w7f3jbKUzlxDZzV1u7?usp=sharing) our pre-processed data. This trial data contains: [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset), [ETH3D](https://www.eth3d.net/slam_datasets) and some self-recorded one. All our data is in `rosbag` format. NOTE: more data will be uploaded in the next days.

<h2>Docker</h2>

Before you locally install anything, bear in mind that you can use our <b>[docker](https://github.com/digiamm/md_slam/blob/main/docker/README.md)</b>.

<h2>Installation</h2>

### [Install](http://wiki.ros.org/noetic/Installation/Ubuntu) ROS Noetic on Ubuntu 20.04

Once ROS is installed, run
``` 
sudo apt-get update 
```
Now install the required extra packages
``` 
sudo apt-get install libeigen3-dev libsuitesparse-dev libqglviewer-dev-qt5 freeglut3-dev libpcl-dev ros-noetic-grid-map-msgs python3-catkin-tools
```
Create a folder for the ROS workspace and go into it
```
mkdir -p /catkin_ws/src && cd /catkin_ws/src 
```
Clone this package and other dependencies on the `src` folder
```
cd ~/catkin_ws/src/
git clone https://github.com/digiamm/md_slam.git
git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git 
git clone https://gitlab.com/srrg-software/srrg_hbst.git 
git clone https://gitlab.com/srrg-software/srrg2_core.git && cd srrg2_core && git checkout c747aa854a2d1f7fdad6516474c4a4d3a543ea47 
git clone https://gitlab.com/srrg-software/srrg2_solver.git && cd srrg2_solver && git checkout eb34f226733532ab67d5e45e7de21b284599af89 
```

Checkout `srrg2_core` and `srrg2_solver` to tested version
```
git checkout ~/catkin_ws/src/srrg2_core c747aa854a2d1f7fdad6516474c4a4d3a543ea47
git checkout ~/catkin_ws/src/srrg2_solver eb34f226733532ab67d5e45e7de21b284599af89
```
Build package and dependencies using `catkin_tools`
```
cd ~/catkin_ws && catkin build md_slam 
```
Finally, source workspace
```
source ~/catkin_ws/devel/setup.bash
```

<h2 name=run>Run MD-SLAM</h2>

Run the pipeline
```
rosrun md_slam md_slam -i path/to/dataset -c path/to/configuration/file
```
Basic configuration files can be found in `configs/`

Other flags can be enabled when running MD-SLAM, such as enable viewer, save data at the end of the run, verbose, etc. The full list of any executables in the package can be see with `-h` flag. For `md_slam` this is the full list:
```
config file to load
-c (--config), default: []

if set enables viewer, otherwise just runs
-e (--enable-viewer), default: [not-set]

displays this help message
-h (--help), default: [not-set]

input bag file to start
-i (--input), default: []

output filename for graph and pyramid serialization
-o (--ouput), default: []

if set enables perspective view, i.e. gl camera follows sensor
-p (--perspective), default: [not-set]

if set enables cerr and cout streams
-v (--verbose), default: [not-set]
```

<h3>View data</h3>

If you run MD-SLAM with `-o` you can save the graph and the point clouds attached to it. If you want to see the output data is enough to run
```
rosrun md_slam show_graph -i path/to/output/file
```

<h3>Evaluate data</h3>

The file generated from the pipeline containing the graph, can be converted in [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats) groundtruth format
```
timestamp tx ty tz qx qy qz qw
``` 
by running the following
```
rosrun md_slam graph_converter -i path/to/graph/file -o path/to/tum/trajectory/file
```

<h2>Use your data</h2>

Our is a purely direct and symmetric pipeline that works independently for RGB-D and LiDAR (the only thing that changes is the projection). For this reason, for both the sensors, the rosbag must have a `camera matrix`, a `grayscale` (or intensity) and a `depth` (or range) images syncronized. Therefore the number of these messages needs to be the same. For instance, an output of `rosbag info` of your newly created rosbag needs to be like this:
```
topics:      /os/camera_info        1190 msgs    : sensor_msgs/CameraInfo
             /os/image_depth        1190 msgs    : sensor_msgs/Image     
             /os/image_intensity    1190 msgs    : sensor_msgs/Image
```
<h3>Camera matrix</h3>

<b>RGB-D</b> camera matrix contains `fx, fy, cx, cy`, focal lenghts and principal points are estimated after intrinsics calibration and usually come with the dataset. 
```
K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
```
<b>LiDAR</b> camera matrix similiarly is parameterized by azimuth and elevation resolution. These are calculated as `azimuth_resolution = (h_fov_max - h_fov_min)/img_cols` and `elevation_resolution = (v_fov_max - v_fov_min)/img_rows`.
```
K: [-1 / azimuth_resolution, 0, img_cols / 2, 0, -1 / elevation_resolution, img_rows / 2, 0, 0, 1]
```
For instance, for an OS0-128 with `v_fov_max = pi/4`, `v_fov_min = -pi/4` with `img_rows = 128` and having the complete encoder rotation of 360deg so `h_fov_max = 2pi`, `h_fov_min = 0` with `img_cols = 1024` we will have the following results on the camera matrix:
```
K: [-162.9746551513672, 0.0, 512.0, 0.0, -79.22404479980469, 64.0, 0.0, 0.0, 1.0]
```
<h3>Input cues - images</h3>

<b>RGB-D</b> data usually already comes with grayscale and depth images already "syncronized". For <b>LiDAR</b> one can generate both intensity and range images from the point cloud by just using a spherical projection (look at the [paper](https://arxiv.org/abs/2203.13237) for more info).  
<h2> Process your data with our utilities </h2>   

<h3>RGB-D</h3>

For RGB-D we provide the executable to convert [ETH3D](https://www.eth3d.net/slam_datasets) (ex TUM format - [more details](https://vision.in.tum.de/data/datasets/rgbd-dataset)) into a rosbag processable by our pipepline. Once you have sourced the workspace
```
source ~/catkin_ws/devel/setup.bash
```
Run
```
rosrun md_slam eth_dataset_manipulator -i associated.txt -k calibration.txt -o md_slam_output.bag
``` 
<h3>LiDAR</h3>

For LiDAR we provide the executable to convert [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset) rosbag into a rosbag processable by our pipeline. Once you have sourced the workspace
```
source ~/catkin_ws/devel/setup.bash
```
Run 
```
rosrun md_slam ncd_manipulator -j
```
This will generate a configuration file `lidar_configuration.json` that you can easily edit based on the LiDAR used. Make sure you add the name of the LiDAR topic used on the configuration file! Once this is done, run
```
rosrun md_slam ncd_manipulator -c lidar_configuration.json -o md_slam_output.bag input.bag
```
If you want to stick together multiple input rosbags into one then you can simply add them at the end of the command (make sure timestamps are consecutives), like
```
rosrun md_slam ncd_manipulator -c lidar_configuration.json -o md_slam_output.bag input1.bag input2.bag ...
```
NOTE: this can be used to process any LiDAR rosbag but we only tested on [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset) data. 

<h2>Paper</h2>

Thank you for citing [MD-SLAM](https://arxiv.org/abs/2203.13237) (accepted IROS 2022), if you use any of this code.
```
@article{di2022md,
  title={MD-SLAM: Multi-cue Direct SLAM},
  author={Di Giammarino, Luca and Brizi, Leonardo and Guadagnino, Tiziano and Stachniss, Cyrill and Grisetti, Giorgio},
  journal={arXiv preprint arXiv:2203.13237},
  year={2022}
}
```








