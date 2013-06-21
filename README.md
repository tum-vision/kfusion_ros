## kfusion_ros 

ROS integration for @GerhardR/kfusion

### Installation

```
$ cd ~/fuerte_workspace
$ git clone https://github.com/tum-vision/kfusion_ros.git
$ rosmake kfusion_ros
```

Note: `~/fuerte_workspace` has to be in your `$ROS_PACKAGE_PATH`. If your CUDA SDK is not installed in the default location set the `$CUDA_TOOLKIT_ROOT_DIR` environment variable to its path, i.e., `export CUDA_TOOLKIT_ROOT_DIR=/path/to/cuda/sdk`

### Running

to run kfusion_ros and a visualization of the volume in RVIZ, type:

```
$ roslaunch kfusion_ros kfusion.launch
```


### Supported ROS versions

 - fuerte
