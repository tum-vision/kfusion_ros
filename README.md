## kfusion_ros 

ROS integration for @GerhardR/kfusion

### Installation

```
$ cd ~/fuerte_workspace
$ git clone https://github.com/tum-vision/kfusion_ros.git
$ rosmake kfusion_ros
```

Note: `~/fuerte_workspace` has to be in your `$ROS_PACKAGE_PATH`

### Running

to run kfusion_ros and a visualization of the volume in RVIZ, type:

```
$ roslaunch kfusion_ros kfusion.launch
```


### Supported ROS versions

 - fuerte
