drone-panda
===========
Robot Controller
----------------

*This work is part of a larger project, it is designed to be used with the
[panda-ik](https://github.com/emmanuel-senft/panda_ik).*

Installation
------------

If not yet installed, start by [installing
ROS](http://wiki.ros.org/ROS/Installation) (tested with ROS Noetic, but
other versions might work as well).

Dependencies:
- [rviz_camera_stream](https://github.com/lucasw/rviz_camera_stream)
- [eigen3](https://eigen.tuxfamily.org/dox/)

Recommend to use [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) to compile.
Then build with:

```
> catkin build drone-panda
```

Usage
-----

### Starting the robot controller 
- `roslaunch drone_panda all.launch`
