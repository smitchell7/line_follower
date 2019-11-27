# Small Group Robotics Programming Demo #

The purpose of this demo is to give high school students the opportunity to have
a successful robotics programming experience that lasts an hour. The intended
application will be implementing a line-follower program on a turtlebot.

The student will program a turtlebot to follow a line using a camera. This
problem is broken into smaller problems, some of which are already solved for
the student.

1. Line Detection *solved*
1. Camera Visualization (to watch the line) *solved*
1. Vehicle Interfacing *solved*
1. Simulation / Hardware Testing *solved*
1. **Controls**

## Running a Simulation ##

A simulation environment needs to be created before the robot can be controlled.

Open the terminal program, then run the following commands:

```bash
cd ~/line_follower_demo
catkin_make
source devel/setup.bash
roslaunch line_follower lf.launch
```

A couple of windows should pop up, showing the robot in simulation. Now we can
start coding.

## Coding ##

Open the atom program, and set it to open the project ~/line_follower_demo/src/line_follower.
Within line_follower, open scripts/control.py.

To launch the script, open a new terminal window and run the following command:

```bash
rosrun line_follower control.py
```

To kill the script, hit control-c.

## Controls ##

Read the control.py script, following the prompts in the comments.
If completed, you should be able to do the following:

1. Control the robot to drive in circles.
1. Control the robot to follow a line.
1. Control the robot to follow a line smoothly.
