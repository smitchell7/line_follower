# Small Group Robotics Programming Demo #

The purpose of this demo is to give high school students the opportunity to have
a successful robotics programming experience that lasts an hour. The intended
application will be implementing a line-follower program on a turtlebot.

The premise of this means that the work to interface with the robot should be as
simple as possible. To achieve this, I've separated the demo proposal into the
following:

1. Experience
2. Backend

## Experience ##

The student will program a turtlebot to follow a line using a camera. This
problem is broken into smaller problems, some of which are already solved for
the student.

1. Line Detection *solved*
1. Camera Visualization (to watch the line) *solved*
1. Vehicle Interfacing *solved*
1. Simulation / Hardware Testing *solved*
1. Controls

### Controls ###

First the students will learn the meaning of the control commands, after which
they'll create a line follower.

#### Understanding /cmd_vel ####

1. Make the robot drive in a straight line.
1. Make the robot drive in a large clockwise circle.
1. Make the robot drive in a small counter-clockwise circle.

#### Line follower ####

##### Simple #####

Every time the line is measured, it sends a message with how far to the side the
detected line is. We can use that information to make the robot react to the
line.

1. When the line is to the left, make the robot turn right.
1. When the line is to the right, make the robot turn left.
1. If center, go straight.

What if the line isn't visible?

##### Proportional Control #####

1. Can we make an equation that makes us turn right a little if the line is a
   little to the left?
1. How can this be improved?

