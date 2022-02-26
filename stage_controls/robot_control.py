"""Interface for robot controls."""

from __future__ import absolute_import, division, print_function

import math
import os
import sys

# Import the specific "marrtino" controls
assert "MARRTINO_APPS_HOME" in os.environ, (
    "MARRTINO_APPS_HOME should be the path of https://bitbucket.org/iocchi/marrtino_apps/")
marrtino_apps_path = os.environ["MARRTINO_APPS_HOME"] + "/program"
sys.path.append(str(marrtino_apps_path))
import robot_cmd_ros as robot


class RobotControl(object):
    """Controller class for the robot."""

    dt = 0.2  # Control frequency

    def __init__(self, max_vel, max_ang_vel):
        """Initializations."""
        robot.begin()
        robot.setMaxSpeed(max_vel, max_ang_vel)
        robot.enableObstacleAvoidance(True)
        os.system(
            "rosparam set /gradientBasedNavigation/max_vel_x %.2f" %
            max_vel
        )
        os.system(
            "rosparam set /gradientBasedNavigation/max_vel_theta %.2f" %
            max_ang_vel
        )

        # State
        self.vel = 0
        self.ang_vel = 0

        print("Initialized")

    def set_velocity(self, vel):
        """Modify the linear velocity."""
        self.vel = vel
        robot.set_speed(self.vel, self.ang_vel, self.dt, stopend=False)

    def set_ang_velocity(self, vel):
        """Modify angular velocity (degrees)."""
        rads = vel / 180 * math.pi
        self.ang_vel = rads
        robot.set_speed(self.vel, self.ang_vel, self.dt, stopend=False)

    def set_position(self, x, y):
        """Modify the position in the map."""
        state = self.get_state()
        angle = state[2]
        robot.stage_setpose(x, y, angle)

    def set_angle(self, angle):
        """Rotate the robot to a target angle.

        Assuming angle is in degrees.
        """
        robot.turn(angle, ref="ABS")

    def set_pose(self, x, y, angle):
        """Modify the pose.

        Assuming angle is in degrees.
        """
        self.set_velocity(0)
        robot.stage_setpose(x, y, angle)

    def get_state(self):
        """Retrieve configuration of the robot.

        A configuration is [x, y, theta, vel, ang_vel]
        """
        pose = robot.get_robot_pose(frame="gt")  # odom gets out of sync with set_pose
        vel = robot.get_robot_vel()    # Linear and angular
        return pose + vel
