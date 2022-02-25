"""Interface for robot controls."""

from __future__ import absolute_import, division, print_function
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

    max_vel = 0.5
    max_ang_vel = 1.0
    dt = 0.2  # Control frequency

    def __init__(self):
        """Initializations."""
        robot.begin()
        robot.setMaxSpeed(self.max_vel, self.max_ang_vel)
        robot.enableObstacleAvoidance(True)
        os.system(
            "rosparam set /gradientBasedNavigation/max_vel_x %.2f" %
            self.max_vel
        )
        os.system(
            "rosparam set /gradientBasedNavigation/max_vel_theta %.2f" %
            self.max_ang_vel
        )

        print("Initialized")

    def set_velocity(self, vel):
        """Modify the linear velocity."""
        robot.set_speed(vel, 0, self.dt, stopend=False)

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

        A configuration is [x, y, theta, vel, ?]
        """
        pose = robot.get_robot_pose(frame="gt")  # odom gets out of sync with set_pose
        vel = robot.get_robot_vel()    # Linear and angular
        return pose + [vel[0]] + [0]
