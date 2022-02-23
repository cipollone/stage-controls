"""Interface for robot controls."""

from __future__ import absolute_import, division, print_function


class RobotControl(object):
    """Controller class for the robot."""

    def __init__(self):
        """Initializations."""
        # TODO: from old repo; update these
        #robot.begin()
        #robot.setMaxSpeed(self._max_tv, self._max_rv)
        #robot.enableObstacleAvoidance(True)
        #os.system("rosparam set /gradientBasedNavigation/max_vel_x %.2f" %
        #    self._max_tv)
        #os.system("rosparam set /gradientBasedNavigation/max_vel_theta %.2f" %
        #    self._max_rv)

        print("Initialized")

    def set_velocity(self, vel):
        """Modify the linear velocity."""
        # TODO: update below
        print("Setting vel ", vel)

    def set_position(self, x, y):
        """Modify the position in the map."""
        # TODO: update below
        print("Setting x, y ", x, y)

    def set_angle(self, angle):
        """Rotate the robot to a target angle.

        Assuming angle is in degrees.
        """
        # TODO: update below
        print("Setting angle", angle)

    def set_pose(self, x, y, angle):
        """Modify the pose.

        Assuming angle is in degrees.
        """
        self.set_velocity(0)
        self.set_position(x, y)
        self.set_angle(angle)

    def get_state(self):
        """Retrieve configuration of the robot.

        A configuration is [x, y, theta, vel, persondetected?]
        """
        # TODO: fill this and remove below
        #p = robot.getRobotPose(frame='gt')
        #v = robot.getRobotVel()
        # person detected?
        return [1, 2, 3, 4, 0]
