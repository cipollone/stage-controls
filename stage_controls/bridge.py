"""Communicate with ROS Stage environment from sockets.

It will open a pair of sockets:
  - One is used to receive actions that should be forwarded to the robot.
  - Another is used to send back the observations/state of the robot.
Currently, the other end of the communication has been implemented in:
    cipollone/ros-stage-rl
"""

from __future__ import absolute_import, division, print_function
from builtins import input
from builtins import str
from builtins import object

import numpy as np

from .streaming import Sender, Receiver
from .robot_control import RobotControl


class StageControls(object):
    """Define actions and observations.

    self.actions defines a list of available actions. Positive indexes are
    mapped to these actions based on their position on this list.
    action_set is an index that allows to experiment with different definitions
    of actions.
    """

    def __init__(self, action_set, verbose=False):
        """Initialize."""
        self._verbose = verbose

        # Choose parameters based on selection
        if action_set == 1:

            # Parameters
            self._acceleration = 0.2
            self._ang_velocity = 40   # In degrees
            self._max_linear_vel = 0.5
            self._start_state = [0, 0, 0, 0]  # [x,y,th,vel]

            # Actions definitions
            self.actions = [
                self._action_faster,
                self._action_slower,
                self._action_turn1,
                self._action_turn2,
                self._action_interact,
            ]

        elif action_set == 2:
            raise NotImplementedError(
                "Further action and parameters definition here"
            )

        else:
            raise ValueError("Not a valid action_set")

        self.n_actions = len(self.actions)

        # Other (non-RL) signals
        self.signals = {
            -1: self._signal_reset,
        }

        # Init vars
        self.state = self._start_state
        self._current_angle = self.state[2]
        self._current_vel = self.state[3]
        assert self._current_vel == 0

        # Start
        self.control = RobotControl()
        self._signal_reset()

    def _saturate_velocities(self):
        """Ensure max and min in velocities."""
        self._current_vel = max(0, min(self._current_vel, self._max_linear_vel))

    def _360_angle(self):
        """Just limit angles."""
        self._current_angle = self._current_angle % 360

    def _action_faster(self):
        self._current_vel += self._acceleration
        self._saturate_velocities()
        return self.control.set_velocity(self._current_vel)

    def _action_slower(self):
        self._current_vel -= self._acceleration
        self._saturate_velocities()
        return self.control.set_velocity(self._current_vel)

    def _action_turn1(self):
        self._current_angle += self._ang_velocity
        self._360_angle()
        return self.control.set_angle(self._current_angle)

    def _action_turn2(self):
        self._current_angle -= self._ang_velocity
        self._360_angle()
        return self.control.set_angle(self._current_angle)

    def _action_interact(self):
        # NOTE: noop for now, just connect this to an interaction routine
        return

    def _signal_reset(self):
        """Reset the environment."""
        self.state = self._start_state
        self.control.set_pose(*self.state[:3])
        self.control.set_velocity(self.state[3])

    def act(self, action):
        """Executes action number i (a positive index) or a signal."""

        # Check
        if action >= self.n_actions and action not in self.signals:
            raise RuntimeError(
                str(action) + " is not an action nor a signal.")

        # Signals
        if action in self.signals:
            if self._verbose:
                print("Received: ", self.signals[action].__name__)
            self.signals[action]()
            return

        # Actions
        if self._verbose:
            print("Action:", "{0:>2}".format(action), end=", ")
        return self.actions[action]()

    def get_state(self):
        """Computes and returns the state vector."""
        self.state = self.control.get_state()
        if self._verbose:
            print("State:", np.array(self.state, dtype=np.float32))
        return self.state


class Connector(object):
    """Connections to operate StageControls.

    This maintains a socket communication with a remote agent that sends
    actions and receives states.
    """

    # Communication protocol
    actions_port = 30005
    states_port = 30006
    state_msg_len = 16    # a numpy vector of 4 float32
    action_msg_len = 4    # a numpy positive scalar of type int32

    class ActionReceiver(Receiver):
        """Just a wrapper that deserializes actions."""

        def receive(self):
            """Return an action received.

            :return: a scalar int that identifies an action (no bound checks)..
            """

            # Receive
            buff = Receiver.receive(self, wait=True)

            # Deserialize
            assert len(buff) == Connector.action_msg_len, (
                "Expected msg len {}, got {}".format(
                    Connector.action_msg_len, len(buff)))
            array = np.frombuffer(buff, dtype=np.int32)
            return array.item()

    class StateSender(Sender):
        """Just a wrapper that serializes states."""

        def send(self, state):
            """Send a state.

            :param state: a numpy array.
            """
            # Serialize
            buff = np.array(state, dtype=np.float32).tobytes()
            assert len(buff) == Connector.state_msg_len, (
                "Expected msg len {}, got {}".format(
                    Connector.state_msg_len, len(buff)))
            # Send
            Sender.send(self, buff)

    def __init__(self, action_set, verbose=False):
        """Initialize."""

        # StageControls
        self.stage_controls = StageControls(action_set, verbose=verbose)

        # Initialize connections
        self.state_sender = Connector.StateSender(
            msg_length=self.state_msg_len, port=self.states_port, wait=True,
        )
        self.action_receiver = Connector.ActionReceiver(
            msg_length=self.action_msg_len, ip="localhost",
            port=self.actions_port, wait=True,
        )

        # Connect now
        self.state_sender.start()
        print("> Serving states on", self.state_sender.server.server_address)
        print(
            "> Connecting to ",
            self.action_receiver.ip, ":",
            self.action_receiver.port,
            " for actions. (pause)",
            sep="",
            end=" ",
        )
        input()
        self.action_receiver.start()

    def run(self):
        """Loop: continuously execute actions and return states.

        This continuously waits for incoming actions in action_receiver,
        it executes them on StageControls and returns the next
        observation/state.
        This never terminates: use CTRL-C.
        """
        while True:

            # Get an action from the agent
            action = self.action_receiver.receive()

            # Move robot
            self.stage_controls.act(action)

            # Return a state
            state = self.stage_controls.get_state()
            self.state_sender.send(state)
