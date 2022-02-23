from __future__ import absolute_import, division, print_function
from builtins import input
import argparse
import random

from . import bridge
from .tools import QuitWithResources


def main():

    # Arguments
    parser = argparse.ArgumentParser(
        description="Allows to control the robot from a remote connection"
    )
    parser.add_argument(
        "action_set", type=int, help="Choose one of the action sets available."
    )
    parser.add_argument(
        "-t", "--test", action="store_true",
        help="Test actions and returned states (Implies -v)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Verbose mode: prints actions and states."
    )

    args = parser.parse_args()

    if args.test:
        test_actions(args.action_set)  # Never ends

    else:
        # Start and loop on bridge connector
        bridge.Connector(
            args.action_set,
            verbose=args.verbose,
        ).run()  # Never ends


def test_actions(action_set):
    """Just test actions and their effect."""

    input("Warning: testing random actions. Are you on a simulator? (Y) ")

    # Ros
    stage_controls = bridge.StageControls(action_set, verbose=True)

    # Flags
    quit = False
    def do_quit():
        nonlocal quit
        quit = True
    QuitWithResources.add("quit-test", do_quit)

    # Reset
    stage_controls.act(-1)

    while not quit:
        # These functions should print because verbose=True
        action = random.randint(-1, stage_controls.n_actions-1)
        stage_controls.act(action)
        stage_controls.get_state()


if __name__ == "__main__":
    main()
