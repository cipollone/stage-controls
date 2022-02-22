# Stage-controls

This package allows to communicate with a running instance of [ros-stage-rl](https://github.com/cipollone/ros-stage-rl).
Together, these two packages define a communicatio protocol.
Branches in each repository are meant to be able to communicate correctly, because they must
agree on the observation and action spaces.

## Use

Run 


## Install

This package can be installed both on python 2 and 3:

    pip install .

Or, install for development with poetry:

    poetry install


## Run
Run main script with:

    python -m stage_controls
