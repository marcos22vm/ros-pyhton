#!/bin/bash
set -e

# setup ros environment
source "/tmp/catkin_ws/devel/setup.bash"
exec "$@"
