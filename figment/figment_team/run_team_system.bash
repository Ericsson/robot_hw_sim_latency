#!/usr/bin/env bash

. ~/figment/install/setup.bash

. ~/figment/devel/setup.bash

# Run the example node
echo "Launching ARIAC example nodes for figment_team"
rosrun figment_ariac scheduler_plan
