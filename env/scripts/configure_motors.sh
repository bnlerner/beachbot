#!/bin/bash

# NOTE: Running these configs from OdriveResources can set the motor config.
# Ensure the motor is in IDLE axis state prior to running.
python3 ~/ODriveResources/examples/can_restore_config.py --channel can0 --node-id 0 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/front_left.json --save-config
python3 ~/ODriveResources/examples/can_restore_config.py --channel can0 --node-id 1 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/front_right.json --save-config
python3 ~/ODriveResources/examples/can_restore_config.py --channel can0 --node-id 2 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/rear_left.json --save-config
python3 ~/ODriveResources/examples/can_restore_config.py --channel can0 --node-id 3 --endpoints-json ~/beachbot/env/motor_configs/flat_endpoints.json --config ~/beachbot/env/motor_configs/beachbot-1/rear_right.json --save-config

