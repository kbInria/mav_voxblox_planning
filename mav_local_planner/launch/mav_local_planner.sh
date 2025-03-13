#!/bin/bash

# Expecting these two rosparam to be already set
NUM_DRONES=$(rosparam get /number_of_drones)
DRONE_NAME=$(rosparam get /drone_name)

echo "Creating $NUM_DRONES nbv selector"

# Acquiring passed arguments
AVOID_COLLISIONS=$1
COMMAND_DT=$2
V_MAX=$3
A_MAX=$4
YAW_RATE_MAX=$5
SAMPLING_DT=$6
IDLE_QUEUE=$7
MIN_SPEED=$8

# Loop through the number of drones and launch each
for ((i=0; i<NUM_DRONES; i++)); do
    namespace="${DRONE_NAME}_${i}"
    roslaunch mav_local_planner mav_local_planner.launch namespace:=$namespace avoid_collisions:=$AVOID_COLLISIONS \
        command_publishing_dt:=$COMMAND_DT v_max:=$V_MAX a_max:=$A_MAX yaw_rate_max:=$YAW_RATE_MAX sampling_dt:=$SAMPLING_DT \
        idle_queue_size:=$IDLE_QUEUE min_speed_treshold:=$MIN_SPEED &
done

# Wait for all launched processes to complete
wait