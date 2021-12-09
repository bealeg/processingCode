#!/bin/bash

module load singularity

randport() {
    echo $(( $(echo $((0x$(xxd -l 2 -p /dev/urandom))) % $((65535 - 1024)) + 1025) ))
}
export ROS_MASTER_URI=http://localhost:$(randport)

singularity run --writable-tmpfs -B /vtti:/vtti -B /vtti/projects05:/home/autoware/data  excuseme_1.1-melodic.sif bash -c 'ROS_LOG_DIR=/tmp/roslog /example.sh '$1
