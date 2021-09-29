#!/bin/bash

cd /home/gbeale/autoware.ai/
source install/setup.bash

cd src/processingCode/

# for FILE in SmallBatch/*
for FILE in SmallBatch/*
do
	out="$(basename $FILE)"
	echo $out
	roslaunch $FILE
done
