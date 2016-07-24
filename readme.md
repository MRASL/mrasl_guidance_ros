# MRASL Guidance ros
An attempt at making a better DJI Guidance package.

## Software Architecture
The `GuidanceManager` object is implemented as a singleton however, before requesting an instance, you **must** set the the nodehandle using `GuidanceManager::setNodeHandle`. Since the Guidance libraries are written in C, having the GuidanceManager be a singleton, helps accessing the required functions in the C callback upon data reception. The GuidanceManager class can then be used in a simple trampoline like in `guidance_node.cpp` or integrated in another node.

### Timestamps
For now, timestamps are done relative to the ros clock whenever a message is received and parsed. We will need to figure out some sort of time sync thing.

## Running
At runtime, the GuidanceManager advertises all the possible topics for now. Maybe later, I can do better checking using parameters and stuff. Do note that due to the FPGA constraints in the Guidance, only two depth images can be computed at a time.

## Camera calibration
For some odd reason, DJI decided not to let us request the camera intrinsics but only the extrinsics (stereo calib). Camera info manager infrastructure is in place for you to use the standard ROS camera calibration nodes/results.

## Published topics
* `~/imu`
* `~/cam[i]/depth`
* `~/cam[i]/left`
* `~/cam[i]/right`
* `~/cam[i]/disparity`
* `~/ultrasonic`
* `~/velocity`
* `~/position`
* `~/obstacle_distance`

## Reference frames
Note that depth images are provided in the left camera reference frame, that's why there's a missing piece of image on the left of depth images.
* `cam[i]`

## GPU usage
Seems like the GPU based stereo block matching is too imprecise. So 1 pair is
CPU block matched.

# Future work
* Actually use dynamic reconfigure
* Make this a nodelet somehow
