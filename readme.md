# MRASL Guidance ros
An attempt at making a better DJI Guidance package.

## Software Architecture
The `GuidanceManager` object is implemented as a singleton however, before requesting an instance, you **must** set the the nodehandle using `GuidanceManager::setNodeHandle`. Since the Guidance libraries are written in C, having the GuidanceManager be a singleton, helps accessing the required functions in the C callback upon data reception. The GuidanceManager class can then be used in a simple trampoline like in `guidance_node.cpp` or integrated in another node.

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

# Future work
* Add services to programatically enable or disable streams
* Actually use dynamic reconfigure
* Make this a nodelet somehow
